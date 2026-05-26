"""tsdf_to_vxw — convert a PCL .pcd point cloud into a .vxw world via a
truncated-signed-distance (TSDF-style) intermediate that smooths the surface.

Strategy chosen: **KDTree-with-truncation surface band**.

Rationale: Open3D's `ScalableTSDFVolume.integrate()` requires depth + intrinsics
which we do not have (we get a raw fused point cloud from SLAM). Poisson
surface reconstruction is the alternative, but on the sparse SLAM scans in
this repo (~30k pts) Poisson regularly produces degenerate / over-extended
hulls that "balloon" into empty space.

So we use a robust simpler proxy that delivers the same end goal ("smoother
surface than naive point-to-voxel"):

  1. Build an Open3D PointCloud from the raw points.
  2. Voxel-down-sample at `tsdf_voxel_size` so we get a roughly uniform
     surface point density (this is the "implicit isosurface samples"
     analogue of TSDF zero-crossings).
  3. Estimate normals (useful for any downstream consumer; also keeps the
     API in line with the documented TSDF pipeline).
  4. Walk a 3D grid at `--voxel-size` resolution over the bounding box,
     marking a voxel occupied iff the down-sampled point cloud has a sample
     within `surface_threshold` of the voxel centre (KDTree query, fully
     vectorised — no Python loop over voxels).

This is morphologically equivalent to dilating the down-sampled surface by
`surface_threshold`, which is exactly what a TSDF "narrow band" gives you:
it fills small holes between samples and produces a thicker, smoother
surface shell.

Usage:
    pixi run python m3_adapter/tsdf_to_vxw.py \\
        E:/aros_slam_ws/lightning_lm_foxy/data/test_compare_baseline/global.pcd \\
        out/baseline_tsdf.vxw \\
        --voxel-size 0.10 --swap-yz --compression gzip
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
import uuid
from collections import Counter
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))
sys.path.insert(0, str(_PROJECT_ROOT / "m3_adapter"))

import vxw_format as vxw  # noqa: E402
from pcd_to_vxw import load_pcd_xyz, ros_zup_to_vxw_yup  # noqa: E402

_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
log = logging.getLogger("tsdf_to_vxw")


_COMPRESSION_MAP = {
    "raw": vxw.Compression.RAW,
    "gzip": vxw.Compression.GZIP,
    "zstd": vxw.Compression.ZSTD,
}


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"tsdf_{ts}.log"
    fmt = logging.Formatter(
        "[%(asctime)s] %(levelname)-5s %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )
    log.setLevel(logging.INFO)
    log.handlers.clear()
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setFormatter(fmt)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    log.addHandler(fh)
    log.addHandler(sh)
    return log_path


def _log_bbox(prefix: str, xyz: np.ndarray) -> None:
    mn = xyz.min(0)
    mx = xyz.max(0)
    log.info(
        "%s X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]  (extent %.1f x %.1f x %.1f m)",
        prefix,
        mn[0], mx[0], mn[1], mx[1], mn[2], mx[2],
        mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2],
    )


def build_palette() -> vxw.Palette:
    return vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=("empty",)),
            vxw.Material(
                id=1,
                name="concrete",
                color_rgb=(180, 180, 180),
                flags=("solid", "destructible"),
                density=2400.0,
                hardness=30.0,
            ),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0, name="unknown", default_material=1),
            vxw.SemanticClass(id=1, name="surface", default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 178, 175)],
    )


def downsample_and_normals(
    xyz: np.ndarray,
    tsdf_voxel_size: float,
) -> tuple[np.ndarray, int]:
    """Voxel-down-sample with Open3D and estimate normals.

    Returns:
        (downsampled_xyz_Nx3_float64, n_input_points)
    """
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    n_in = len(pcd.points)

    pcd_ds = pcd.voxel_down_sample(voxel_size=tsdf_voxel_size)
    # Normals: not strictly required by the KDTree path below, but estimating
    # them produces the same "surface-aware" intermediate any TSDF pipeline
    # would emit and lets downstream code consume them later if desired.
    radius = max(tsdf_voxel_size * 2.0, 0.05)
    pcd_ds.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
    )

    ds_xyz = np.asarray(pcd_ds.points, dtype=np.float64)
    return ds_xyz, n_in


def voxelize_surface_band(
    ds_xyz: np.ndarray,
    voxel_size: float,
    surface_threshold: float,
    chunk_extent: int,
    compression: vxw.Compression,
    material_id: int,
) -> tuple[dict, tuple[int, int, int], tuple[int, int, int], int]:
    """Mark every voxel within `surface_threshold` of any downsampled point.

    Uses scipy's cKDTree for an O((N+M) log N) nearest-neighbour query — no
    Python loops over voxels.

    Returns:
        (chunks_dict, bounds_min_chunk, bounds_max_chunk, total_occupied_voxels)
    """
    from scipy.spatial import cKDTree

    # 1) Compute bounding box of voxels we need to consider. Expand by
    #    surface_threshold + half-voxel so we catch voxels whose centres
    #    sit just outside the cloud bbox but whose dilation band reaches in.
    pad = surface_threshold + 0.5 * voxel_size
    mn = ds_xyz.min(axis=0) - pad
    mx = ds_xyz.max(axis=0) + pad

    # Voxel-index ranges (inclusive).
    vi_min = np.floor(mn / voxel_size).astype(np.int64)
    vi_max = np.floor(mx / voxel_size).astype(np.int64)
    nx, ny, nz = (vi_max - vi_min + 1).tolist()

    n_candidate = int(nx) * int(ny) * int(nz)
    log.info(
        "      candidate voxel grid %d x %d x %d = %d cells",
        nx, ny, nz, n_candidate,
    )

    if n_candidate <= 0:
        raise RuntimeError("empty candidate voxel grid (degenerate bounding box)")

    # 2) Build KDTree on the down-sampled surface points.
    tree = cKDTree(ds_xyz)

    # 3) Generate every voxel CENTRE in the candidate grid (vectorised).
    #    To bound memory on very large worlds, process in z-slabs.
    occupied_voxels: list[np.ndarray] = []
    # Approx memory: nx*ny*slab_z * 3 * 8 bytes for centres. Keep < ~256 MB.
    max_cells_per_slab = max(1, (1 << 24) // max(1, nx * ny))
    slab_z = min(nz, max(1, max_cells_per_slab))

    ax = (vi_min[0] + np.arange(nx, dtype=np.int64))
    ay = (vi_min[1] + np.arange(ny, dtype=np.int64))

    total_occ = 0
    for z0 in range(0, nz, slab_z):
        z1 = min(nz, z0 + slab_z)
        az = (vi_min[2] + np.arange(z0, z1, dtype=np.int64))

        gx, gy, gz = np.meshgrid(ax, ay, az, indexing="ij")
        idx = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        # Voxel centre in metres.
        centres = (idx.astype(np.float64) + 0.5) * voxel_size
        dists, _ = tree.query(centres, k=1, workers=-1)
        mask = dists <= surface_threshold
        if mask.any():
            occupied_voxels.append(idx[mask])
        total_occ += int(mask.sum())

    if total_occ == 0:
        raise RuntimeError(
            "no voxels were marked occupied — surface_threshold too small for "
            "this cloud density?"
        )

    vc_unique = np.unique(np.concatenate(occupied_voxels, axis=0), axis=0)

    # 4) Chunkize (same pattern as pcd_to_vxw.voxelize_and_group).
    cc = np.floor_divide(vc_unique, chunk_extent).astype(np.int64)
    local = (vc_unique - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)

    chunks: dict = {}
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        locs = local[mask]
        arr = np.zeros(
            (chunk_extent, chunk_extent, chunk_extent), dtype=vxw.VOXEL_DTYPE
        )
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = material_id
        arr["semantic_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = 1
        ckey = tuple(int(x) for x in chunk_keys[chunk_id])
        chunks[ckey] = vxw.Chunk(
            coord=ckey,
            voxels=arr,
            encoding=vxw.Encoding.RLE,
            compression=compression,
        )

    bounds_min = tuple(int(x) for x in cc.min(axis=0))
    bounds_max = tuple(int(x) + 1 for x in cc.max(axis=0))
    return chunks, bounds_min, bounds_max, len(vc_unique)


def main() -> None:
    log_path = _setup_logging()

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_pcd", type=Path)
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument(
        "--voxel-size", type=float, default=0.10,
        help="metres per voxel in the output .vxw (default 0.10)",
    )
    ap.add_argument(
        "--tsdf-voxel-size", type=float, default=None,
        help="metres per voxel for the TSDF/down-sample stage "
             "(default: same as --voxel-size)",
    )
    ap.add_argument(
        "--tsdf-trunc", type=float, default=None,
        help="TSDF truncation distance in metres (default: 4 x --voxel-size). "
             "Currently informational only in the KDTree strategy.",
    )
    ap.add_argument("--chunk-extent", type=int, default=32)
    ap.add_argument(
        "--compression",
        choices=["raw", "gzip", "zstd"],
        default="gzip",
        help="chunk payload compression (Godot can't decompress ZSTD).",
    )
    ap.add_argument(
        "--swap-yz", action="store_true",
        help="convert ROS Z-up frame → vxw Y-up frame (same rotation as pcd_to_vxw).",
    )
    ap.add_argument(
        "--surface-threshold", type=float, default=None,
        help="abs(SDF) <= this is 'on the surface' (default: 0.5 x --voxel-size).",
    )
    ap.add_argument(
        "--material-id", type=int, default=1,
        help="material id written to occupied voxels (default 1 = concrete).",
    )
    ap.add_argument(
        "--source-slam", default="lightning_lm_foxy",
        help="SLAM system name written to manifest.source.slam_system",
    )
    args = ap.parse_args()

    voxel_size = float(args.voxel_size)
    tsdf_voxel_size = float(args.tsdf_voxel_size) if args.tsdf_voxel_size else voxel_size
    tsdf_trunc = float(args.tsdf_trunc) if args.tsdf_trunc else 4.0 * voxel_size
    surface_threshold = (
        float(args.surface_threshold)
        if args.surface_threshold is not None
        else 0.5 * voxel_size
    )

    log.info("session start  log=%s", log_path)
    log.info(
        "config  input=%s  output=%s  voxel_size=%.3fm  tsdf_voxel=%.3fm  "
        "tsdf_trunc=%.3fm  surf_thresh=%.3fm  chunk_extent=%d  "
        "compression=%s  swap_yz=%s  material_id=%d",
        args.input_pcd, args.output_vxw, voxel_size, tsdf_voxel_size,
        tsdf_trunc, surface_threshold, args.chunk_extent, args.compression,
        args.swap_yz, args.material_id,
    )

    if not args.input_pcd.is_file():
        log.error("input file not found: %s", args.input_pcd)
        sys.exit(1)

    # [1/5] Load
    log.info("[1/5] loading %s (%d bytes)",
             args.input_pcd, args.input_pcd.stat().st_size)
    t0 = time.perf_counter()
    xyz = load_pcd_xyz(args.input_pcd)
    log.info("      %d points in %.2fs", len(xyz), time.perf_counter() - t0)
    _log_bbox("      raw bbox     ", xyz)

    if len(xyz) < 100:
        log.error("input has only %d points (<100), refusing to voxelise", len(xyz))
        sys.exit(1)

    if args.swap_yz:
        xyz = ros_zup_to_vxw_yup(xyz)
        _log_bbox("      Y-up bbox    ", xyz)

    # [2/5] TSDF-proxy (downsample + normals)
    log.info("[2/5] TSDF-proxy: voxel_down_sample(%.3fm) + estimate_normals",
             tsdf_voxel_size)
    t0 = time.perf_counter()
    ds_xyz, n_in = downsample_and_normals(xyz, tsdf_voxel_size)
    log.info("      %d in -> %d down-sampled points in %.2fs  (ratio %.2fx)",
             n_in, len(ds_xyz), time.perf_counter() - t0,
             n_in / max(1, len(ds_xyz)))

    if len(ds_xyz) == 0:
        log.error("down-sampling produced 0 points (tsdf_voxel_size too coarse?)")
        sys.exit(1)

    # [3/5] Voxelise surface band
    log.info(
        "[3/5] voxelising surface band  voxel_size=%.3fm  surf_thresh=%.3fm  "
        "chunk_extent=%d  compression=%s",
        voxel_size, surface_threshold, args.chunk_extent, args.compression,
    )
    t0 = time.perf_counter()
    chunks, bmin, bmax, total_voxels = voxelize_surface_band(
        ds_xyz, voxel_size, surface_threshold,
        args.chunk_extent, _COMPRESSION_MAP[args.compression],
        args.material_id,
    )
    log.info(
        "      %d chunks  %d occupied voxels  bounds_min=%s bounds_max=%s  "
        "in %.2fs",
        len(chunks), total_voxels, bmin, bmax, time.perf_counter() - t0,
    )

    occ_per_chunk = Counter()
    for c in chunks.values():
        occ_per_chunk[int((c.voxels["material_id"] != 0).sum())] += 1
    log.info("      voxels-per-chunk histogram (top 5): %s",
             occ_per_chunk.most_common(5))

    # [4/5] Write
    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=voxel_size,
        chunk_extent=args.chunk_extent,
        bounds_chunks_min=bmin,
        bounds_chunks_max=bmax,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system=args.source_slam,
        source_sensor="livox",
        raw_data_hash=f"sha-skip:{args.input_pcd.name}",
    )
    palette = build_palette()
    world = vxw.World(manifest=manifest, palette=palette, chunks=chunks)

    log.info("[4/5] writing %s", args.output_vxw)
    t0 = time.perf_counter()
    vxw.write_world(args.output_vxw, world)
    log.info("      wrote in %.2fs", time.perf_counter() - t0)

    # [5/5] Verify
    log.info("[5/5] verifying round-trip")
    t0 = time.perf_counter()
    loaded = vxw.read_world(args.output_vxw)
    loaded_occ = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in loaded.chunks.values()
    )
    log.info(
        "      loaded back %d chunks  %d voxels  in %.2fs",
        len(loaded.chunks), loaded_occ, time.perf_counter() - t0,
    )
    if loaded_occ != total_voxels:
        log.error("round-trip lost voxels! expected=%d got=%d",
                  total_voxels, loaded_occ)
        sys.exit(2)
    if len(loaded.chunks) != len(chunks):
        log.error("round-trip lost chunks! expected=%d got=%d",
                  len(chunks), len(loaded.chunks))
        sys.exit(2)

    total_size = sum(
        p.stat().st_size for p in args.output_vxw.rglob("*") if p.is_file()
    )
    pcd_size = args.input_pcd.stat().st_size
    log.info(
        "RESULT  pcd_bytes=%d  vxw_bytes=%d  ratio=%.4f  voxels_per_chunk=%.1f",
        pcd_size, total_size, total_size / pcd_size,
        total_voxels / max(1, len(chunks)),
    )
    log.info("DONE    written=%s  log=%s", args.output_vxw, log_path)


if __name__ == "__main__":
    main()
