"""dbscan_to_vxw — convert a PCL .pcd point cloud into a .vxw world where each
DBSCAN-clustered geometric object is rendered with its own material id.

Strategy: Path C step — geometric clustering as a Hydra-free proxy for semantic
segmentation. We voxelise the cloud, then run sklearn's DBSCAN on the *voxel
centres* (not raw points — voxelisation already shrinks the dataset 10-100x so
DBSCAN finishes in well under a second on room-scale inputs). Each cluster id
is assigned a distinct material id (with auto-generated HSV palette colours);
DBSCAN's noise label (-1) maps to a dedicated "noise" material.

Output: every voxel carries a cluster-derived material_id, so in Godot you can
visually distinguish furniture / walls / floor segments just by colour.

Usage:
    pixi run python m3_adapter/dbscan_to_vxw.py \\
        E:/aros_slam_ws/lightning_lm_foxy/data/test_compare_baseline/global.pcd \\
        out/baseline_dbscan.vxw \\
        --voxel-size 0.10 --swap-yz --eps 0.15 --min-samples 5 --compression gzip
"""

from __future__ import annotations

import argparse
import colorsys
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
from m3_adapter.common import load_pcd_xyz, ros_zup_to_vxw_yup  # noqa: E402

_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
log = logging.getLogger("dbscan_to_vxw")


_COMPRESSION_MAP = {
    "raw": vxw.Compression.RAW,
    "gzip": vxw.Compression.GZIP,
    "zstd": vxw.Compression.ZSTD,
}


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"dbscan_{ts}.log"
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


# ---------------------------------------------------------------------------
# DBSCAN over voxel centres
# ---------------------------------------------------------------------------

def voxelize_unique(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """Return the unique integer voxel indices (N,3 int64) that contain points."""
    vc = np.floor(xyz / voxel_size).astype(np.int64)
    return np.unique(vc, axis=0)


def cluster_voxel_centres(
    vc_unique: np.ndarray,
    voxel_size: float,
    eps: float,
    min_samples: int,
) -> np.ndarray:
    """Run DBSCAN on the voxel-centre positions in metres.

    Returns: labels array of shape (N,) — integer cluster ids (>=0) or -1 for
    noise, one per row of vc_unique.
    """
    from m3_adapter.clustering import dbscan_labels

    centres = (vc_unique.astype(np.float64) + 0.5) * voxel_size
    return dbscan_labels(centres, eps, min_samples).astype(np.int64)


# ---------------------------------------------------------------------------
# Palette construction — N HSV-evenly-spaced cluster materials
# ---------------------------------------------------------------------------

def _hsv_rgb(hue: float, sat: float = 0.85, val: float = 0.95) -> tuple[int, int, int]:
    r, g, b = colorsys.hsv_to_rgb(hue % 1.0, sat, val)
    return (int(round(r * 255)), int(round(g * 255)), int(round(b * 255)))


def build_cluster_palette(n_clusters: int, include_noise: bool) -> vxw.Palette:
    """Build a palette with one material per cluster (HSV-evenly spaced).

    Material layout:
        id 0 = air
        id 1 = noise (only if include_noise; mid-grey)
        ids 2..2+n_clusters-1 = clusters 0..n-1
    If include_noise is False, cluster 0 starts at id 1 (no noise material).

    Caps to 254 cluster materials (255 incl. noise; 256 incl. air).
    """
    materials = [vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=("empty",))]
    base_id = 1
    if include_noise:
        materials.append(
            vxw.Material(
                id=1,
                name="noise",
                color_rgb=(110, 110, 110),
                flags=("solid", "destructible"),
                roughness=0.9,
            )
        )
        base_id = 2

    max_clusters = 256 - base_id  # leave room
    if n_clusters > max_clusters:
        log.warning(
            "DBSCAN produced %d clusters but palette holds max %d; extra "
            "clusters will all share the last material id",
            n_clusters, max_clusters,
        )
        n_clusters_palette = max_clusters
    else:
        n_clusters_palette = n_clusters

    for k in range(n_clusters_palette):
        hue = k / max(1, n_clusters_palette)
        rgb = _hsv_rgb(hue)
        materials.append(
            vxw.Material(
                id=base_id + k,
                name=f"cluster_{k}",
                color_rgb=rgb,
                flags=("solid", "destructible"),
                roughness=0.8,
            )
        )

    semantic_classes = [
        vxw.SemanticClass(id=0, name="unknown", default_material=1),
        vxw.SemanticClass(id=1, name="cluster", default_material=base_id),
    ]
    return vxw.Palette(
        materials=materials,
        semantic_classes=semantic_classes,
        color_lut=[m.color_rgb for m in materials],
    )


def labels_to_material_ids(
    labels: np.ndarray,
    include_noise: bool,
    n_clusters: int,
) -> np.ndarray:
    """Map DBSCAN labels (-1 noise, 0..n-1 clusters) to vxw material ids.

    With include_noise=True: noise → 1, cluster k → 2+k.
    With include_noise=False: noise → 0 (will be filtered out as air),
                              cluster k → 1+k.
    Clipped to the palette's max material id (see build_cluster_palette).
    """
    base_id = 2 if include_noise else 1
    max_clusters = 256 - base_id
    mids = np.empty_like(labels, dtype=np.uint8)
    noise_mask = labels < 0
    if include_noise:
        mids[noise_mask] = 1
    else:
        mids[noise_mask] = 0
    # Cluster rows
    cluster_rows = ~noise_mask
    clipped = np.minimum(labels[cluster_rows], max_clusters - 1)
    mids[cluster_rows] = (base_id + clipped).astype(np.uint8)
    _ = n_clusters  # only used by caller for sizing the palette
    return mids


# ---------------------------------------------------------------------------
# Chunk grouping (mirror pcd_to_vxw.voxelize_and_group, but with per-voxel mid)
# ---------------------------------------------------------------------------

def chunkize(
    vc_unique: np.ndarray,
    material_ids: np.ndarray,
    chunk_extent: int,
    compression: vxw.Compression,
) -> tuple[dict, tuple[int, int, int], tuple[int, int, int], int]:
    """Group voxels by chunk and write them with per-voxel material ids.

    Voxels whose material_id == 0 are skipped (air).
    Returns: (chunks, bounds_min, bounds_max, total_occupied)
    """
    nonzero = material_ids != 0
    vc_unique = vc_unique[nonzero]
    material_ids = material_ids[nonzero]

    if len(vc_unique) == 0:
        raise RuntimeError("no occupied voxels after cluster→material mapping")

    cc = np.floor_divide(vc_unique, chunk_extent).astype(np.int64)
    local = (vc_unique - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)

    chunks: dict = {}
    total_occupied = 0
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        locs = local[mask]
        mids = material_ids[mask]
        arr = np.zeros(
            (chunk_extent, chunk_extent, chunk_extent), dtype=vxw.VOXEL_DTYPE
        )
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = mids
        arr["semantic_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = 1  # "cluster"
        ckey = tuple(int(x) for x in chunk_keys[chunk_id])
        chunks[ckey] = vxw.Chunk(
            coord=ckey,
            voxels=arr,
            encoding=vxw.Encoding.RLE,
            compression=compression,
        )
        total_occupied += int(mask.sum())

    bounds_min = tuple(int(x) for x in cc.min(axis=0))
    bounds_max = tuple(int(x) + 1 for x in cc.max(axis=0))
    return chunks, bounds_min, bounds_max, total_occupied


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    log_path = _setup_logging()

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_pcd", type=Path)
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument(
        "--voxel-size", type=float, default=0.10,
        help="metres per voxel (default 0.10)",
    )
    ap.add_argument(
        "--eps", type=float, default=None,
        help="DBSCAN eps in metres (default: 1.5 x --voxel-size)",
    )
    ap.add_argument(
        "--min-samples", type=int, default=5,
        help="DBSCAN min_samples (default 5)",
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
        help="convert ROS Z-up frame → vxw Y-up frame.",
    )
    ap.add_argument(
        "--drop-noise", action="store_true",
        help="discard DBSCAN noise points (default: keep them as 'noise' material).",
    )
    ap.add_argument(
        "--source-slam", default="lightning_lm_foxy",
        help="SLAM system name written to manifest.source.slam_system",
    )
    args = ap.parse_args()

    voxel_size = float(args.voxel_size)
    eps = float(args.eps) if args.eps is not None else 1.5 * voxel_size
    include_noise = not args.drop_noise

    log.info("session start  log=%s", log_path)
    log.info(
        "config  input=%s  output=%s  voxel_size=%.3fm  eps=%.3fm  min_samples=%d  "
        "chunk_extent=%d  compression=%s  swap_yz=%s  drop_noise=%s",
        args.input_pcd, args.output_vxw, voxel_size, eps, args.min_samples,
        args.chunk_extent, args.compression, args.swap_yz, args.drop_noise,
    )

    if not args.input_pcd.is_file():
        log.error("input file not found: %s", args.input_pcd)
        sys.exit(1)

    # [1/6] Load
    log.info("[1/6] loading %s (%d bytes)",
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

    # [2/6] Voxelize (dedupe)
    log.info("[2/6] voxelising  voxel_size=%.3fm", voxel_size)
    t0 = time.perf_counter()
    vc_unique = voxelize_unique(xyz, voxel_size)
    log.info(
        "      %d unique occupied voxels in %.2fs  (%.1fx reduction from %d points)",
        len(vc_unique), time.perf_counter() - t0,
        len(xyz) / max(1, len(vc_unique)), len(xyz),
    )

    # [3/6] DBSCAN
    log.info("[3/6] DBSCAN  eps=%.3fm  min_samples=%d  on %d voxel centres",
             eps, args.min_samples, len(vc_unique))
    t0 = time.perf_counter()
    labels = cluster_voxel_centres(vc_unique, voxel_size, eps, args.min_samples)
    dt = time.perf_counter() - t0

    n_noise = int((labels < 0).sum())
    n_clusters = int(labels.max()) + 1 if (labels >= 0).any() else 0
    log.info(
        "      %d clusters  +  %d noise voxels  in %.2fs",
        n_clusters, n_noise, dt,
    )

    # Top-5 cluster size report
    if n_clusters > 0:
        counts = Counter(int(l) for l in labels if l >= 0)
        top5 = counts.most_common(5)
        log.info("      top-5 cluster sizes (cluster_id : voxel_count): %s", top5)

    # [4/6] Map to material ids
    log.info("[4/6] mapping labels → material ids  (include_noise=%s)", include_noise)
    material_ids = labels_to_material_ids(labels, include_noise, n_clusters)

    # [5/6] Chunkize + write
    log.info("[5/6] chunkising and writing %s", args.output_vxw)
    t0 = time.perf_counter()
    chunks, bmin, bmax, total_occupied = chunkize(
        vc_unique, material_ids, args.chunk_extent,
        _COMPRESSION_MAP[args.compression],
    )
    log.info(
        "      %d chunks  %d occupied voxels  bounds_min=%s bounds_max=%s  in %.2fs",
        len(chunks), total_occupied, bmin, bmax, time.perf_counter() - t0,
    )

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
    palette = build_cluster_palette(n_clusters, include_noise)
    world = vxw.World(manifest=manifest, palette=palette, chunks=chunks)

    t0 = time.perf_counter()
    vxw.write_world(args.output_vxw, world)
    log.info("      wrote in %.2fs", time.perf_counter() - t0)

    # [6/6] Round-trip verify (with one retry to dodge NTFS race on Windows)
    log.info("[6/6] verifying round-trip")
    loaded_occ = -1
    loaded_chunks = -1
    last_exc: Exception | None = None
    for attempt in range(2):
        try:
            t0 = time.perf_counter()
            loaded = vxw.read_world(args.output_vxw)
            loaded_occ = sum(
                int((c.voxels["material_id"] != 0).sum())
                for c in loaded.chunks.values()
            )
            loaded_chunks = len(loaded.chunks)
            log.info(
                "      attempt %d: loaded back %d chunks  %d voxels  in %.2fs",
                attempt + 1, loaded_chunks, loaded_occ, time.perf_counter() - t0,
            )
            if loaded_occ == total_occupied and loaded_chunks == len(chunks):
                break
            log.warning(
                "      mismatch (expected %d chunks / %d voxels; got %d / %d) "
                "— retrying once",
                len(chunks), total_occupied, loaded_chunks, loaded_occ,
            )
            time.sleep(0.2)
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            log.warning("      attempt %d failed: %s — retrying", attempt + 1, exc)
            time.sleep(0.2)

    if loaded_occ != total_occupied:
        log.error("round-trip lost voxels! expected=%d got=%d  last_exc=%r",
                  total_occupied, loaded_occ, last_exc)
        sys.exit(2)
    if loaded_chunks != len(chunks):
        log.error("round-trip lost chunks! expected=%d got=%d  last_exc=%r",
                  len(chunks), loaded_chunks, last_exc)
        sys.exit(2)

    total_size = sum(
        p.stat().st_size for p in args.output_vxw.rglob("*") if p.is_file()
    )
    pcd_size = args.input_pcd.stat().st_size
    log.info(
        "RESULT  clusters=%d  noise_voxels=%d  occupied=%d  chunks=%d  "
        "pcd_bytes=%d  vxw_bytes=%d  ratio=%.4f  voxels_per_chunk=%.1f",
        n_clusters, n_noise, total_occupied, len(chunks),
        pcd_size, total_size, total_size / max(1, pcd_size),
        total_occupied / max(1, len(chunks)),
    )
    log.info("DONE    written=%s  log=%s", args.output_vxw, log_path)


if __name__ == "__main__":
    main()
