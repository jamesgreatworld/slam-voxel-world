"""pcd_to_vxw — convert a PCL .pcd point cloud into a .vxw world.

Source spec: §M3 of docs/superpowers/plans/2026-05-22-slam-voxel-world.md.
Input format: PCL binary-compressed .pcd with at least x,y,z fields.
              (Produced by SlamSystem::SaveMap in lightning_lm_foxy.)

Usage:
    pixi run python m3_adapter/pcd_to_vxw.py \\
        E:/aros_slam_ws/lightning_lm_foxy/data/test_compare_baseline/global.pcd \\
        out/baseline.vxw \\
        --voxel-size 0.10
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
log = logging.getLogger("pcd_to_vxw")


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"adapter_{ts}.log"
    fmt = logging.Formatter("[%(asctime)s] %(levelname)-5s %(name)s  %(message)s",
                            datefmt="%H:%M:%S")
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


def load_pcd_xyz(path: Path) -> np.ndarray:
    """Read XYZ from a .pcd (any encoding pypcd4 supports)."""
    from pypcd4 import PointCloud

    pc = PointCloud.from_path(str(path))
    arr = pc.numpy()
    if arr.shape[1] < 3:
        raise ValueError(f"pcd has only {arr.shape[1]} fields, need >=3 (XYZ)")
    return np.ascontiguousarray(arr[:, :3], dtype=np.float64)


def ros_zup_to_vxw_yup(xyz: np.ndarray) -> np.ndarray:
    """Convert ROS frame (X-forward, Y-left, Z-up) to vxw frame (Godot convention).

    Both right-handed. vxw uses Godot/OpenGL convention: X-right, Y-up, Z-back
    (camera looks down -Z, so "forward" is -Z). To make ROS-forward map to
    Godot-forward (the natural user expectation), ROS.X → -vxw.Z.

    Verified: this transform has determinant +1 (rotation, not reflection).
    See tests/test_adapter.py::test_coord_swap_preserves_handedness.
    """
    out = np.empty_like(xyz)
    out[:, 0] = -xyz[:, 1]  # vxw.X (right)   = -ROS.Y (left)
    out[:, 1] = xyz[:, 2]   # vxw.Y (up)      =  ROS.Z (up)
    out[:, 2] = -xyz[:, 0]  # vxw.Z (back)    = -ROS.X (forward)
    return out


_COMPRESSION_MAP = {
    "raw": vxw.Compression.RAW,
    "gzip": vxw.Compression.GZIP,
    "zstd": vxw.Compression.ZSTD,
}


def voxelize_and_group(
    xyz: np.ndarray,
    voxel_size: float,
    chunk_extent: int,
    compression: vxw.Compression = vxw.Compression.GZIP,
) -> tuple[dict, tuple[int, int, int], tuple[int, int, int]]:
    """Map points to (chunk_coord, local_coord, ...) and group by chunk.

    Returns (chunks_dict, bounds_min_chunk, bounds_max_chunk).
    """
    # Voxel coordinates (signed int)
    vc = np.floor(xyz / voxel_size).astype(np.int64)
    # Dedupe — many points may fall in the same voxel
    vc_unique = np.unique(vc, axis=0)

    cc = np.floor_divide(vc_unique, chunk_extent).astype(np.int64)
    local = (vc_unique - cc * chunk_extent).astype(np.uint8)

    # Group voxels by chunk coord using numpy
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)

    chunks: dict = {}
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        locs = local[mask]
        arr = np.zeros(
            (chunk_extent, chunk_extent, chunk_extent), dtype=vxw.VOXEL_DTYPE
        )
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = 1
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
    return chunks, bounds_min, bounds_max


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
            vxw.SemanticClass(id=1, name="wall", default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 178, 175)],
    )


def main() -> None:
    log_path = _setup_logging()

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_pcd", type=Path)
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument(
        "--voxel-size",
        type=float,
        default=0.10,
        help="meters per voxel (default 0.10, recommend 0.05 for room-scale)",
    )
    ap.add_argument("--chunk-extent", type=int, default=32)
    ap.add_argument(
        "--compression",
        choices=["raw", "gzip", "zstd"],
        default="gzip",
        help="chunk payload compression (default gzip; Godot can't decompress ZSTD)",
    )
    ap.add_argument(
        "--swap-yz",
        action="store_true",
        help="convert ROS Z-up frame → vxw Y-up frame",
    )
    ap.add_argument(
        "--source-slam",
        default="lightning_lm_foxy",
        help="SLAM system name written to manifest.source.slam_system",
    )
    args = ap.parse_args()

    log.info("session start  log=%s", log_path)
    log.info(
        "config  input=%s  output=%s  voxel_size=%.3fm  chunk_extent=%d  compression=%s  swap_yz=%s",
        args.input_pcd, args.output_vxw, args.voxel_size, args.chunk_extent,
        args.compression, args.swap_yz,
    )

    if not args.input_pcd.is_file():
        log.error("input file not found: %s", args.input_pcd)
        sys.exit(1)

    log.info("[1/4] loading %s (%d bytes)", args.input_pcd, args.input_pcd.stat().st_size)
    t0 = time.perf_counter()
    xyz = load_pcd_xyz(args.input_pcd)
    log.info("      %d points in %.2fs", len(xyz), time.perf_counter() - t0)
    _log_bbox("      raw bbox     ", xyz)

    if args.swap_yz:
        xyz = ros_zup_to_vxw_yup(xyz)
        _log_bbox("      Y-up bbox    ", xyz)

    log.info(
        "[2/4] voxelizing  voxel_size=%.3fm  chunk_extent=%d  compression=%s",
        args.voxel_size, args.chunk_extent, args.compression,
    )
    t0 = time.perf_counter()
    chunks, bmin, bmax = voxelize_and_group(
        xyz, args.voxel_size, args.chunk_extent, _COMPRESSION_MAP[args.compression]
    )
    log.info(
        "      %d chunks  bounds_min=%s bounds_max=%s  in %.2fs",
        len(chunks), bmin, bmax, time.perf_counter() - t0,
    )

    occ_per_chunk = Counter()
    for c in chunks.values():
        occ_per_chunk[int((c.voxels["material_id"] != 0).sum())] += 1
    total_voxels = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in chunks.values()
    )
    log.info("      total occupied voxels: %d", total_voxels)

    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=args.voxel_size,
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

    log.info("[3/4] writing %s", args.output_vxw)
    t0 = time.perf_counter()
    vxw.write_world(args.output_vxw, world)
    log.info("      wrote in %.2fs", time.perf_counter() - t0)

    log.info("[4/4] verifying round-trip")
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
        log.error("round-trip lost voxels! expected=%d got=%d", total_voxels, loaded_occ)
        sys.exit(2)
    if len(loaded.chunks) != len(chunks):
        log.error("round-trip lost chunks! expected=%d got=%d", len(chunks), len(loaded.chunks))
        sys.exit(2)

    total_size = sum(
        p.stat().st_size for p in args.output_vxw.rglob("*") if p.is_file()
    )
    pcd_size = args.input_pcd.stat().st_size
    log.info("RESULT  pcd_bytes=%d  vxw_bytes=%d  ratio=%.4f  voxels_per_chunk=%.1f",
             pcd_size, total_size, total_size / pcd_size,
             total_voxels / max(1, len(chunks)))
    log.info("DONE    written=%s  log=%s", args.output_vxw, log_path)


if __name__ == "__main__":
    main()
