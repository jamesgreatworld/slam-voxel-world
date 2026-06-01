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


from m3_adapter.common import (  # noqa: E402
    COMPRESSION_MAP as _COMPRESSION_MAP,
    build_concrete_palette as build_palette,
    load_pcd_xyz,
    ros_zup_to_vxw_yup,
    voxelize_and_group,
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
