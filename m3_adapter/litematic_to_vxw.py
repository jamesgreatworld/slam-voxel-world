"""litematic_to_vxw — import a Minecraft Litematica schematic into a .vxw world.

Uses litemapy (https://github.com/SmylerMC/litemapy) to parse `.litematic`.
Maps `minecraft:<block>` IDs to our material_ids via palette_minecraft.json.
Unknown blocks fall back to `stone` (id=1).

Usage:
    pixi run python m3_adapter/litematic_to_vxw.py \\
        input.litematic out/castle.vxw --voxel-size 1.0

A standard Minecraft block is 1 m^3, so --voxel-size 1.0 preserves scale.
Use 0.25 to "miniaturize" or 4.0 to upscale.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
import uuid
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.mc_blockmap import (  # noqa: E402
    block_id_to_material as _shared_block_id_to_material,
    load_minecraft_palette as _shared_load_minecraft_palette,
)


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
_DEFAULT_PALETTE = Path(__file__).resolve().parent / "palette_minecraft.json"
log = logging.getLogger("litematic_to_vxw")


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"litematic_{ts}.log"
    fmt = logging.Formatter(
        "[%(asctime)s] %(levelname)-5s %(name)s  %(message)s", datefmt="%H:%M:%S"
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


load_minecraft_palette = _shared_load_minecraft_palette
block_id_to_material = _shared_block_id_to_material


def litematic_to_vxw(
    litematic_path: Path,
    output_vxw: Path,
    voxel_size: float,
    chunk_extent: int,
    compression: vxw.Compression,
    palette_path: Path,
) -> None:
    """Convert a .litematic into a .vxw at the given path."""
    from litemapy import Schematic

    log.info("loading palette from %s", palette_path)
    palette, name_to_id = load_minecraft_palette(palette_path)
    log.info("palette has %d materials", len(palette.materials))

    log.info("loading %s", litematic_path)
    schem = Schematic.load(str(litematic_path))
    log.info("schematic regions: %s", list(schem.regions.keys()))

    voxels_dict: dict[tuple[int, int, int], int] = {}
    used_material_counts: dict[int, int] = defaultdict(int)

    for region_name, region in schem.regions.items():
        # Region origin in litematica is signed; size can be negative (we use abs).
        # Iterate via region.range_x/y/z if available; otherwise use width/height/length.
        ranges_x = list(region.range_x()) if hasattr(region, "range_x") else range(0, abs(region.width))
        ranges_y = list(region.range_y()) if hasattr(region, "range_y") else range(0, abs(region.height))
        ranges_z = list(region.range_z()) if hasattr(region, "range_z") else range(0, abs(region.length))
        log.info(
            "region %r: x[%d..%d] y[%d..%d] z[%d..%d]",
            region_name,
            min(ranges_x), max(ranges_x),
            min(ranges_y), max(ranges_y),
            min(ranges_z), max(ranges_z),
        )

        n_blocks = 0
        n_skipped_air = 0
        for x in ranges_x:
            for y in ranges_y:
                for z in ranges_z:
                    block_state = region[x, y, z]
                    block_id = block_state.id if hasattr(block_state, "id") else str(block_state)
                    material_id = block_id_to_material(block_id, name_to_id)
                    if material_id == 0:
                        n_skipped_air += 1
                        continue
                    voxels_dict[(x, y, z)] = material_id
                    used_material_counts[material_id] += 1
                    n_blocks += 1
        log.info(
            "region %r: %d non-air blocks (skipped %d air)",
            region_name, n_blocks, n_skipped_air,
        )

    if not voxels_dict:
        log.error("no non-air blocks found — output would be empty, aborting")
        sys.exit(1)

    # Group voxels into chunks
    bmin_chunk = (1_000_000, 1_000_000, 1_000_000)
    bmax_chunk = (-1_000_000, -1_000_000, -1_000_000)
    chunks_to_voxels: dict[tuple[int, int, int], list[tuple[int, int, int, int]]] = defaultdict(list)
    for (vx, vy, vz), mid in voxels_dict.items():
        cx, cy, cz = vx // chunk_extent, vy // chunk_extent, vz // chunk_extent
        lx, ly, lz = vx - cx * chunk_extent, vy - cy * chunk_extent, vz - cz * chunk_extent
        chunks_to_voxels[(cx, cy, cz)].append((lx, ly, lz, mid))
        bmin_chunk = (min(bmin_chunk[0], cx), min(bmin_chunk[1], cy), min(bmin_chunk[2], cz))
        bmax_chunk = (max(bmax_chunk[0], cx), max(bmax_chunk[1], cy), max(bmax_chunk[2], cz))
    bmax_chunk = (bmax_chunk[0] + 1, bmax_chunk[1] + 1, bmax_chunk[2] + 1)

    chunks: dict[tuple, vxw.Chunk] = {}
    for ckey, voxlist in chunks_to_voxels.items():
        arr = np.zeros((chunk_extent, chunk_extent, chunk_extent), dtype=vxw.VOXEL_DTYPE)
        for lx, ly, lz, mid in voxlist:
            arr["material_id"][lx, ly, lz] = mid
        chunks[ckey] = vxw.Chunk(
            coord=ckey,
            voxels=arr,
            encoding=vxw.Encoding.RLE,
            compression=compression,
        )

    log.info(
        "produced %d chunks, bounds=%s..%s", len(chunks), bmin_chunk, bmax_chunk
    )
    log.info("material histogram:")
    id_to_name = {m.id: m.name for m in palette.materials}
    for mid, count in sorted(used_material_counts.items(), key=lambda kv: -kv[1]):
        log.info("  %3d  %-16s  %6d voxels", mid, id_to_name.get(mid, "?"), count)

    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=voxel_size,
        chunk_extent=chunk_extent,
        bounds_chunks_min=bmin_chunk,
        bounds_chunks_max=bmax_chunk,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system="litematic",
        source_sensor=str(litematic_path.name),
    )
    world = vxw.World(manifest=manifest, palette=palette, chunks=chunks)
    vxw.write_world(output_vxw, world)
    log.info("wrote %s", output_vxw)


def main() -> None:
    log_path = _setup_logging()
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_litematic", type=Path)
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument("--voxel-size", type=float, default=1.0,
                    help="meters per voxel (default 1.0 = native MC block size)")
    ap.add_argument("--chunk-extent", type=int, default=32)
    ap.add_argument("--compression", choices=["raw", "gzip", "zstd"], default="gzip")
    ap.add_argument("--palette", type=Path, default=_DEFAULT_PALETTE,
                    help="palette_minecraft.json path")
    args = ap.parse_args()
    compression_map = {
        "raw": vxw.Compression.RAW,
        "gzip": vxw.Compression.GZIP,
        "zstd": vxw.Compression.ZSTD,
    }
    log.info("session start  log=%s", log_path)
    litematic_to_vxw(
        args.input_litematic,
        args.output_vxw,
        args.voxel_size,
        args.chunk_extent,
        compression_map[args.compression],
        args.palette,
    )


if __name__ == "__main__":
    main()
