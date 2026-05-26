"""anvil_to_vxw — import a Minecraft Java-edition world (Anvil format) into a .vxw.

Reads `region/r.*.mca` files via the `anvil-parser` Python library. Iterates
chunks within a configurable bounding box and Y range, maps Minecraft block
IDs to our material IDs via mc_blockmap.MC_TO_MATERIAL_NAME, and writes
a single .vxw.

Usage:
    pixi run python m3_adapter/anvil_to_vxw.py \\
        path/to/world out/world.vxw \\
        --y-range 60,90 \\
        --chunk-bbox -8,-8,8,8

Default Y range is 60..90 (typical surface band) to keep output manageable;
override with --y-range MIN,MAX. Default chunk bbox is the full extent
of all region files found.

Caveats:
  - anvil-parser supports Minecraft Java up to ~1.17. Newer worlds (1.18+
    with extended Y range and palette changes) may need a different parser
    such as `nucleation` or `amulet-core`. The error path tries to skip
    chunks that fail to decode rather than abort.
  - This adapter is a structural / interactive demo, not a perfectly faithful
    Minecraft renderer. Block states (orientation, water level, etc.) are
    discarded; only block type matters for voxel material.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
import uuid
from collections import defaultdict
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import vxw_format as vxw  # noqa: E402
from m3_adapter.mc_blockmap import (  # noqa: E402
    block_id_to_material,
    load_minecraft_palette,
)

_PROJECT_ROOT = _HERE.parent
_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
_DEFAULT_PALETTE = _HERE / "palette_minecraft.json"
log = logging.getLogger("anvil_to_vxw")


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"anvil_{ts}.log"
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


def _parse_region_filename(path: Path) -> tuple[int, int] | None:
    """`r.X.Z.mca` → (X, Z) integers; returns None if filename doesn't match."""
    parts = path.stem.split(".")
    if len(parts) != 3 or parts[0] != "r":
        return None
    try:
        return int(parts[1]), int(parts[2])
    except ValueError:
        return None


def _iter_blocks_in_chunk(chunk, y_min: int, y_max: int):
    """Yield (xi, yi, zi, block_id_str) for each non-air block in the chunk.

    Defensive: catches per-block exceptions so a single bad block doesn't kill
    the whole region. Returns nothing if the chunk decoder fails entirely.
    """
    try:
        for y in range(y_min, y_max + 1):
            for x in range(16):
                for z in range(16):
                    try:
                        block = chunk.get_block(x, y, z)
                    except Exception:
                        continue
                    if block is None:
                        continue
                    bid = getattr(block, "id", None)
                    if bid is None:
                        bid = str(block)
                    if not bid.startswith("minecraft:"):
                        bid = "minecraft:" + bid
                    yield x, y, z, bid
    except Exception as exc:
        log.warning("chunk decode failed mid-iteration: %s", exc)


def anvil_to_vxw(
    world_dir: Path,
    output_vxw: Path,
    voxel_size: float,
    chunk_extent: int,
    compression: vxw.Compression,
    palette_path: Path,
    y_min: int,
    y_max: int,
    chunk_bbox: tuple[int, int, int, int] | None,
) -> None:
    import anvil  # heavy; import lazily so --help doesn't need it

    log.info("loading palette from %s", palette_path)
    palette, name_to_id = load_minecraft_palette(palette_path)

    region_dir = world_dir / "region"
    if not region_dir.is_dir():
        log.error("expected %s/region/ subdir (Minecraft world directory)", world_dir)
        sys.exit(1)

    mca_files = sorted(region_dir.glob("r.*.mca"))
    log.info("found %d .mca region files in %s", len(mca_files), region_dir)
    if not mca_files:
        log.error("no .mca files found")
        sys.exit(1)

    voxels_dict: dict[tuple[int, int, int], int] = {}
    used_material_counts: dict[int, int] = defaultdict(int)
    chunks_seen = 0
    chunks_skipped = 0

    for mca in mca_files:
        rxz = _parse_region_filename(mca)
        if rxz is None:
            log.warning("skipping unrecognized filename: %s", mca.name)
            continue
        rx, rz = rxz
        try:
            region = anvil.Region.from_file(str(mca))
        except Exception as exc:
            log.warning("cannot open region %s: %s", mca.name, exc)
            continue

        for cx in range(32):
            for cz in range(32):
                wcx = rx * 32 + cx
                wcz = rz * 32 + cz
                if chunk_bbox is not None:
                    cx_min, cz_min, cx_max, cz_max = chunk_bbox
                    if wcx < cx_min or wcx > cx_max or wcz < cz_min or wcz > cz_max:
                        continue
                try:
                    chunk = region.get_chunk(cx, cz)
                except Exception:
                    chunks_skipped += 1
                    continue
                if chunk is None:
                    chunks_skipped += 1
                    continue
                chunks_seen += 1
                for xi, yi, zi, bid in _iter_blocks_in_chunk(chunk, y_min, y_max):
                    mat_id = block_id_to_material(bid, name_to_id)
                    if mat_id == 0:
                        continue
                    wx = wcx * 16 + xi
                    wz = wcz * 16 + zi
                    voxels_dict[(wx, yi, wz)] = mat_id
                    used_material_counts[mat_id] += 1
        log.info(
            "region (%d, %d): cumulative %d voxels (%d chunks seen, %d skipped)",
            rx, rz, len(voxels_dict), chunks_seen, chunks_skipped,
        )

    if not voxels_dict:
        log.error("no non-air blocks decoded — possibly a Minecraft version "
                  "anvil-parser does not support (try nucleation or amulet-core)")
        sys.exit(2)

    # ---- Voxelize into .vxw chunks ----
    bmin_chunk = (10**9, 10**9, 10**9)
    bmax_chunk = (-10**9, -10**9, -10**9)
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

    log.info("produced %d .vxw chunks, bounds=%s..%s", len(chunks), bmin_chunk, bmax_chunk)
    id_to_name = {m.id: m.name for m in palette.materials}
    log.info("material histogram:")
    for mid, count in sorted(used_material_counts.items(), key=lambda kv: -kv[1])[:20]:
        log.info("  %3d  %-16s  %8d voxels", mid, id_to_name.get(mid, "?"), count)

    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=voxel_size,
        chunk_extent=chunk_extent,
        bounds_chunks_min=bmin_chunk,
        bounds_chunks_max=bmax_chunk,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system="anvil",
        source_sensor=str(world_dir.name),
    )
    world = vxw.World(manifest=manifest, palette=palette, chunks=chunks)
    vxw.write_world(output_vxw, world)
    log.info("wrote %s", output_vxw)


def main() -> None:
    log_path = _setup_logging()
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("world_dir", type=Path, help="Minecraft world dir (contains region/)")
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument("--voxel-size", type=float, default=1.0)
    ap.add_argument("--chunk-extent", type=int, default=32)
    ap.add_argument("--compression", choices=["raw", "gzip", "zstd"], default="gzip")
    ap.add_argument("--palette", type=Path, default=_DEFAULT_PALETTE)
    ap.add_argument("--y-range", default="60,90",
                    help="MIN,MAX block Y to import (default 60,90 for typical surface)")
    ap.add_argument("--chunk-bbox", default=None,
                    help="cx_min,cz_min,cx_max,cz_max in MC chunk coords; default = all regions")
    args = ap.parse_args()
    y_min, y_max = (int(x) for x in args.y_range.split(","))
    cbbox = None
    if args.chunk_bbox:
        nums = [int(x) for x in args.chunk_bbox.split(",")]
        if len(nums) != 4:
            log.error("--chunk-bbox must be 4 comma-separated ints, got %d", len(nums))
            sys.exit(1)
        cbbox = tuple(nums)
    compression_map = {
        "raw": vxw.Compression.RAW,
        "gzip": vxw.Compression.GZIP,
        "zstd": vxw.Compression.ZSTD,
    }
    log.info("session start  log=%s", log_path)
    log.info(
        "config  world=%s  out=%s  voxel_size=%.2fm  y=[%d,%d]  bbox=%s",
        args.world_dir, args.output_vxw, args.voxel_size, y_min, y_max, cbbox,
    )
    anvil_to_vxw(
        args.world_dir, args.output_vxw, args.voxel_size, args.chunk_extent,
        compression_map[args.compression], args.palette, y_min, y_max, cbbox,
    )


if __name__ == "__main__":
    main()
