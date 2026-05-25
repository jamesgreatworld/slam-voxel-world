"""Generate samples/single_wall.vxw — the canonical P0 sample world.

Layout: a 5×5×1 concrete wall sitting on the floor of a single chunk.
Run: pixi run python samples/generate_single_wall.py
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402


def main() -> None:
    out = Path(__file__).resolve().parent / "single_wall.vxw"
    if out.exists():
        shutil.rmtree(out)

    manifest = vxw.Manifest(
        world_id="00000000-0000-0000-0000-00000000ca11",
        voxel_size_meters=0.05,
        chunk_extent=32,
        bounds_chunks_min=(0, 0, 0),
        bounds_chunks_max=(1, 1, 1),
        created_at="2026-05-23T00:00:00Z",
        source_slam_system="manual",
        source_sensor="none",
    )

    palette = vxw.Palette(
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

    voxels = np.zeros((32, 32, 32), dtype=vxw.VOXEL_DTYPE)
    voxels[0:5, 0:5, 0] = (1, 1, 0, 0)  # material=concrete, semantic=wall

    chunk = vxw.Chunk(
        coord=(0, 0, 0),
        voxels=voxels,
        encoding=vxw.Encoding.RLE,
        compression=vxw.Compression.ZSTD,
    )

    world = vxw.World(manifest=manifest, palette=palette, chunks={(0, 0, 0): chunk})
    vxw.write_world(out, world)

    # Sanity-read it back and print stats
    loaded = vxw.read_world(out)
    occupied = int((loaded.chunks[(0, 0, 0)].voxels["material_id"] != 0).sum())
    chunk_file = out / "chunks" / "0_0_0.chunk"
    print(f"wrote {out}")
    print(f"  chunks: {list(loaded.chunks.keys())}")
    print(f"  occupied voxels in chunk: {occupied}")
    print(f"  chunks/0_0_0.chunk size: {chunk_file.stat().st_size} bytes")


if __name__ == "__main__":
    main()
