"""Generate a small Litematica test file with multiple material types.

Run: pixi run python m3_adapter/make_test_litematic.py
Output: out/test_castle.litematic — a 16×8×16 castle with stone walls,
oak floors, glass windows, glowstone lamps, gold roof.
"""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))


def main() -> None:
    from litemapy import Schematic, Region, BlockState

    out_path = _PROJECT_ROOT / "out" / "test_castle.litematic"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    width, height, length = 16, 8, 16
    region = Region(0, 0, 0, width, height, length)

    stone = BlockState("minecraft:stone")
    oak_planks = BlockState("minecraft:oak_planks")
    glass = BlockState("minecraft:glass")
    glowstone = BlockState("minecraft:glowstone")
    gold = BlockState("minecraft:gold_block")
    air = BlockState("minecraft:air")

    # Fill with air first
    for x in range(width):
        for y in range(height):
            for z in range(length):
                region[x, y, z] = air

    # Stone floor at y=0
    for x in range(width):
        for z in range(length):
            region[x, 0, z] = stone

    # Oak plank floor at y=1
    for x in range(1, width - 1):
        for z in range(1, length - 1):
            region[x, 1, z] = oak_planks

    # Stone walls y=1..5 around perimeter
    for y in range(1, 6):
        for x in range(width):
            region[x, y, 0] = stone
            region[x, y, length - 1] = stone
        for z in range(length):
            region[0, y, z] = stone
            region[width - 1, y, z] = stone

    # Glass windows on the front wall (z=0) at y=2..3
    for x in range(2, width - 2, 3):
        region[x, 2, 0] = glass
        region[x, 3, 0] = glass

    # Glass windows on the right wall (x=width-1)
    for z in range(2, length - 2, 3):
        region[width - 1, 2, z] = glass
        region[width - 1, 3, z] = glass

    # Glowstone lamps inside ceiling y=5
    for x in range(3, width - 3, 4):
        for z in range(3, length - 3, 4):
            region[x, 5, z] = glowstone

    # Gold roof at y=6 (a small pyramid for visual interest)
    for x in range(2, width - 2):
        for z in range(2, length - 2):
            region[x, 6, z] = gold
    for x in range(4, width - 4):
        for z in range(4, length - 4):
            region[x, 7, z] = gold

    schem = region.as_schematic(
        name="TestCastle",
        author="slam-voxel-world",
        description="Synthetic multi-material test castle for litematic adapter.",
    )
    schem.save(str(out_path))
    print(f"wrote {out_path}")
    # Count blocks
    counts: dict[str, int] = {}
    for x in range(width):
        for y in range(height):
            for z in range(length):
                bid = region[x, y, z].id
                counts[bid] = counts.get(bid, 0) + 1
    print("block counts:")
    for k, v in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"  {v:6d}  {k}")


if __name__ == "__main__":
    main()
