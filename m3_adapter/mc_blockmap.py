"""Shared Minecraft block-ID → material_id mapping.

Used by both litematic_to_vxw.py and anvil_to_vxw.py. Keeps the mapping in
one place so adding a new MC block type is a single edit.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import vxw_format as vxw  # noqa: E402


# Minecraft block id (without minecraft: prefix, without [state]) →
# our material name in palette_minecraft.json. Anything not here falls
# back to "stone".
MC_TO_MATERIAL_NAME: dict[str, str] = {
    "stone": "stone",
    "smooth_stone": "stone",
    "andesite": "stone",
    "diorite": "stone",
    "granite": "stone",
    "polished_andesite": "stone",
    "polished_diorite": "stone",
    "polished_granite": "stone",
    "cobblestone": "cobblestone",
    "mossy_cobblestone": "cobblestone",
    "dirt": "dirt",
    "coarse_dirt": "dirt",
    "podzol": "dirt",
    "rooted_dirt": "dirt",
    "grass_block": "grass_block",
    "mycelium": "grass_block",
    "sand": "sand",
    "red_sand": "sand",
    "sandstone": "sand",
    "smooth_sandstone": "sand",
    "oak_planks": "oak_planks",
    "oak_log": "oak_log",
    "oak_wood": "oak_log",
    "stripped_oak_log": "oak_planks",
    "oak_leaves": "leaves",
    "spruce_planks": "spruce_planks",
    "spruce_log": "oak_log",
    "spruce_leaves": "leaves",
    "birch_planks": "birch_planks",
    "birch_log": "oak_log",
    "birch_leaves": "leaves",
    "dark_oak_planks": "oak_planks",
    "dark_oak_log": "oak_log",
    "dark_oak_leaves": "leaves",
    "jungle_planks": "oak_planks",
    "jungle_log": "oak_log",
    "jungle_leaves": "leaves",
    "acacia_planks": "oak_planks",
    "acacia_log": "oak_log",
    "acacia_leaves": "leaves",
    "glass": "glass",
    "white_stained_glass": "glass",
    "gray_stained_glass": "glass",
    "black_stained_glass": "glass",
    "glass_pane": "glass",
    "glowstone": "glowstone",
    "sea_lantern": "glowstone",
    "shroomlight": "glowstone",
    "redstone_lamp": "redstone_lamp",
    "iron_block": "iron_block",
    "iron_ore": "stone",  # ore is mostly stone with iron flecks
    "gold_block": "gold_block",
    "gold_ore": "stone",
    "diamond_block": "diamond_block",
    "diamond_ore": "stone",
    "emerald_block": "diamond_block",  # close enough teal
    "white_wool": "wool_white",
    "red_wool": "wool_red",
    "blue_wool": "wool_blue",
    "white_concrete": "concrete_white",
    "gray_concrete": "concrete_gray",
    "light_gray_concrete": "concrete_white",
    "black_concrete": "concrete_gray",
    "red_concrete": "concrete_red",
    "blue_concrete": "concrete_blue",
    "bricks": "brick",
    "brick_block": "brick",
    "nether_bricks": "brick",
    "stone_bricks": "stone_bricks",
    "polished_stone_bricks": "stone_bricks",
    "mossy_stone_bricks": "stone_bricks",
    "chiseled_stone_bricks": "stone_bricks",
    "cracked_stone_bricks": "stone_bricks",
    "obsidian": "obsidian",
    "water": "water",
    "lava": "lava",
    "ice": "ice",
    "packed_ice": "ice",
    "blue_ice": "ice",
    "snow_block": "snow_block",
    "snow": "snow_block",
    "bedrock": "obsidian",
    "deepslate": "stone",
    "tuff": "stone",
    "calcite": "snow_block",
    "moss_block": "grass_block",
}

# Block IDs that should be treated as empty space (no voxel emitted).
AIR_LIKE: frozenset[str] = frozenset({
    "air", "cave_air", "void_air", "structure_void",
})


def load_minecraft_palette(path: Path) -> tuple:
    """Load palette_minecraft.json → (vxw.Palette, name→id dict)."""
    data = json.loads(path.read_text(encoding="utf-8"))
    materials = [vxw.Material.from_dict(m) for m in data["materials"]]
    semantic_classes = [vxw.SemanticClass.from_dict(c) for c in data["semantic_classes"]]
    palette = vxw.Palette(
        materials=materials,
        semantic_classes=semantic_classes,
        color_lut=[tuple(c) for c in data.get("color_lut", [])],
    )
    name_to_id = {m.name: m.id for m in materials}
    return palette, name_to_id


def block_id_to_material(block_id: str, name_to_id: dict) -> int:
    """Map `minecraft:<name>[state]` → material_id. Air-like → 0, unknown → 1 (stone)."""
    name = block_id
    if name.startswith("minecraft:"):
        name = name[len("minecraft:"):]
    bracket = name.find("[")
    if bracket >= 0:
        name = name[:bracket]
    if name in AIR_LIKE:
        return 0
    mapped = MC_TO_MATERIAL_NAME.get(name, "stone")
    return name_to_id.get(mapped, 1)
