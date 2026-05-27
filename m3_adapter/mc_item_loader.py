"""mc_item_loader — parse a Minecraft-style block model pack into ItemPresets.

Reads MC-standard block model JSON (parent/textures/elements with from/to/
faces) from `mc_item_pack/`. Outputs `mc_item_pack/_compiled.json` that
godot_viewer/item_picker reads directly: for each item, a list of sub-boxes
(min/max in metres, normalised so the model's overall bbox is centred at the
origin), one solid colour per face derived from the MC texture name.

Why this layout: any standard MC resource pack drops into the same models/
textures/ structure, so future packs only need a manifest.json appended.

MC convention:
  - Element coords are in "MC units" where 16 units = 1 block (≈1.0 m by
    default; we keep that mapping).
  - `from` and `to` are box corners in [0..N] (N can exceed 16 for >1 block).
  - `textures: {key: "block/foo"}` and faces reference `#key`.
  - Faces with `texture` field name must resolve to a known texture name in
    _TEXTURE_COLORS. Unknown textures get a debug magenta fallback.

Run:
  pixi run python m3_adapter/mc_item_loader.py
    --pack m3_adapter/mc_item_pack
    --out  m3_adapter/mc_item_pack/_compiled.json
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

log = logging.getLogger("mc_item_loader")

# MC-block coord scale: 16 units = 1 metre (we keep the natural MC mapping).
_MC_UNIT_TO_METRE = 1.0 / 16.0

# Curated MC-vanilla texture name → average colour. Hand-picked; anything not
# in this table falls back to a deterministic hash-based colour so the model
# still renders (in debug magenta-ish range).
_TEXTURE_COLORS: dict[str, tuple[int, int, int]] = {
    "block/oak_planks":              (159, 132, 77),
    "block/oak_log_top":             (165, 130, 65),
    "block/dark_oak_planks":         (80, 53, 27),
    "block/spruce_planks":           (115, 85, 49),
    "block/birch_planks":            (215, 195, 140),
    "block/red_wool":                (165, 36, 27),
    "block/orange_wool":             (240, 119, 28),
    "block/yellow_wool":             (240, 180, 47),
    "block/green_wool":              (94, 124, 22),
    "block/lime_wool":               (110, 188, 33),
    "block/light_blue_wool":         (96, 168, 213),
    "block/blue_wool":               (53, 57, 157),
    "block/white_wool":              (236, 236, 236),
    "block/gray_wool":               (62, 68, 73),
    "block/light_gray_wool":         (143, 143, 134),
    "block/black_wool":              (21, 21, 26),
    "block/iron_block":              (220, 220, 220),
    "block/gold_block":              (251, 222, 79),
    "block/black_concrete":          (15, 16, 19),
    "block/white_concrete":          (207, 213, 214),
    "block/gray_concrete":           (62, 68, 73),
    "block/red_concrete":            (143, 33, 33),
    "block/light_blue_stained_glass":(101, 159, 217),
    "block/glass":                   (210, 230, 240),
    "block/stone":                   (125, 125, 125),
    "block/cobblestone":             (122, 122, 122),
    "block/bedrock":                 (84, 84, 84),
}


def _fallback_color(name: str) -> tuple[int, int, int]:
    import hashlib
    h = hashlib.md5(name.encode()).digest()
    # bias towards mid-brightness so the debug colour is visible
    return (60 + h[0] % 160, 60 + h[1] % 160, 60 + h[2] % 160)


def _texture_png_relpath(texture_name: str, pack_root: Path) -> str | None:
    """`block/oak_planks` → 'textures/block/oak_planks.png' if the file exists
    under pack_root; else None. Path is relative to pack_root so consumers
    can join with whatever absolute prefix they need."""
    rel = Path("textures") / Path(texture_name).with_suffix(".png")
    if (pack_root / rel).is_file():
        return str(rel.as_posix())
    return None


@dataclass
class ItemBox:
    min_m: tuple                       # (x, y, z) metres, model-origin-centred
    max_m: tuple
    face_colors: dict = field(default_factory=dict)    # face_name -> (r,g,b)
    face_textures: dict = field(default_factory=dict)  # face_name -> rel PNG path
    dominant_texture: str | None = None                # rel PNG path; phase-1 albedo


@dataclass
class ItemPreset:
    id: str
    category: str
    default_label: int                 # uHumans2 super_id this item maps to
    boxes: list                        # list[ItemBox]
    overall_extents_m: tuple           # (dx, dy, dz) metres
    source_model: str                  # relative path to source .json


def _resolve_texture(face: dict, textures: dict) -> str:
    """`#key` → walks textures dict (which may also contain `#otherkey`)."""
    name = face.get("texture", "")
    seen: set[str] = set()
    while isinstance(name, str) and name.startswith("#"):
        key = name[1:]
        if key in seen:
            return "block/missing"
        seen.add(key)
        name = textures.get(key, "block/missing")
    return name if isinstance(name, str) else "block/missing"


def _color_for(texture_name: str) -> tuple[int, int, int]:
    if texture_name in _TEXTURE_COLORS:
        return _TEXTURE_COLORS[texture_name]
    return _fallback_color(texture_name)


def _model_bbox(elements: list) -> tuple[tuple, tuple]:
    """Return (min, max) in MC units across all elements (before centring)."""
    mns = [min(e["from"][i], e["to"][i]) for e in elements for i in range(3)]
    mxs = [max(e["from"][i], e["to"][i]) for e in elements for i in range(3)]
    return (
        (min(mns[0::3]), min(mns[1::3]), min(mns[2::3])),
        (max(mxs[0::3]), max(mxs[1::3]), max(mxs[2::3])),
    )


def load_model(model_path: Path, pack_root: Path) -> tuple[list, tuple, dict]:
    """Parse one MC block model JSON. Returns (boxes_in_metres, extents_m,
    textures_map). Sub-box coordinates are centred at the model's overall
    centre (so the entity's `position` puts that centre in the world).

    Each box also gets face_textures (face_name → PNG rel path) and a
    dominant_texture (the most-frequently-referenced face texture; consumers
    that only support one albedo per box pick this)."""
    data = json.loads(model_path.read_text())
    textures = data.get("textures", {})
    elements = data.get("elements", [])
    if not elements:
        return [], (0.0, 0.0, 0.0), textures

    (mnx, mny, mnz), (mxx, mxy, mzz) = _model_bbox(elements)
    extents_units = (mxx - mnx, mxy - mny, mzz - mnz)
    cx = (mnx + mxx) / 2
    cy = (mny + mxy) / 2
    cz = (mnz + mzz) / 2

    boxes: list[ItemBox] = []
    for el in elements:
        f, t = el["from"], el["to"]
        box_min = (
            (min(f[0], t[0]) - cx) * _MC_UNIT_TO_METRE,
            (min(f[1], t[1]) - cy) * _MC_UNIT_TO_METRE,
            (min(f[2], t[2]) - cz) * _MC_UNIT_TO_METRE,
        )
        box_max = (
            (max(f[0], t[0]) - cx) * _MC_UNIT_TO_METRE,
            (max(f[1], t[1]) - cy) * _MC_UNIT_TO_METRE,
            (max(f[2], t[2]) - cz) * _MC_UNIT_TO_METRE,
        )
        face_colors: dict[str, tuple] = {}
        face_textures: dict[str, str] = {}
        tex_counts: dict[str, int] = {}
        for face_name, face in (el.get("faces") or {}).items():
            tname = _resolve_texture(face, textures)
            face_colors[face_name] = _color_for(tname)
            png = _texture_png_relpath(tname, pack_root)
            if png is not None:
                face_textures[face_name] = png
                tex_counts[png] = tex_counts.get(png, 0) + 1
        dominant = max(tex_counts, key=tex_counts.get) if tex_counts else None
        boxes.append(ItemBox(
            min_m=box_min, max_m=box_max,
            face_colors=face_colors,
            face_textures=face_textures,
            dominant_texture=dominant,
        ))

    extents_m = tuple(e * _MC_UNIT_TO_METRE for e in extents_units)
    return boxes, extents_m, textures


def load_pack(pack_root: Path) -> dict[str, ItemPreset]:
    manifest = json.loads((pack_root / "manifest.json").read_text())
    presets: dict[str, ItemPreset] = {}
    for entry in manifest.get("items", []):
        model_path = pack_root / entry["model"]
        if not model_path.is_file():
            log.warning("model file missing for item %r: %s",
                        entry["id"], model_path)
            continue
        boxes, extents_m, _ = load_model(model_path, pack_root)
        presets[entry["id"]] = ItemPreset(
            id=entry["id"],
            category=str(entry.get("category", "misc")),
            default_label=int(entry.get("default_label", 0)),
            boxes=boxes,
            overall_extents_m=extents_m,
            source_model=str(model_path.relative_to(pack_root).as_posix()),
        )
    return presets


def _preset_to_dict(p: ItemPreset) -> dict:
    return {
        "id": p.id,
        "category": p.category,
        "default_label": p.default_label,
        "overall_extents_m": list(p.overall_extents_m),
        "source_model": p.source_model,
        "boxes": [
            {
                "min": list(b.min_m),
                "max": list(b.max_m),
                "face_colors": {k: list(v) for k, v in b.face_colors.items()},
                "face_textures": dict(b.face_textures),
                "dominant_texture": b.dominant_texture,
            }
            for b in p.boxes
        ],
    }


def compile_pack(pack_root: Path, out_path: Path) -> int:
    presets = load_pack(pack_root)
    payload = {
        "pack_format": "1.0",
        "pack_root": str(pack_root.as_posix()),
        "presets": [_preset_to_dict(p) for p in presets.values()],
    }
    out_path.write_text(json.dumps(payload, indent=2))
    return len(presets)


def main() -> None:
    logging.basicConfig(level=logging.INFO,
                        format="[%(levelname)s] %(name)s  %(message)s")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pack", type=Path,
                    default=Path(__file__).parent / "mc_item_pack")
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "mc_item_pack" / "_compiled.json")
    args = ap.parse_args()
    if not (args.pack / "manifest.json").is_file():
        log.error("no manifest.json under %s", args.pack)
        sys.exit(1)
    n = compile_pack(args.pack, args.out)
    log.info("compiled %d presets → %s", n, args.out)


if __name__ == "__main__":
    main()
