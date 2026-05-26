"""hydra_mesh_to_vxw — import a Hydra reconstruction into .vxw.

Reads the outputs of a Hydra (MIT-SPARK) SLAM run from its dataset directory:
  - mesh.ply      the reconstructed surface mesh (vertex positions + colors + normals)
  - dsg.json      the Dynamic Scene Graph (semantic objects, places, layers)

Voxelizes the mesh and (optionally) overlays semantic-object materials onto
the resulting voxel grid. Output is a single .vxw world directory.

Usage:
    pixi run python m3_adapter/hydra_mesh_to_vxw.py \\
        F:/hydra_ws/datasets/14floor/backend  out/14floor.vxw \\
        --voxel-size 0.10  --use-dsg-objects

Path A integration (no C++):
  - Hydra runs separately (Linux/ROS2/CUDA) and writes mesh.ply + dsg.json
  - This adapter consumes ONLY those JSON+PLY files. No Hydra library import.
  - Same adapter works for any scene Hydra ever produces (14floor, uhumans2, ...).
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

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import vxw_format as vxw  # noqa: E402

_PROJECT_ROOT = _HERE.parent
_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
log = logging.getLogger("hydra_to_vxw")


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"hydra_{ts}.log"
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


# ---------------------------------------------------------------------------
# Palette: a small Hydra-aware palette. Semantic labels from DSG objects get
# mapped to one of these material ids. Add categories as you encounter them.
# ---------------------------------------------------------------------------

def build_hydra_palette() -> vxw.Palette:
    return vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air",       color_rgb=(0, 0, 0),       flags=("empty",)),
            vxw.Material(id=1, name="concrete",  color_rgb=(180, 180, 180), flags=("solid", "destructible"), roughness=0.85),
            vxw.Material(id=2, name="floor",     color_rgb=(150, 130, 105), flags=("solid", "destructible"), roughness=0.9),
            vxw.Material(id=3, name="wood",      color_rgb=(162, 130, 78),  flags=("solid", "destructible", "flammable"), roughness=0.75),
            vxw.Material(id=4, name="chair",     color_rgb=(130, 90, 60),   flags=("solid", "destructible", "flammable"), roughness=0.8),
            vxw.Material(id=5, name="table",     color_rgb=(150, 105, 70),  flags=("solid", "destructible", "flammable"), roughness=0.75),
            vxw.Material(id=6, name="sofa",      color_rgb=(140, 90, 100),  flags=("solid", "destructible", "flammable"), roughness=0.95),
            vxw.Material(id=7, name="bed",       color_rgb=(180, 160, 130), flags=("solid", "destructible", "flammable"), roughness=0.95),
            vxw.Material(id=8, name="screen",    color_rgb=(40, 40, 50),    flags=("solid", "destructible"), emission_rgb=(80, 100, 150), emission_energy=0.6, roughness=0.3, metallic=0.1),
            vxw.Material(id=9, name="lamp",      color_rgb=(255, 230, 140), flags=("solid", "destructible", "emissive"), emission_rgb=(255, 220, 130), emission_energy=2.5, roughness=0.4),
            vxw.Material(id=10, name="plant",    color_rgb=(80, 140, 70),   flags=("solid", "destructible"), roughness=0.95),
            vxw.Material(id=11, name="window",   color_rgb=(200, 230, 255), flags=("solid", "destructible", "transparent"), transparent=True, roughness=0.1),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0, name="unknown",      default_material=1),
            vxw.SemanticClass(id=1, name="wall",         default_material=1),
            vxw.SemanticClass(id=2, name="floor",        default_material=2),
            vxw.SemanticClass(id=3, name="chair",        default_material=4),
            vxw.SemanticClass(id=4, name="table",        default_material=5),
            vxw.SemanticClass(id=5, name="sofa",         default_material=6),
            vxw.SemanticClass(id=6, name="bed",          default_material=7),
            vxw.SemanticClass(id=7, name="tv_monitor",   default_material=8),
            vxw.SemanticClass(id=8, name="lamp",         default_material=9),
            vxw.SemanticClass(id=9, name="plant",        default_material=10),
            vxw.SemanticClass(id=10, name="window",      default_material=11),
        ],
        color_lut=[],
    )


# Semantic category strings that Hydra/uhumans2 typically use, mapped to our
# material ids. Add to this when you see new labels in dsg.json.
DSG_CATEGORY_TO_MATERIAL: dict[str, int] = {
    "chair": 4, "armchair": 4, "stool": 4, "office_chair": 4,
    "table": 5, "desk": 5, "coffee_table": 5,
    "couch": 6, "sofa": 6,
    "bed": 7,
    "tv_monitor": 8, "monitor": 8, "tv": 8, "screen": 8, "computer": 8,
    "lamp": 9, "light": 9, "ceiling_light": 9,
    "potted_plant": 10, "plant": 10, "flower": 10,
    "window": 11, "glass": 11,
    "wall": 1,
    "floor": 2, "carpet": 2, "rug": 2,
    "ceiling": 1,
    "door": 3,
    "cabinet": 3, "shelves": 3, "bookshelf": 3, "cupboard": 3,
}


# ---------------------------------------------------------------------------
# Mesh → voxel
# ---------------------------------------------------------------------------

def voxelize_mesh(mesh_path: Path, voxel_size: float) -> tuple[np.ndarray, np.ndarray | None]:
    """Open3D-based mesh voxelization. Returns (voxel_centres_m_world, colors_or_None).

    Uses Open3D's `VoxelGrid.create_from_triangle_mesh` which marches the
    mesh surface and emits one voxel per surface cell. With vertex colors,
    each voxel gets the mean color of triangles in that cell.
    """
    import open3d as o3d   # lazy import

    log.info("reading mesh %s", mesh_path)
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    nverts = len(mesh.vertices)
    ntris = len(mesh.triangles)
    has_color = mesh.has_vertex_colors()
    log.info("mesh: %d vertices, %d triangles, vertex_colors=%s", nverts, ntris, has_color)
    if nverts == 0 or ntris == 0:
        raise RuntimeError("empty mesh — Hydra output broken?")

    bb = mesh.get_axis_aligned_bounding_box()
    extent = bb.get_extent()
    log.info("mesh bbox: %s..%s   extent %.2f x %.2f x %.2f m",
             bb.min_bound, bb.max_bound, extent[0], extent[1], extent[2])

    log.info("voxelizing at %.3fm ...", voxel_size)
    t0 = time.perf_counter()
    voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=voxel_size)
    voxels = voxel_grid.get_voxels()
    log.info("voxelized: %d occupied cells in %.2fs", len(voxels), time.perf_counter() - t0)

    if not voxels:
        raise RuntimeError("no voxels produced — voxel_size too large or mesh empty")

    # voxel_grid.origin = world-space corner of voxel (0,0,0); centre = origin + (idx+0.5)*voxel_size
    origin = np.asarray(voxel_grid.origin, dtype=np.float64)
    grid_indices = np.array([v.grid_index for v in voxels], dtype=np.int64)
    centres = origin[None, :] + (grid_indices.astype(np.float64) + 0.5) * voxel_size

    colors: np.ndarray | None = None
    if has_color:
        colors = np.array([v.color for v in voxels], dtype=np.float32)   # 0..1 floats
        log.info("voxel colors carried over from mesh vertex colors")
    return centres, colors, grid_indices


# ---------------------------------------------------------------------------
# DSG parsing (best-effort; format varies by Hydra version)
# ---------------------------------------------------------------------------

def load_dsg_objects(dsg_path: Path) -> list[dict]:
    """Extract semantic objects with bbox + category from a Hydra dsg.json.

    The schema isn't 100% stable across Hydra versions, so we hunt through
    layer dicts and yield anything that looks like an object (has a 'bbox'
    or 'world_R_bbox' field).
    """
    if not dsg_path.is_file():
        log.info("no dsg.json at %s — skipping semantic overlay", dsg_path)
        return []
    log.info("reading DSG %s", dsg_path)
    data = json.loads(dsg_path.read_text(encoding="utf-8"))

    objects: list[dict] = []
    # Hydra/spark_dsg schemas seen so far: top-level has 'layers' array;
    # each layer has 'nodes'; object nodes have 'attributes' with bounding boxes.
    layers = data.get("layers") or data.get("Layers") or []
    for layer in layers:
        nodes = layer.get("nodes") or layer.get("Nodes") or []
        for node in nodes:
            attrs = node.get("attributes") or node.get("Attributes") or {}
            name = attrs.get("name") or attrs.get("semantic_label") or ""
            category = attrs.get("category") or attrs.get("semantic_label") or attrs.get("name") or ""
            if not category:
                continue
            # Look for bbox in a few common forms
            bbox = None
            for k in ("bounding_box", "bbox", "world_R_bbox", "world_T_bbox"):
                if k in attrs:
                    bbox = attrs[k]
                    break
            position = attrs.get("position") or attrs.get("world_t_position")
            if bbox is None and position is None:
                continue
            objects.append({
                "name": name,
                "category": str(category).lower(),
                "bbox": bbox,
                "position": position,
                "id": node.get("id") or node.get("symbol"),
            })
    log.info("DSG: parsed %d candidate objects", len(objects))
    return objects


def category_to_material(category: str) -> int:
    """Map an arbitrary DSG category string → our material_id. Default 1 (concrete)."""
    cat = category.lower().strip()
    if cat in DSG_CATEGORY_TO_MATERIAL:
        return DSG_CATEGORY_TO_MATERIAL[cat]
    # heuristics on substrings
    for key, mid in DSG_CATEGORY_TO_MATERIAL.items():
        if key in cat:
            return mid
    return 1


# ---------------------------------------------------------------------------
# .vxw chunkization
# ---------------------------------------------------------------------------

def voxels_to_world(
    centres_m: np.ndarray,
    voxel_size: float,
    chunk_extent: int,
    compression: vxw.Compression,
    palette: vxw.Palette,
    default_material_id: int = 1,
    per_voxel_material: np.ndarray | None = None,
) -> tuple[dict, tuple, tuple, dict]:
    """centres_m: Nx3 world-space voxel centres. Returns chunks dict + bounds + histogram."""
    inv = 1.0 / voxel_size
    voxel_indices = np.floor(centres_m * inv + 0.5).astype(np.int64)  # round to int grid
    voxel_indices = np.unique(voxel_indices, axis=0)

    # Per-voxel material: array len N (matching voxel_indices order). If None → all default.
    if per_voxel_material is None:
        materials = np.full(voxel_indices.shape[0], default_material_id, dtype=np.uint8)
    else:
        materials = per_voxel_material

    cc = np.floor_divide(voxel_indices, chunk_extent).astype(np.int64)
    local = (voxel_indices - cc * chunk_extent).astype(np.uint8)

    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)

    chunks: dict = {}
    histogram: dict = defaultdict(int)
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        locs = local[mask]
        mids = materials[mask] if per_voxel_material is not None else np.full(int(mask.sum()), default_material_id, dtype=np.uint8)
        arr = np.zeros((chunk_extent, chunk_extent, chunk_extent), dtype=vxw.VOXEL_DTYPE)
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = mids
        arr["semantic_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = 1  # surface
        ckey = tuple(int(x) for x in chunk_keys[chunk_id])
        chunks[ckey] = vxw.Chunk(
            coord=ckey,
            voxels=arr,
            encoding=vxw.Encoding.RLE,
            compression=compression,
        )
        for m in mids.tolist():
            histogram[int(m)] += 1

    bmin = tuple(int(x) for x in cc.min(axis=0))
    bmax = tuple(int(x) + 1 for x in cc.max(axis=0))
    return chunks, bmin, bmax, histogram


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def hydra_to_vxw(
    hydra_dir: Path,
    output_vxw: Path,
    voxel_size: float,
    chunk_extent: int,
    compression: vxw.Compression,
    use_dsg_objects: bool,
) -> None:
    mesh_path = hydra_dir / "mesh.ply"
    dsg_path = hydra_dir / "dsg.json"
    if not mesh_path.is_file():
        log.error("expected %s — is this a Hydra dataset dir?", mesh_path)
        sys.exit(1)

    centres, mesh_colors, grid_indices = voxelize_mesh(mesh_path, voxel_size)

    per_voxel_material: np.ndarray | None = None
    if use_dsg_objects:
        objects = load_dsg_objects(dsg_path)
        if objects:
            per_voxel_material = np.full(centres.shape[0], 1, dtype=np.uint8)  # default concrete
            applied = 0
            for obj in objects:
                pos = obj.get("position")
                bbox = obj.get("bbox")
                cat = obj.get("category", "")
                mid = category_to_material(cat)
                if mid == 1:
                    continue  # unknown / wall, leave default
                # Apply bbox if available, else fall back to a small ball around position
                if isinstance(bbox, dict) and {"min", "max"}.issubset(bbox):
                    bmin = np.asarray(bbox["min"], dtype=np.float64)
                    bmax = np.asarray(bbox["max"], dtype=np.float64)
                    mask = np.all((centres >= bmin) & (centres <= bmax), axis=1)
                elif isinstance(bbox, list) and len(bbox) >= 6:
                    bmin = np.asarray(bbox[:3], dtype=np.float64)
                    bmax = np.asarray(bbox[3:6], dtype=np.float64)
                    mask = np.all((centres >= bmin) & (centres <= bmax), axis=1)
                elif pos is not None and isinstance(pos, list) and len(pos) == 3:
                    p = np.asarray(pos, dtype=np.float64)
                    radius = 0.6   # generic blob, 60cm
                    mask = np.linalg.norm(centres - p, axis=1) < radius
                else:
                    continue
                per_voxel_material[mask] = np.uint8(mid)
                applied += int(mask.sum())
            log.info("DSG overlay applied to %d voxels across %d objects", applied, len(objects))
        else:
            log.info("no DSG objects found — falling back to single-material output")

    palette = build_hydra_palette()
    chunks, bmin_chunk, bmax_chunk, histogram = voxels_to_world(
        centres, voxel_size, chunk_extent, compression, palette,
        default_material_id=1, per_voxel_material=per_voxel_material,
    )
    log.info("produced %d chunks, bounds %s..%s", len(chunks), bmin_chunk, bmax_chunk)
    id_to_name = {m.id: m.name for m in palette.materials}
    log.info("material histogram:")
    for mid, count in sorted(histogram.items(), key=lambda kv: -kv[1])[:12]:
        log.info("  %3d  %-12s  %8d voxels", mid, id_to_name.get(mid, "?"), count)

    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=voxel_size,
        chunk_extent=chunk_extent,
        bounds_chunks_min=bmin_chunk,
        bounds_chunks_max=bmax_chunk,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system="Hydra",
        source_sensor=str(hydra_dir.name),
    )
    world = vxw.World(manifest=manifest, palette=palette, chunks=chunks)
    vxw.write_world(output_vxw, world)
    log.info("wrote %s", output_vxw)


def main() -> None:
    log_path = _setup_logging()
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("hydra_dir", type=Path,
                    help="Hydra dataset dir containing mesh.ply (and ideally dsg.json)")
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument("--voxel-size", type=float, default=0.10)
    ap.add_argument("--chunk-extent", type=int, default=32)
    ap.add_argument("--compression", choices=["raw", "gzip", "zstd"], default="gzip")
    ap.add_argument("--use-dsg-objects", action="store_true",
                    help="parse dsg.json and overlay semantic-object materials")
    args = ap.parse_args()
    compression_map = {
        "raw": vxw.Compression.RAW,
        "gzip": vxw.Compression.GZIP,
        "zstd": vxw.Compression.ZSTD,
    }
    log.info("session start  log=%s", log_path)
    log.info("config  hydra_dir=%s  output=%s  voxel_size=%.3fm  use_dsg=%s",
             args.hydra_dir, args.output_vxw, args.voxel_size, args.use_dsg_objects)
    hydra_to_vxw(
        args.hydra_dir, args.output_vxw, args.voxel_size, args.chunk_extent,
        compression_map[args.compression], args.use_dsg_objects,
    )


if __name__ == "__main__":
    main()
