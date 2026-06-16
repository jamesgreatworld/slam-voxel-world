"""render.py — stamp the GVD skeleton / graph nodes / rooms into a .vxw for the
Godot viewer, plus write the places-graph JSON. One stamp_voxels primitive
backs all overlays. (GVD subsystem.)"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import vxw_format as vxw

SKELETON_NAME = "gvd_skeleton"
SKELETON_COLOR = (0, 255, 255)        # cyan
SKELETON_EMISSION = 3.0
PLACE_NODE_NAME = "place_node"
PLACE_NODE_COLOR = (255, 0, 255)      # magenta
PLACE_NODE_EMISSION = 4.0
ROOM_COLORS = [
    (255, 80, 80), (80, 255, 80), (80, 80, 255), (255, 255, 80),
    (255, 80, 255), (80, 255, 255), (255, 160, 40), (160, 80, 255),
    (40, 255, 160), (255, 120, 160), (160, 255, 80), (120, 160, 255),
]
ROOM_EMISSION = 4.0


def stamp_voxels(world, cells_world, name, color_rgb, emission=3.0):
    """Append one glowing material and write `cells_world` (an (N,3) int array
    of WORLD voxel coords) into the world's chunks (creating chunks as needed).
    Recomputes manifest bounds_chunks. Returns the new material id. Idempotent
    on empty input (still appends the material so the palette is predictable)."""
    extent = world.manifest.chunk_extent
    pal = world.palette
    mat_id = max(m.id for m in pal.materials) + 1
    if mat_id > 255:
        raise ValueError(f"palette full; cannot add material {name!r}")
    col_idx = len(pal.color_lut)
    pal.materials.append(vxw.Material(
        id=mat_id, name=name, color_rgb=tuple(color_rgb),
        flags=("gvd", "emit"), emission_rgb=tuple(color_rgb), emission_energy=emission))
    pal.color_lut.append(tuple(color_rgb))
    cells = np.asarray(cells_world, dtype=np.int64)
    if cells.size == 0:
        return mat_id
    cc = np.floor_divide(cells, extent)
    local = (cells - cc * extent).astype(np.uint8)
    for ck in np.unique(cc, axis=0):
        m = np.all(cc == ck, axis=1)
        ckey = tuple(int(x) for x in ck)
        if ckey in world.chunks:
            arr = world.chunks[ckey].voxels
        else:
            arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
            world.chunks[ckey] = vxw.Chunk(coord=ckey, voxels=arr,
                encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP)
        loc = local[m]
        arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = mat_id
        arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 0
        arr["color_palette_idx"][loc[:, 0], loc[:, 1], loc[:, 2]] = col_idx
    keys = np.array(list(world.chunks.keys()))
    world.manifest.bounds_chunks_min = tuple(int(x) for x in keys.min(axis=0))
    world.manifest.bounds_chunks_max = tuple(int(x) + 1 for x in keys.max(axis=0))
    return mat_id


def _node_marker_cells(nodes, vmin, marker_radius):
    r = marker_radius
    cells = []
    for nd in nodes:
        cx, cy, cz = nd.idx
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    cells.append((cx + dx, cy + dy, cz + dz))
    return np.array(cells, dtype=np.int64) + np.asarray(vmin, dtype=np.int64)


def stamp_skeleton(world, gvd, vmin):
    cells = np.argwhere(gvd) + np.asarray(vmin, dtype=np.int64)
    return stamp_voxels(world, cells, SKELETON_NAME, SKELETON_COLOR, SKELETON_EMISSION)


def stamp_graph_nodes(world, graph, marker_radius: int = 1):
    if not graph.nodes:
        return
    cells = _node_marker_cells(graph.nodes, graph.vmin, marker_radius)
    stamp_voxels(world, cells, PLACE_NODE_NAME, PLACE_NODE_COLOR, PLACE_NODE_EMISSION)


def stamp_room_nodes(world, graph, marker_radius: int = 1):
    """One stamp call per room colour (room_id % 12)."""
    if not graph.nodes:
        return
    k = len(ROOM_COLORS)
    by_color = {}
    for nd in graph.nodes:
        by_color.setdefault(int(nd.room) % k, []).append(nd)
    for j in sorted(by_color):
        cells = _node_marker_cells(by_color[j], graph.vmin, marker_radius)
        stamp_voxels(world, cells, f"room_{j}", ROOM_COLORS[j], ROOM_EMISSION)


def write_graph_json(path, graph):
    """Write the places graph as JSON. Node pos_m = (idx+vmin)*voxel_size.
    Includes per-node 'room' and top-level 'num_rooms' when any node.room>=0."""
    has_rooms = any(n.room >= 0 for n in graph.nodes)
    out_nodes = []
    for i, nd in enumerate(graph.nodes):
        wv = np.asarray(nd.idx, dtype=np.int64) + np.asarray(graph.vmin, dtype=np.int64)
        pos = wv.astype(float) * graph.voxel_size
        d = {"id": i, "pos_m": [float(pos[0]), float(pos[1]), float(pos[2])],
             "clearance_m": float(nd.clearance_m), "degree": int(nd.degree),
             "type": nd.type}
        if has_rooms:
            d["room"] = int(nd.room)
        out_nodes.append(d)
    out_edges = [{"a": int(a), "b": int(b), "length_m": float(ln)}
                 for (a, b, ln) in graph.edges]
    doc = {"nodes": out_nodes, "edges": out_edges}
    if has_rooms:
        doc["num_rooms"] = len({n.room for n in graph.nodes})
    Path(path).write_text(json.dumps(doc, indent=2))
