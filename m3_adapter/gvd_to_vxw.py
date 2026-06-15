"""gvd_to_vxw.py — batch 3D GVD adapter (SP-A).

Reads a .vxw, densifies occupancy over its tight bbox, floods interior free
space from a seed, computes ESDF + GVD (m3_adapter.voxel_gvd), and writes a
new .vxw = original geometry + GVD skeleton voxels in a glowing material, so
the existing Godot viewer shows the skeleton with zero viewer changes.

ESDF/parent are transient (numpy only); the core .vxw format is untouched
(see docs/vision.md §4). CLI entry: see main().
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.voxel_gvd import (  # noqa: E402
    flood_free_space,
    compute_esdf,
    extract_gvd,
    denoise_occupancy,
    thin_gvd,
)

GVD_MATERIAL_NAME = "gvd_skeleton"
GVD_COLOR = (0, 255, 255)  # cyan
GVD_EMISSION_ENERGY = 3.0

PLACE_NODE_NAME = "place_node"
PLACE_NODE_COLOR = (255, 0, 255)  # magenta
PLACE_NODE_EMISSION = 4.0


def densify_occupancy(world, pad: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Dense bool occupancy grid over the world's tight bbox, with `pad`
    voxels of free margin on every side (so flood can surround geometry).

    Returns (occupied_bool (nx,ny,nz), vmin (3,) world-voxel coord of index 0).
    """
    extent = world.manifest.chunk_extent
    parts = []
    for ccoord, chunk in world.chunks.items():
        v = chunk.voxels
        mask = v["material_id"] != 0
        if not mask.any():
            continue
        idx = np.argwhere(mask)
        base = np.array(ccoord, dtype=np.int64) * extent
        parts.append(idx + base)
    if not parts:
        raise ValueError("world has no occupied voxels")
    allc = np.concatenate(parts, axis=0)
    vmin = allc.min(axis=0) - pad
    vmax = allc.max(axis=0) + pad
    dims = (vmax - vmin + 1).astype(int)
    occ = np.zeros(tuple(int(d) for d in dims), dtype=bool)
    rel = allc - vmin
    occ[rel[:, 0], rel[:, 1], rel[:, 2]] = True
    return occ, vmin


def _check_free(occupied: np.ndarray, idx: tuple, source: str) -> None:
    dims = occupied.shape
    if any(i < 0 or i >= d for i, d in zip(idx, dims)):
        raise ValueError(f"{source} seed {idx} is outside the world bbox {dims}")
    if occupied[idx]:
        raise ValueError(f"{source} seed {idx} is inside an obstacle")


def resolve_seed(
    occupied: np.ndarray,
    vmin: np.ndarray,
    voxel_size: float,
    seed_metres=None,
    spawn_hint=None,
) -> tuple[int, int, int]:
    """Pick the flood seed as a dense index. Priority: explicit --seed (metres)
    > manifest spawn_hint (metres) > auto (deepest interior point).
    """
    if seed_metres is not None:
        wv = np.round(np.asarray(seed_metres, dtype=float) / voxel_size).astype(int)
        idx = tuple(int(x) for x in (wv - vmin))
        _check_free(occupied, idx, "explicit --seed")
        return idx
    if spawn_hint is not None and len(spawn_hint) >= 3:
        wv = np.round(np.asarray(spawn_hint[:3], dtype=float) / voxel_size).astype(int)
        idx = tuple(int(x) for x in (wv - vmin))
        _check_free(occupied, idx, "spawn_hint")
        return idx
    # auto: the non-obstacle cell furthest from any obstacle = likely room centre
    dist = ndimage.distance_transform_edt(~occupied)
    idx = tuple(int(x) for x in np.unravel_index(int(np.argmax(dist)), dist.shape))
    return idx


def overlay_gvd_into_world(world, gvd: np.ndarray, vmin: np.ndarray) -> None:
    """Append a glowing GVD material to the palette and write each GVD voxel
    into the world's chunks (creating chunks for room interiors that had no
    geometry). Recomputes manifest bounds_chunks to cover any new chunks.
    Mutates `world` in place.
    """
    extent = world.manifest.chunk_extent
    pal = world.palette
    new_mat_id = max(m.id for m in pal.materials) + 1
    if new_mat_id > 255:
        raise ValueError("palette is full (256 materials); cannot add GVD material")
    new_color_idx = len(pal.color_lut)
    pal.materials.append(
        vxw.Material(
            id=new_mat_id,
            name=GVD_MATERIAL_NAME,
            color_rgb=GVD_COLOR,
            flags=("gvd", "emit"),
            emission_rgb=GVD_COLOR,
            emission_energy=GVD_EMISSION_ENERGY,
        )
    )
    pal.color_lut.append(GVD_COLOR)

    gvd_idx = np.argwhere(gvd)
    if len(gvd_idx) == 0:
        return
    world_v = gvd_idx + vmin
    cc = np.floor_divide(world_v, extent)
    local = (world_v - cc * extent).astype(np.uint8)
    for ck in np.unique(cc, axis=0):
        m = np.all(cc == ck, axis=1)
        ckey = tuple(int(x) for x in ck)
        if ckey in world.chunks:
            arr = world.chunks[ckey].voxels
        else:
            arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
            world.chunks[ckey] = vxw.Chunk(
                coord=ckey, voxels=arr,
                encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP,
            )
        loc = local[m]
        arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = new_mat_id
        arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 0
        arr["color_palette_idx"][loc[:, 0], loc[:, 1], loc[:, 2]] = new_color_idx

    keys = np.array(list(world.chunks.keys()))
    world.manifest.bounds_chunks_min = tuple(int(x) for x in keys.min(axis=0))
    world.manifest.bounds_chunks_max = tuple(int(x) + 1 for x in keys.max(axis=0))


def _write_graph_json(path: Path, nodes, edges, vmin, voxel_size) -> None:
    out_nodes = []
    for i, nd in enumerate(nodes):
        wv = np.asarray(nd["idx"], dtype=np.int64) + vmin
        pos = (wv.astype(float) * voxel_size)
        out_nodes.append({
            "id": i,
            "pos_m": [float(pos[0]), float(pos[1]), float(pos[2])],
            "clearance_m": float(nd["clearance_m"]),
            "degree": int(nd["degree"]),
            "type": nd["type"],
        })
    out_edges = [{"a": int(a), "b": int(b), "length_m": float(ln)}
                 for (a, b, ln) in edges]
    path.write_text(json.dumps({"nodes": out_nodes, "edges": out_edges}, indent=2))


def overlay_nodes_into_world(world, nodes, vmin, marker_radius: int = 1) -> None:
    """Draw each place node as a (2r+1)^3 magenta emissive cube."""
    if not nodes:
        return
    extent = world.manifest.chunk_extent
    pal = world.palette
    new_mat_id = max(m.id for m in pal.materials) + 1
    if new_mat_id > 255:
        raise ValueError("palette full; cannot add place_node material")
    new_color_idx = len(pal.color_lut)
    pal.materials.append(vxw.Material(
        id=new_mat_id, name=PLACE_NODE_NAME, color_rgb=PLACE_NODE_COLOR,
        flags=("place", "emit"), emission_rgb=PLACE_NODE_COLOR,
        emission_energy=PLACE_NODE_EMISSION,
    ))
    pal.color_lut.append(PLACE_NODE_COLOR)
    r = marker_radius
    cells = []
    for nd in nodes:
        cx, cy, cz = nd["idx"]
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    cells.append((cx + dx, cy + dy, cz + dz))
    mc = np.array(cells, dtype=np.int64) + vmin
    cc = np.floor_divide(mc, extent)
    local = (mc - cc * extent).astype(np.uint8)
    for ck in np.unique(cc, axis=0):
        m = np.all(cc == ck, axis=1)
        ckey = tuple(int(x) for x in ck)
        if ckey in world.chunks:
            arr = world.chunks[ckey].voxels
        else:
            arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
            world.chunks[ckey] = vxw.Chunk(
                coord=ckey, voxels=arr,
                encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP,
            )
        loc = local[m]
        arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = new_mat_id
        arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 0
        arr["color_palette_idx"][loc[:, 0], loc[:, 1], loc[:, 2]] = new_color_idx
    keys = np.array(list(world.chunks.keys()))
    world.manifest.bounds_chunks_min = tuple(int(x) for x in keys.min(axis=0))
    world.manifest.bounds_chunks_max = tuple(int(x) + 1 for x in keys.max(axis=0))


def run_gvd(
    input_vxw: str,
    output_vxw: str,
    seed_metres=None,
    d_min: float = 0.20,
    theta_sep: float = 0.40,
    pad: int = 1,
    band_max: float | None = 1.0,
    min_component: int = 0,
    thin: bool = False,
    graph: bool = False,
    merge_radius_m: float = 0.15,
) -> dict:
    """Full batch GVD pipeline. Returns a stats dict (also printed by main)."""
    t0 = time.perf_counter()
    world = vxw.read_world(Path(input_vxw))
    vsize = world.manifest.voxel_size_meters

    occ, vmin = densify_occupancy(world, pad=pad)
    occ_raw = int(occ.sum())
    if min_component and min_component > 1:
        occ = denoise_occupancy(occ, min_component)
    occ_clean = int(occ.sum())
    t_dense = time.perf_counter()

    seed = resolve_seed(
        occ, vmin, vsize, seed_metres=seed_metres,
        spawn_hint=world.manifest.spawn_hint,
    )
    free = flood_free_space(occ, seed)
    flood_fraction = float(free.sum()) / float(free.size)
    t_flood = time.perf_counter()

    dist_m, parent = compute_esdf(occ, vsize)

    # shell-band clip: restrict free to cells within band_max metres of a surface
    if band_max is not None and band_max > 0:
        free = free & (dist_m <= band_max)

    free_fraction = float(free.sum()) / float(free.size)

    gvd = extract_gvd(free, dist_m, parent, vsize, d_min=d_min, theta_sep=theta_sep)
    gvd_thick = int(gvd.sum())
    if thin:
        gvd = thin_gvd(gvd)
    t_gvd = time.perf_counter()

    nodes, edges = [], []
    if graph:
        from m3_adapter.gvd_graph import skeleton_to_graph
        nodes, edges = skeleton_to_graph(gvd, dist_m, vsize, merge_radius_m=merge_radius_m)

    overlay_gvd_into_world(world, gvd, vmin)
    if graph:
        overlay_nodes_into_world(world, nodes, vmin)
        _write_graph_json(
            Path(output_vxw).with_suffix(".graph.json"), nodes, edges, vmin, vsize
        )
    vxw.write_world(Path(output_vxw), world)
    t_write = time.perf_counter()

    stats = {
        "voxel_size_m": vsize,
        "bbox_dims": tuple(int(x) for x in occ.shape),
        "seed_idx": tuple(int(x) for x in seed),
        "free_cells": int(free.sum()),
        "flood_fraction": flood_fraction,
        "free_fraction": free_fraction,
        "band_max": band_max,
        "occ_voxels_raw": occ_raw,
        "occ_voxels_clean": occ_clean,
        "min_component": min_component,
        "gvd_thick": gvd_thick,
        "thin": thin,
        "gvd_voxels": int(gvd.sum()),
        "graph_nodes": len(nodes),
        "graph_edges": len(edges),
        "t_densify_s": round(t_dense - t0, 2),
        "t_flood_s": round(t_flood - t_dense, 2),
        "t_gvd_s": round(t_gvd - t_flood, 2),
        "t_write_s": round(t_write - t_gvd, 2),
        "t_total_s": round(t_write - t0, 2),
        "leak_warning": flood_fraction > 0.5,
    }
    print(f"[gvd] bbox {stats['bbox_dims']} voxel {vsize} m")
    print(f"[gvd] seed (dense idx) {stats['seed_idx']}")
    print(f"[gvd] flood free: {flood_fraction:.1%} of bbox (pre-band)")
    if stats["leak_warning"]:
        print(
            f"[gvd] note: scan is open (flood >50%); shell-band clip bounds it. "
            f"band_max={band_max} m -> {free_fraction * 100:.1f}% of bbox kept"
        )
    print(f"[gvd] GVD skeleton voxels: {stats['gvd_voxels']} (band_max={band_max} m)")
    if min_component and min_component > 1:
        print(f"[gvd] denoise: occ {occ_raw} -> {occ_clean} (dropped {occ_raw - occ_clean}, min_component={min_component})")
    if thin:
        print(f"[gvd] thin: GVD {gvd_thick} -> {stats['gvd_voxels']} voxels (skeletonized)")
    if graph:
        print(f"[gvd] places graph: {len(nodes)} nodes, {len(edges)} edges "
              f"-> {Path(output_vxw).with_suffix('.graph.json').name}")
    print(
        f"[gvd] timing s: densify={stats['t_densify_s']} flood={stats['t_flood_s']} "
        f"gvd={stats['t_gvd_s']} write={stats['t_write_s']} total={stats['t_total_s']}"
    )
    return stats


def _parse_seed(s: str):
    parts = s.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--seed needs 'x,y,z' in metres")
    return [float(p) for p in parts]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_vxw", type=str)
    ap.add_argument("output_vxw", type=str)
    ap.add_argument(
        "--seed", type=_parse_seed, default=None,
        help="interior seed point 'x,y,z' in metres (overrides spawn_hint/auto)",
    )
    ap.add_argument("--d-min", type=float, default=0.20,
                    help="min clearance (m) for a GVD voxel (drops surface noise)")
    ap.add_argument("--theta-sep", type=float, default=0.40,
                    help="min parent spacing (m) to count as different obstacles")
    ap.add_argument("--pad", type=int, default=1,
                    help="free-voxel margin around geometry for flooding")
    ap.add_argument("--band-max", type=float, default=1.0,
                    help="shell-band: keep only free cells within this many "
                         "metres of a surface (<=0 disables the band)")
    ap.add_argument("--min-component", type=int, default=0,
                    help="drop obstacle components smaller than N voxels (0=off)")
    ap.add_argument("--thin", action="store_true",
                    help="skeletonize the GVD voxel set to ~1-voxel curves")
    ap.add_argument("--graph", action="store_true",
                    help="sparsify the (thinned) skeleton into a places graph "
                         "(.graph.json + magenta node markers)")
    ap.add_argument("--merge-radius", type=float, default=0.15,
                    help="merge graph nodes within this many metres")
    args = ap.parse_args()
    bmax = args.band_max if args.band_max and args.band_max > 0 else None
    run_gvd(
        args.input_vxw, args.output_vxw, seed_metres=args.seed,
        d_min=args.d_min, theta_sep=args.theta_sep, pad=args.pad, band_max=bmax,
        min_component=args.min_component, thin=args.thin,
        graph=args.graph, merge_radius_m=args.merge_radius,
    )


if __name__ == "__main__":
    main()
