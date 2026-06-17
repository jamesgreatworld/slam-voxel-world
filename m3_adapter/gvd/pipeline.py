"""pipeline.py — staged GVD pipeline: GvdConfig + run(cfg). Composes
field / graph / rooms / render. (GVD subsystem.)"""
from __future__ import annotations

from dataclasses import dataclass
import time
from pathlib import Path

import numpy as np

import vxw_format as vxw
from m3_adapter.gvd import field, render
from m3_adapter.gvd.graph import skeleton_to_graph, prune_spurs, merge_close
from m3_adapter.gvd.rooms import partition_rooms


@dataclass
class GvdConfig:
    input_vxw: str
    output_vxw: str
    seed_metres: tuple | None = None
    pad: int = 1
    band_max: float | None = 1.0
    min_component: int = 0
    d_min: float = 0.20
    theta_sep: float = 0.40
    thin: bool = False
    graph: bool = False
    merge_radius_m: float = 0.15
    prune_spurs_m: float = 0.0
    merge_close_m: float = 0.0
    rooms: bool = False
    room_resolution: float = 1.0
    observed_free_path: str | None = None


def run(cfg: GvdConfig) -> dict:
    t0 = time.perf_counter()
    world = vxw.read_world(Path(cfg.input_vxw))
    vsize = world.manifest.voxel_size_meters

    occ, vmin = field.densify_occupancy(world, pad=cfg.pad)
    occ_raw = int(occ.sum())
    if cfg.min_component and cfg.min_component > 1:
        occ = field.denoise_occupancy(occ, cfg.min_component)
    occ_clean = int(occ.sum())

    dist_m, parent = field.compute_esdf(occ, vsize)

    if cfg.observed_free_path:
        mask, mvmin, mvs = field.load_observed_free(cfg.observed_free_path)
        if mask.shape != occ.shape or not np.array_equal(np.asarray(mvmin), np.asarray(vmin)):
            raise ValueError(
                f"observed_free grid mismatch: mask shape {mask.shape} vmin {tuple(int(x) for x in mvmin)} "
                f"vs occ shape {occ.shape} vmin {tuple(int(x) for x in vmin)} — "
                f"regenerate the mask against this .vxw with pad={cfg.pad}")
        free = mask & ~occ
        flood_fraction = float(free.sum()) / float(free.size)  # here = observed-free fraction
        seed_idx = (-1, -1, -1)  # not used in this path
    else:
        seed = field.resolve_seed(occ, vmin, vsize, seed_metres=cfg.seed_metres,
                                  spawn_hint=world.manifest.spawn_hint)
        seed_idx = tuple(int(x) for x in seed)
        free = field.flood_free_space(occ, seed)
        flood_fraction = float(free.sum()) / float(free.size)

    if cfg.band_max is not None and cfg.band_max > 0:
        free = free & (dist_m <= cfg.band_max)
    free_fraction = float(free.sum()) / float(free.size)

    gvd = field.extract_gvd(free, dist_m, parent, vsize, d_min=cfg.d_min, theta_sep=cfg.theta_sep)
    gvd_thick = int(gvd.sum())
    if cfg.thin:
        gvd = field.thin_gvd(gvd)
    t_field = time.perf_counter()

    graph_obj = None
    n_raw = 0
    do_graph = cfg.graph or cfg.rooms
    if do_graph:
        graph_obj = skeleton_to_graph(gvd, dist_m, vsize, vmin, merge_radius_m=cfg.merge_radius_m)
        n_raw = len(graph_obj.nodes)
        if cfg.prune_spurs_m and cfg.prune_spurs_m > 0:
            graph_obj = prune_spurs(graph_obj, cfg.prune_spurs_m)
        if cfg.merge_close_m and cfg.merge_close_m > 0:
            graph_obj = merge_close(graph_obj, cfg.merge_close_m)
    num_rooms = 0
    if cfg.rooms and graph_obj is not None:
        labels = partition_rooms(graph_obj, resolution=cfg.room_resolution)
        for nd, r in zip(graph_obj.nodes, labels):
            nd.room = int(r)
        num_rooms = len(set(labels)) if labels else 0
    t_graph = time.perf_counter()

    render.stamp_skeleton(world, gvd, vmin)
    if do_graph and graph_obj is not None:
        if cfg.rooms:
            render.stamp_room_nodes(world, graph_obj)
        else:
            render.stamp_graph_nodes(world, graph_obj)
        render.write_graph_json(Path(cfg.output_vxw).with_suffix(".graph.json"), graph_obj)
    vxw.write_world(Path(cfg.output_vxw), world)
    t_write = time.perf_counter()

    stats = {
        "voxel_size_m": vsize,
        "bbox_dims": tuple(int(x) for x in occ.shape),
        "seed_idx": seed_idx,
        "occ_raw": occ_raw, "occ_clean": occ_clean,
        "flood_fraction": flood_fraction, "free_fraction": free_fraction,
        "band_max": cfg.band_max,
        "gvd_thick": gvd_thick, "gvd_voxels": int(gvd.sum()),
        "graph_nodes_raw": n_raw,
        "graph_nodes": (len(graph_obj.nodes) if graph_obj is not None else 0),
        "graph_edges": (len(graph_obj.edges) if graph_obj is not None else 0),
        "num_rooms": num_rooms,
        "observed_free": bool(cfg.observed_free_path),
        "leak_warning": flood_fraction > 0.5,
        "t_field_s": round(t_field - t0, 2),
        "t_graph_s": round(t_graph - t_field, 2),
        "t_write_s": round(t_write - t_graph, 2),
        "t_total_s": round(t_write - t0, 2),
    }
    print(f"[gvd] bbox {stats['bbox_dims']} voxel {vsize} m  seed {seed_idx}")
    if cfg.min_component and cfg.min_component > 1:
        print(f"[gvd] denoise: occ {occ_raw} -> {occ_clean}")
    print(f"[gvd] flood {flood_fraction:.1%} (pre-band) -> band {cfg.band_max} m kept {free_fraction:.1%}")
    print(f"[gvd] GVD voxels: {stats['gvd_voxels']}" + (f" (thinned from {gvd_thick})" if cfg.thin else ""))
    if do_graph:
        msg = f"[gvd] graph: {stats['graph_nodes']} nodes / {stats['graph_edges']} edges"
        if cfg.prune_spurs_m or cfg.merge_close_m:
            msg += f" (cleaned from {n_raw})"
        print(msg)
    if cfg.rooms:
        print(f"[gvd] rooms: {num_rooms} communities")
    print(f"[gvd] timing s: field={stats['t_field_s']} graph={stats['t_graph_s']} write={stats['t_write_s']} total={stats['t_total_s']}")
    return stats
