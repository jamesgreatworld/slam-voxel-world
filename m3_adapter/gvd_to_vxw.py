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
)

GVD_MATERIAL_NAME = "gvd_skeleton"
GVD_COLOR = (0, 255, 255)  # cyan
GVD_EMISSION_ENERGY = 3.0


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


def run_gvd(
    input_vxw: str,
    output_vxw: str,
    seed_metres=None,
    d_min: float = 0.20,
    theta_sep: float = 0.40,
    pad: int = 1,
) -> dict:
    """Full batch GVD pipeline. Returns a stats dict (also printed by main)."""
    t0 = time.perf_counter()
    world = vxw.read_world(Path(input_vxw))
    vsize = world.manifest.voxel_size_meters

    occ, vmin = densify_occupancy(world, pad=pad)
    t_dense = time.perf_counter()

    seed = resolve_seed(
        occ, vmin, vsize, seed_metres=seed_metres,
        spawn_hint=world.manifest.spawn_hint,
    )
    free = flood_free_space(occ, seed)
    free_fraction = float(free.sum()) / float(free.size)
    t_flood = time.perf_counter()

    dist_m, parent = compute_esdf(occ, vsize)
    gvd = extract_gvd(free, dist_m, parent, vsize, d_min=d_min, theta_sep=theta_sep)
    t_gvd = time.perf_counter()

    overlay_gvd_into_world(world, gvd, vmin)
    vxw.write_world(Path(output_vxw), world)
    t_write = time.perf_counter()

    stats = {
        "voxel_size_m": vsize,
        "bbox_dims": tuple(int(x) for x in occ.shape),
        "seed_idx": tuple(int(x) for x in seed),
        "free_cells": int(free.sum()),
        "free_fraction": free_fraction,
        "gvd_voxels": int(gvd.sum()),
        "t_densify_s": round(t_dense - t0, 2),
        "t_flood_s": round(t_flood - t_dense, 2),
        "t_gvd_s": round(t_gvd - t_flood, 2),
        "t_write_s": round(t_write - t_gvd, 2),
        "t_total_s": round(t_write - t0, 2),
        "leak_warning": free_fraction > 0.5,
    }
    print(f"[gvd] bbox {stats['bbox_dims']} voxel {vsize} m")
    print(f"[gvd] seed (dense idx) {stats['seed_idx']}")
    print(
        f"[gvd] flood free: {stats['free_cells']} cells "
        f"= {free_fraction:.1%} of bbox"
    )
    if stats["leak_warning"]:
        print(
            "[gvd] WARN: flood fills >50% of bbox — likely leaking through "
            "wall/ceiling holes; inspect the result, consider --seed or the "
            "2.5D fallback (design §4)."
        )
    print(f"[gvd] GVD skeleton voxels: {stats['gvd_voxels']}")
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
    args = ap.parse_args()
    run_gvd(
        args.input_vxw, args.output_vxw, seed_metres=args.seed,
        d_min=args.d_min, theta_sep=args.theta_sep, pad=args.pad,
    )


if __name__ == "__main__":
    main()
