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
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

import vxw_format as vxw
from m3_adapter.voxel_gvd import flood_free_space, compute_esdf, extract_gvd

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
