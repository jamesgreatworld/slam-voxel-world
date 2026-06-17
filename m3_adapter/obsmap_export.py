"""obsmap_export.py — derive a renderable geometric .vxw from an ObsMap's
occupancy (one concrete material, or semantic-coloured when a semantic_grid is
given). Bridges the log-odds map to the .vxw format the Godot viewer / GVD
pipeline read. (SP-B v1 / v3.)"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.common import build_concrete_palette  # noqa: E402


def occupancy_to_vxw(
    occ_mask,
    vmin,
    voxel_size,
    out_path,
    chunk_extent=32,
    semantic_grid=None,
    palette=None,
):
    """Write a geometric .vxw from the occupied cells of *occ_mask*.

    Non-semantic mode (default):
        Every True cell gets material_id=1, semantic_id=1, concrete palette.

    Semantic mode (when *semantic_grid* and *palette* are provided):
        Every True cell gets material_id = semantic_id = the uint8 super_id
        from *semantic_grid* at the same position.  Cells occupied but with
        semantic label 0 (unknown) fall back to material_id=1 so they still
        render.  *palette* must be a vxw.Palette whose material ids cover the
        range of super_ids present (build_palette from uhumans2_to_vxw
        satisfies this).

    Args:
        occ_mask:       bool ndarray (nx, ny, nz) — True = occupied.
        vmin:           world-voxel coord of mask[0,0,0] (int64 (3,)).
        voxel_size:     metres per voxel side (float).
        out_path:       destination .vxw path.
        chunk_extent:   voxels per chunk side (<=255, default 32).
        semantic_grid:  optional uint8 ndarray same shape as occ_mask;
                        super_id per cell (0 = unknown).
        palette:        optional vxw.Palette to use instead of concrete.
    """
    coords = np.argwhere(occ_mask).astype(np.int64) + np.asarray(vmin, dtype=np.int64)
    if len(coords) == 0:
        raise ValueError("occupancy mask is empty")
    if chunk_extent > 255:
        raise ValueError(f"chunk_extent {chunk_extent} exceeds uint8 max (255)")

    # Gather per-occupied-cell semantic labels if requested.
    use_semantic = semantic_grid is not None and palette is not None
    if use_semantic:
        # occ_idx[i] = (ix, iy, iz) index into occ_mask / semantic_grid
        occ_idx = np.argwhere(occ_mask).astype(np.int64)  # (K, 3)
        cell_labels = semantic_grid[occ_idx[:, 0], occ_idx[:, 1], occ_idx[:, 2]]  # (K,) uint8
        # Unknown (label 0) occupied cells: fall back to material_id=1
        mat_ids = cell_labels.copy()
        mat_ids[mat_ids == 0] = 1

    cc = np.floor_divide(coords, chunk_extent)
    local = (coords - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)
    chunks = {}
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        loc = local[mask]
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
        if use_semantic:
            m_ids = mat_ids[mask]
            s_ids = cell_labels[mask]
            arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = m_ids
            arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = s_ids
        else:
            arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 1
            arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 1
        ckey = tuple(int(x) for x in chunk_keys[chunk_id])
        chunks[ckey] = vxw.Chunk(coord=ckey, voxels=arr,
                                 encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP)
    keys = np.array([tuple(int(x) for x in ck) for ck in chunk_keys])
    man = vxw.Manifest(
        world_id="obsmap_live", voxel_size_meters=float(voxel_size), chunk_extent=chunk_extent,
        bounds_chunks_min=tuple(int(x) for x in keys.min(axis=0)),
        bounds_chunks_max=tuple(int(x) + 1 for x in keys.max(axis=0)))
    chosen_palette = palette if use_semantic else build_concrete_palette()
    vxw.write_world(Path(out_path), vxw.World(manifest=man, palette=chosen_palette, chunks=chunks))
