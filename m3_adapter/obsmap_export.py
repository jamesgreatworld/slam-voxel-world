"""obsmap_export.py — derive a renderable geometric .vxw from an ObsMap's
occupancy (one concrete material). Bridges the log-odds map to the .vxw format
the Godot viewer / GVD pipeline read. (SP-B v1.)"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.common import build_concrete_palette  # noqa: E402


def occupancy_to_vxw(occ_mask, vmin, voxel_size, out_path, chunk_extent=32):
    """Write a geometric .vxw: every True cell in occ_mask becomes a concrete
    voxel (material_id=1, semantic_id=1). vmin = world-voxel coord of mask[0,0,0]."""
    coords = np.argwhere(occ_mask).astype(np.int64) + np.asarray(vmin, dtype=np.int64)
    if len(coords) == 0:
        raise ValueError("occupancy mask is empty")
    if chunk_extent > 255:
        raise ValueError(f"chunk_extent {chunk_extent} exceeds uint8 max (255)")
    cc = np.floor_divide(coords, chunk_extent)
    local = (coords - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)
    chunks = {}
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        loc = local[mask]
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
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
    vxw.write_world(Path(out_path), vxw.World(manifest=man, palette=build_concrete_palette(), chunks=chunks))
