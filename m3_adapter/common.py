"""m3_adapter/common — shared helpers used by every import adapter.

The adapters in this directory all do the same shape of work: read some
external format, optionally re-frame the coordinates, voxelise, build a
palette, and write a .vxw. They used to import each other for these
helpers (everyone reached into pcd_to_vxw), which made the per-adapter
intent muddier than it had to be. This module is the single home.

The functions here have no side effects beyond reading input files; they
do not log, do not touch CLI argparse, and accept already-decoded numpy
data. CLI / logging stays per-adapter.

Categories:
  - I/O               : load_pcd_xyz
  - coordinate frames : ros_zup_to_vxw_yup
  - voxelisation      : voxelize_and_group  (single-material, label=1)
  - palettes          : build_concrete_palette  (one solid material;
                        adapters that need richer palettes build their own)
  - compression       : COMPRESSION_MAP
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402


COMPRESSION_MAP = {
    "raw": vxw.Compression.RAW,
    "gzip": vxw.Compression.GZIP,
    "zstd": vxw.Compression.ZSTD,
}


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def load_pcd_xyz(path: Path) -> np.ndarray:
    """Read XYZ from a .pcd (any encoding pypcd4 supports). Returns (N, 3)
    float64 in the file's native frame (caller decides whether to swap)."""
    from pypcd4 import PointCloud

    pc = PointCloud.from_path(str(path))
    arr = pc.numpy()
    if arr.shape[1] < 3:
        raise ValueError(f"pcd has only {arr.shape[1]} fields, need >=3 (XYZ)")
    return np.ascontiguousarray(arr[:, :3], dtype=np.float64)


# ---------------------------------------------------------------------------
# Coordinate frames
# ---------------------------------------------------------------------------

def ros_zup_to_vxw_yup(xyz: np.ndarray) -> np.ndarray:
    """Convert ROS frame (X-forward, Y-left, Z-up) to vxw frame.

    vxw uses Godot/OpenGL convention: X-right, Y-up, Z-back. So ROS-forward
    maps to Godot-forward via ROS.X → -vxw.Z. Right-handed → right-handed,
    det = +1 (rotation, not reflection). See tests/test_adapter.py.
    """
    out = np.empty_like(xyz)
    out[:, 0] = -xyz[:, 1]  # vxw.X  = -ROS.Y
    out[:, 1] = xyz[:, 2]   # vxw.Y  =  ROS.Z
    out[:, 2] = -xyz[:, 0]  # vxw.Z  = -ROS.X
    return out


# ---------------------------------------------------------------------------
# Voxelisation
# ---------------------------------------------------------------------------

def voxelize_and_group(
    xyz: np.ndarray,
    voxel_size: float,
    chunk_extent: int,
    compression: vxw.Compression = vxw.Compression.GZIP,
) -> tuple[dict, tuple[int, int, int], tuple[int, int, int]]:
    """Voxelise an (N, 3) point cloud into single-material chunks.

    Every occupied voxel gets material_id=1 and semantic_id=1. Adapters that
    need per-voxel material/semantic should build their own chunk dict —
    this helper covers the "concrete blob" baseline that pcd / bag / dbscan
    fallbacks rely on.

    Returns (chunks_dict, bounds_chunks_min, bounds_chunks_max).
    """
    vc = np.floor(xyz / voxel_size).astype(np.int64)
    vc_unique = np.unique(vc, axis=0)
    cc = np.floor_divide(vc_unique, chunk_extent).astype(np.int64)
    local = (vc_unique - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)

    chunks: dict = {}
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        locs = local[mask]
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = 1
        arr["semantic_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = 1
        ckey = tuple(int(x) for x in chunk_keys[chunk_id])
        chunks[ckey] = vxw.Chunk(
            coord=ckey, voxels=arr,
            encoding=vxw.Encoding.RLE, compression=compression,
        )

    bounds_min = tuple(int(x) for x in cc.min(axis=0))
    bounds_max = tuple(int(x) + 1 for x in cc.max(axis=0))
    return chunks, bounds_min, bounds_max


# ---------------------------------------------------------------------------
# Palettes
# ---------------------------------------------------------------------------

def build_concrete_palette() -> vxw.Palette:
    """Baseline 2-material palette: air + concrete. The default for any
    adapter that doesn't carry semantic labels (pcd, bag-no-trajectory)."""
    return vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=("empty",)),
            vxw.Material(
                id=1,
                name="concrete",
                color_rgb=(180, 180, 180),
                flags=("solid", "destructible"),
                density=2400.0,
                hardness=30.0,
            ),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0, name="unknown", default_material=1),
            vxw.SemanticClass(id=1, name="wall", default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 178, 175)],
    )
