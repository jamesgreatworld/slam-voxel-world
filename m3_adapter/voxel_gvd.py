"""voxel_gvd.py — pure-numpy 3D GVD primitives (SP-A).

No file I/O, no vxw_format dependency: arrays in, arrays out, so the same
functions serve the batch adapter (gvd_to_vxw.py) and a future incremental
mapper (SP-B). See docs/superpowers/specs/2026-06-13-spa-batch-gvd-design.md.

Pipeline: flood_free_space -> compute_esdf -> extract_gvd.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage


def flood_free_space(occupied: np.ndarray, seed: tuple[int, int, int]) -> np.ndarray:
    """6-connected free-space region containing `seed`.

    Args:
        occupied: bool array (nx,ny,nz); True = obstacle.
        seed: index into the array; must be a free (non-obstacle) cell.

    Returns:
        bool array, True where free space is 6-connected to `seed`.

    Raises:
        ValueError: if `seed` is inside an obstacle.
    """
    free = ~occupied
    if not free[tuple(seed)]:
        raise ValueError(f"seed {seed} is inside an obstacle")
    structure = ndimage.generate_binary_structure(3, 1)  # 6-connectivity
    labels, _ = ndimage.label(free, structure=structure)
    return labels == labels[tuple(seed)]
