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


def compute_esdf(
    occupied: np.ndarray, voxel_size: float
) -> tuple[np.ndarray, np.ndarray]:
    """Euclidean distance field to the nearest obstacle, + the obstacle index.

    Computed over the WHOLE grid against the true obstacle set `occupied`
    (not against the flooded free set — see plan's algorithm note). The
    caller restricts to flooded free space in extract_gvd.

    Args:
        occupied: bool array (nx,ny,nz); True = obstacle.
        voxel_size: metres per voxel.

    Returns:
        (dist_m, parent):
            dist_m: float array (nx,ny,nz), metres to nearest obstacle.
            parent: int array (3, nx,ny,nz), index of that nearest obstacle
                    voxel ("basis point" in Hydra terms).
    """
    dist_vox, parent = ndimage.distance_transform_edt(
        ~occupied, return_indices=True
    )
    return dist_vox * voxel_size, parent
