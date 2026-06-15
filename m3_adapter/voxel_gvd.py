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


def extract_gvd(
    free: np.ndarray,
    dist_m: np.ndarray,
    parent: np.ndarray,
    voxel_size: float,
    d_min: float = 0.20,
    theta_sep: float = 0.40,
) -> np.ndarray:
    """GVD voxels = free cells equidistant to >=2 distinct obstacles.

    A free candidate cell (dist >= d_min) is on the GVD if some 6-neighbour
    (also free) has a nearest-obstacle parent at least `theta_sep` metres away
    from this cell's parent — i.e. two distance wavefronts collide here. This
    is the Hydra GVD definition; parent SPACING replaces Hydra's parent-vector
    angle (simpler, more robust to voxelisation jaggies).

    Vectorised over the 3 axes via paired lo/hi slices (no np.roll wrap-around).

    Args:
        free: bool array (nx,ny,nz), True = flooded interior free space.
        dist_m: metres to nearest obstacle (compute_esdf).
        parent: (3,nx,ny,nz) nearest-obstacle index (compute_esdf).
        voxel_size: metres per voxel.
        d_min: drop near-surface noise ridges below this clearance (m).
        theta_sep: min parent spacing to count as "different obstacles" (m).

    Returns:
        bool array (nx,ny,nz), True = GVD skeleton voxel.
    """
    px = parent[0].astype(np.float64)
    py = parent[1].astype(np.float64)
    pz = parent[2].astype(np.float64)
    cand = free & (dist_m >= d_min)
    theta_vox2 = (theta_sep / voxel_size) ** 2
    gvd = np.zeros(free.shape, dtype=bool)
    for axis in range(3):
        lo = [slice(None)] * 3
        hi = [slice(None)] * 3
        lo[axis] = slice(0, -1)
        hi[axis] = slice(1, None)
        lo = tuple(lo)
        hi = tuple(hi)
        d2 = (px[lo] - px[hi]) ** 2 + (py[lo] - py[hi]) ** 2 + (pz[lo] - pz[hi]) ** 2
        both_free = free[lo] & free[hi]
        sep = both_free & (d2 >= theta_vox2)
        gvd[lo] |= cand[lo] & sep
        gvd[hi] |= cand[hi] & sep
    return gvd


def denoise_occupancy(
    occupied: np.ndarray, min_component_size: int, connectivity: int = 3
) -> np.ndarray:
    """Drop connected obstacle components smaller than `min_component_size`
    voxels — removes isolated scan noise / small fragments before GVD.

    Args:
        occupied: bool array (nx,ny,nz); True = obstacle.
        min_component_size: components with fewer voxels than this are removed.
        connectivity: ndimage structuring rank (3 = 26-connectivity).

    Returns:
        cleaned bool array (a no-op copy if min_component_size <= 1).
    """
    if min_component_size <= 1:
        return occupied
    structure = ndimage.generate_binary_structure(3, connectivity)
    labels, n = ndimage.label(occupied, structure=structure)
    if n == 0:
        return occupied
    sizes = np.bincount(labels.ravel())
    keep = sizes >= min_component_size
    keep[0] = False  # background label is never an obstacle
    return keep[labels]


def thin_gvd(gvd: np.ndarray) -> np.ndarray:
    """3D morphological skeletonisation of the GVD voxel set -> ~1-voxel-wide
    curves (skimage Lee's method). Returns a bool subset of `gvd`.
    """
    from skimage.morphology import skeletonize

    if not gvd.any():
        return gvd
    return np.asarray(skeletonize(gvd), dtype=bool)
