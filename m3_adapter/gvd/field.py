"""field.py — voxel field stage: occupancy densify/seed, flood, ESDF, GVD, denoise, thin. Pure numpy/scipy. (GVD subsystem, see the architecture spec.)"""
from __future__ import annotations

import numpy as np
from scipy import ndimage


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


def extract_gvd_local(
    occ: np.ndarray,
    free: np.ndarray,
    voxel_size: float,
    bbox_min,
    bbox_max,
    margin_vox: int,
    d_min: float = 0.20,
    theta_sep: float = 0.40,
) -> np.ndarray:
    """Recompute the GVD only inside the core box [bbox_min, bbox_max) (dense
    indices into the full grid), using `margin_vox` extra cells on each side so
    boundary cells see nearby obstacles.  occ, free: full-grid bool arrays.

    Returns a full-grid bool array that is True only for GVD cells inside the
    CORE box (margin region is used for context but not returned).

    Parent-index relativity note: compute_esdf returns parent as indices into
    the sub-array.  extract_gvd only uses the DIFFERENCE between neighbouring
    cells' parent vectors (squared distance between two parent voxels in the
    same coordinate frame).  Since both parents live in the same sub-array
    frame, the difference — and hence the theta_sep test — is identical to
    what the global computation would produce.  No offset adjustment is needed.
    """
    shape = occ.shape
    lo = [max(0, int(bbox_min[a]) - margin_vox) for a in range(3)]
    hi = [min(shape[a], int(bbox_max[a]) + margin_vox) for a in range(3)]
    sl = tuple(slice(lo[a], hi[a]) for a in range(3))
    occ_sub = occ[sl]
    free_sub = free[sl]
    dist_sub, parent_sub = compute_esdf(occ_sub, voxel_size)
    gvd_sub = extract_gvd(free_sub, dist_sub, parent_sub, voxel_size,
                          d_min=d_min, theta_sep=theta_sep)
    out = np.zeros(shape, dtype=bool)
    core_lo = [int(bbox_min[a]) for a in range(3)]
    core_hi = [int(bbox_max[a]) for a in range(3)]
    csl_full = tuple(slice(core_lo[a], core_hi[a]) for a in range(3))
    csl_sub = tuple(slice(core_lo[a] - lo[a], core_hi[a] - lo[a]) for a in range(3))
    out[csl_full] = gvd_sub[csl_sub]
    return out


def replace_region(persistent_gvd, bbox_min, bbox_max, new_region):
    """Update a persistent full-grid GVD bool array in place: clear the core
    box [bbox_min,bbox_max) then OR in `new_region` (which is a full-grid bool
    that is True only inside that box, as returned by extract_gvd_local).
    Returns persistent_gvd (mutated)."""
    sl = tuple(slice(int(bbox_min[a]), int(bbox_max[a])) for a in range(3))
    persistent_gvd[sl] = new_region[sl]
    return persistent_gvd


def load_observed_free(path):
    """Load an observed-free mask sidecar. Returns (mask_bool, vmin_int, voxel_size)."""
    d = np.load(path)
    return d["mask"].astype(bool), d["vmin"].astype(np.int64), float(d["voxel_size"])


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
