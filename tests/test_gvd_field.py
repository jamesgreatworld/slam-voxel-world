import numpy as np
import pytest
from m3_adapter.gvd.field import (
    flood_free_space, compute_esdf, extract_gvd, denoise_occupancy, thin_gvd,
    densify_occupancy, resolve_seed,
)


def _hollow_box(inner=19):
    n = inner + 2
    occ = np.zeros((n, n, n), dtype=bool)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    return occ, (n // 2, n // 2, n // 2)


def test_flood_fills_box_interior():
    occ, c = _hollow_box(19)
    free = flood_free_space(occ, c)
    assert free.sum() == 19 ** 3 and free[c] and not free[0, 0, 0]


def test_flood_seed_in_obstacle_raises():
    occ, _ = _hollow_box(19)
    with pytest.raises(ValueError):
        flood_free_space(occ, (0, 0, 0))


def test_esdf_box_center_distance():
    occ, c = _hollow_box(21)
    dist_m, parent = compute_esdf(occ, voxel_size=0.5)
    assert dist_m[c] == pytest.approx(11 * 0.5, abs=1e-6)
    assert parent.shape == (3,) + occ.shape


def test_gvd_bisector_between_two_obstacles():
    n = 21
    occ = np.zeros((n, n, n), dtype=bool)
    occ[2, 10, 10] = True
    occ[18, 10, 10] = True
    dist_m, parent = compute_esdf(occ, 1.0)
    gvd = extract_gvd(~occ, dist_m, parent, 1.0, d_min=1.0, theta_sep=2.0)
    assert gvd[10].any()
    assert not gvd[4, 10, 10]


def test_denoise_drops_small_components():
    occ = np.zeros((30, 30, 30), dtype=bool)
    occ[5:15, 5:15, 5:15] = True
    occ[25, 25, 25] = True
    cleaned = denoise_occupancy(occ, min_component_size=50)
    assert cleaned[10, 10, 10] and not cleaned[25, 25, 25] and cleaned.sum() == 1000


def test_thin_reduces_subset():
    g = np.zeros((20, 20, 20), dtype=bool)
    g[5:15, 5:15, 9:12] = True
    t = thin_gvd(g)
    assert t.sum() < g.sum() and t.sum() > 0 and np.all(g[t])


# --- densify_occupancy + resolve_seed tests ---

import vxw_format as vxw
from m3_adapter.common import build_concrete_palette


def _make_box_vxw(tmp_path, extent=32, vsize=0.5):
    """A hollow box room in chunk (0,0,0): walls at the 0 and 20 faces."""
    pal = build_concrete_palette()
    arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
    occ = np.zeros((21, 21, 21), dtype=bool)
    occ[0] = occ[20] = True
    occ[:, 0] = occ[:, 20] = True
    occ[:, :, 0] = occ[:, :, 20] = True
    ii = np.argwhere(occ)
    arr["material_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    arr["semantic_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    chunks = {(0, 0, 0): vxw.Chunk(coord=(0, 0, 0), voxels=arr)}
    man = vxw.Manifest(
        world_id="test", voxel_size_meters=vsize, chunk_extent=extent,
        bounds_chunks_min=(0, 0, 0), bounds_chunks_max=(1, 1, 1),
    )
    w = vxw.World(manifest=man, palette=pal, chunks=chunks)
    p = tmp_path / "box.vxw"
    vxw.write_world(p, w)
    return p


def test_densify_gives_expected_vmin_and_shape(tmp_path):
    p = _make_box_vxw(tmp_path)
    world = vxw.read_world(p)
    occ, vmin = densify_occupancy(world, pad=1)
    # box walls span world-voxels 0..20 -> with pad=1, vmin = (-1,-1,-1)
    assert tuple(int(x) for x in vmin) == (-1, -1, -1)
    assert occ.dtype == bool
    assert occ.any()


def test_resolve_seed_auto_returns_free_idx(tmp_path):
    p = _make_box_vxw(tmp_path)
    world = vxw.read_world(p)
    occ, vmin = densify_occupancy(world, pad=1)
    seed = resolve_seed(occ, vmin, voxel_size=0.5)
    # auto seed must be a free (non-obstacle) cell
    assert not occ[seed]
