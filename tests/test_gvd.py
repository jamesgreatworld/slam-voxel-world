import numpy as np
import pytest
from m3_adapter.voxel_gvd import flood_free_space


def _hollow_box(inner=19):
    """Solid shell cube. Returns (occupied_bool, interior_center_idx).
    Grid is (inner+2)^3; walls on every outer face; interior 1..inner free."""
    n = inner + 2
    occ = np.zeros((n, n, n), dtype=bool)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    center = (n // 2, n // 2, n // 2)
    return occ, center


def test_flood_fills_box_interior():
    occ, center = _hollow_box(inner=19)
    free = flood_free_space(occ, center)
    # interior is 19^3 free cells, all reachable from centre
    assert free.sum() == 19 ** 3
    assert free[center]
    # walls are never free
    assert not free[0, 0, 0]


def test_flood_seed_in_obstacle_raises():
    occ, _ = _hollow_box(inner=19)
    with pytest.raises(ValueError):
        flood_free_space(occ, (0, 0, 0))


def test_flood_does_not_leak_through_sealed_wall():
    # two 5^3 rooms side by side sharing a solid wall -> seed in room A
    # must NOT reach room B.
    occ = np.zeros((13, 7, 7), dtype=bool)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    occ[6, :, :] = True          # solid dividing wall at x=6
    free = flood_free_space(occ, (3, 3, 3))   # room A (x=1..5)
    assert free[3, 3, 3]
    assert not free[9, 3, 3]     # room B (x=7..11) unreachable
    # a 1-voxel hole in the divider DOES leak (demonstrates the risk)
    occ[6, 3, 3] = False
    free2 = flood_free_space(occ, (3, 3, 3))
    assert free2[9, 3, 3]


from m3_adapter.voxel_gvd import compute_esdf


def test_esdf_box_center_distance():
    occ, center = _hollow_box(inner=21)   # interior 1..21, center at 11
    dist_m, parent = compute_esdf(occ, voxel_size=0.5)
    # centre is 11 voxels from the nearest wall (index 0 or 22) -> 11 * 0.5 m
    assert dist_m[center] == pytest.approx(11 * 0.5, abs=1e-6)
    # parent is the (3, nx,ny,nz) index of the nearest obstacle voxel
    assert parent.shape == (3,) + occ.shape
    pcoord = parent[:, center[0], center[1], center[2]]
    # nearest obstacle must actually BE an obstacle
    assert occ[pcoord[0], pcoord[1], pcoord[2]]


from m3_adapter.voxel_gvd import extract_gvd


def test_gvd_is_bisector_between_two_obstacles():
    # two point obstacles along x; GVD = perpendicular bisector plane x=10
    n = 21
    occ = np.zeros((n, n, n), dtype=bool)
    occ[2, 10, 10] = True
    occ[18, 10, 10] = True
    free = ~occ
    dist_m, parent = compute_esdf(occ, voxel_size=1.0)
    gvd = extract_gvd(free, dist_m, parent, voxel_size=1.0, d_min=1.0, theta_sep=2.0)
    # midplane x=10 (equidistant to both) contains GVD voxels
    assert gvd[10].any()
    # a cell clearly nearer obstacle A is not on the GVD
    assert not gvd[4, 10, 10]


def test_gvd_respects_free_mask_and_dmin():
    occ, center = _hollow_box(inner=21)
    free = flood_free_space(occ, center)
    dist_m, parent = compute_esdf(occ, voxel_size=0.5)
    gvd = extract_gvd(free, dist_m, parent, voxel_size=0.5, d_min=0.20, theta_sep=0.40)
    # every GVD voxel is free and at least d_min from any obstacle
    assert gvd[~free].sum() == 0
    assert (dist_m[gvd] >= 0.20 - 1e-9).all()
    # the medial axis of a box room is non-empty
    assert gvd.any()


import vxw_format as vxw
from m3_adapter.common import build_concrete_palette
from m3_adapter.gvd_to_vxw import densify_occupancy, resolve_seed, overlay_gvd_into_world


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


def test_densify_and_overlay_roundtrip(tmp_path):
    p = _make_box_vxw(tmp_path)
    world = vxw.read_world(p)
    occ, vmin = densify_occupancy(world, pad=1)
    # box walls span world-voxels 0..20 -> with pad=1, vmin = (-1,-1,-1)
    assert tuple(int(x) for x in vmin) == (-1, -1, -1)
    assert occ.dtype == bool and occ.any()

    seed = resolve_seed(occ, vmin, voxel_size=0.5)  # auto: deepest interior point
    assert not occ[seed]

    free = flood_free_space(occ, seed)
    dist_m, parent = compute_esdf(occ, 0.5)
    gvd = extract_gvd(free, dist_m, parent, 0.5, d_min=0.20, theta_sep=0.40)
    assert gvd.any()

    n_mat_before = len(world.palette.materials)
    overlay_gvd_into_world(world, gvd, vmin)
    # a new glowing material was appended
    assert len(world.palette.materials) == n_mat_before + 1
    gvd_mat = world.palette.materials[-1]
    assert gvd_mat.emission_energy > 0
    # the overlaid world round-trips and the gvd material id appears in chunks
    out = tmp_path / "box_gvd.vxw"
    vxw.write_world(out, world)
    w2 = vxw.read_world(out)
    found = any(
        (ch.voxels["material_id"] == gvd_mat.id).any() for ch in w2.chunks.values()
    )
    assert found


def test_cli_end_to_end_on_box(tmp_path):
    from m3_adapter.gvd_to_vxw import run_gvd
    src = _make_box_vxw(tmp_path)
    out = tmp_path / "box_gvd.vxw"
    stats = run_gvd(
        str(src), str(out), seed_metres=None,
        d_min=0.20, theta_sep=0.40, pad=1,
    )
    # stats dict reports the run
    assert stats["gvd_voxels"] > 0
    assert 0.0 < stats["free_fraction"] <= 1.0
    # output world loads and carries the glowing skeleton material
    w = vxw.read_world(out)
    names = [m.name for m in w.palette.materials]
    assert "gvd_skeleton" in names


def test_band_restricts_free_to_shell():
    # the band invariant: with a band, every GVD voxel is within band_max of a
    # surface; without a band, the GVD reaches deeper (into the room centre).
    occ, center = _hollow_box(inner=21)
    free = flood_free_space(occ, center)
    dist_m, parent = compute_esdf(occ, voxel_size=0.5)
    band = 1.0
    free_band = free & (dist_m <= band)
    gvd_band = extract_gvd(free_band, dist_m, parent, 0.5, d_min=0.20, theta_sep=0.40)
    assert gvd_band.any()
    assert (dist_m[gvd_band] <= band + 1e-9).all()
    gvd_full = extract_gvd(free, dist_m, parent, 0.5, d_min=0.20, theta_sep=0.40)
    assert (dist_m[gvd_full] > band).any()   # unbanded reaches deeper


def test_run_gvd_band_reduces_and_records(tmp_path):
    from m3_adapter.gvd_to_vxw import run_gvd
    src = _make_box_vxw(tmp_path)
    banded = run_gvd(str(src), str(tmp_path / "b.vxw"), seed_metres=None,
                     d_min=0.20, theta_sep=0.40, pad=1, band_max=1.0)
    unbanded = run_gvd(str(src), str(tmp_path / "n.vxw"), seed_metres=None,
                       d_min=0.20, theta_sep=0.40, pad=1, band_max=None)
    assert banded["band_max"] == 1.0
    assert banded["gvd_voxels"] <= unbanded["gvd_voxels"]
    # stats expose both the (leaky) flood fraction and the post-band free fraction
    assert "flood_fraction" in banded and "free_fraction" in banded
    assert banded["free_fraction"] <= banded["flood_fraction"] + 1e-9
