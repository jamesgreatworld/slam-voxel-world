import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.wall import WallFill

WALL = 19


def _wall_with_door_and_hole():
    # Z-running wall at fixed x=5, plane y2..8 z2..8. No free on either x side
    # (both unknown) -> tie -> thicken defaults to +x.
    m = ObsMap.new((10, 12, 12), np.zeros(3, np.int64), 0.1)
    for y in range(2, 9):
        for z in range(2, 9):
            m.logodds[5, y, z] = 5.0; m.sem_label[5, y, z] = WALL
    for y in range(2, 5):              # doorway: observed-free, must NOT fill
        for z in range(5, 7):
            m.logodds[5, y, z] = -5.0; m.sem_label[5, y, z] = 0
    m.logodds[5, 6, 6] = 0.0; m.sem_label[5, 6, 6] = 0   # unobserved small hole
    return m


def test_patch_hole_preserve_door_thicken_and_perimeter():
    m = _wall_with_door_and_hole()
    ov = run_pipeline(m, [WallFill(thickness_m=0.2, min_wall_cells=10, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[5, 6, 6] and sem[5, 6, 6] == WALL    # hole patched
    assert not occ[5, 3, 5]                          # doorway (free) preserved
    assert occ[6, 6, 6] and sem[6, 6, 6] == WALL     # thickened (tie -> +x), interior
    assert occ[6, 2, 2]                              # perimeter cell thickened too
    assert not m.occupancy_mask()[5, 6, 6]           # L0 untouched


def test_thickens_toward_back_not_into_room():
    # Z-wall at x=5; the room (observed-free) is on +x (x=6). Back is -x (x=4, unknown).
    m = ObsMap.new((10, 12, 12), np.zeros(3, np.int64), 0.1)
    for y in range(2, 9):
        for z in range(2, 9):
            m.logodds[5, y, z] = 5.0; m.sem_label[5, y, z] = WALL
            m.logodds[6, y, z] = -5.0          # room (free) on +x side
    m.logodds[5, 6, 6] = 0.0; m.sem_label[5, 6, 6] = 0   # unobserved hole
    ov = run_pipeline(m, [WallFill(thickness_m=0.2, min_wall_cells=10, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[5, 6, 6]            # hole patched
    assert occ[4, 6, 6]           # thickened toward BACK (-x), into unknown
    assert not occ[6, 6, 6]       # NOT into the room (+x, free)


def test_small_blob_is_not_a_wall():
    # a 3x3x1 wall patch (9 cells, height 3) — below min_wall_cells/min_height -> ignored
    m = ObsMap.new((8, 8, 8), np.zeros(3, np.int64), 0.1)
    for y in range(2, 5):
        for z in range(2, 5):
            m.logodds[4, y, z] = 5.0; m.sem_label[4, y, z] = WALL
    assert run_pipeline(m, [WallFill(min_wall_cells=20, min_height=4)]).voxels == []


def test_z_wall_not_extended_along_its_length():
    # The Z-wall spans z 2..8; v2 must NOT add wall cells outside that z-range
    # (the old z-slice double-processing bug did).
    m = _wall_with_door_and_hole()
    ov = run_pipeline(m, [WallFill(thickness_m=0.2, min_wall_cells=10, close_radius=1)])
    for d in ov.voxels:
        assert 2 <= d.idx[2] <= 8       # no spurious extension in z


def test_noop_without_wall():
    m = ObsMap.new((6, 6, 6), np.zeros(3, np.int64), 0.1)
    assert run_pipeline(m, [WallFill()]).voxels == []
