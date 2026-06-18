import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.wall import WallFill

WALL = 19


def _wall_with_door_and_hole():
    # wall at fixed x=5, spanning y 2..8, z 2..8 (a y-z plane)
    m = ObsMap.new((10, 12, 12), np.zeros(3, np.int64), 0.1)
    for y in range(2, 9):
        for z in range(2, 9):
            m.logodds[5, y, z] = 5.0
            m.sem_label[5, y, z] = WALL
    # doorway: observed-free (saw through), y 2..4, z 5..6 — must NOT be filled
    for y in range(2, 5):
        for z in range(5, 7):
            m.logodds[5, y, z] = -5.0
            m.sem_label[5, y, z] = 0
    # a small UNobserved hole (unknown, logodds 0) at (5,6,6) — should be patched
    m.logodds[5, 6, 6] = 0.0
    m.sem_label[5, 6, 6] = 0
    return m


def test_wall_fill_patches_hole_preserves_door_and_thickens():
    m = _wall_with_door_and_hole()
    ov = run_pipeline(m, [WallFill(thickness_m=0.2, min_wall_cells=10, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[5, 6, 6] and sem[5, 6, 6] == WALL     # unobserved hole patched
    assert not occ[5, 3, 5]                          # doorway (observed_free) NOT filled
    assert occ[6, 6, 6] and sem[6, 6, 6] == WALL     # thickened along +x (T=2)
    assert not m.occupancy_mask()[5, 6, 6]           # L0 untouched


def test_wall_fill_thickens_perimeter_cells():
    m = _wall_with_door_and_hole()
    ov = run_pipeline(m, [WallFill(thickness_m=0.2, min_wall_cells=10, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    # a CORNER wall cell (perimeter of the bbox) must also be thickened along +x
    assert occ[5, 2, 2]            # original wall corner still there
    assert occ[6, 2, 2] and sem[6, 2, 2] == WALL   # perimeter thickened (regression: was missed)


def test_wall_fill_noop_without_wall():
    m = ObsMap.new((6, 6, 6), np.zeros(3, np.int64), 0.1)
    assert run_pipeline(m, [WallFill()]).voxels == []
