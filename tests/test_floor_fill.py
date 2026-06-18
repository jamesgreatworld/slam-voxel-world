import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.floor import FloorFill

FLOOR = 3


def _obs_holey_floor():
    # 10x6x10 grid. Floor plane at y=1 over a 6x6 footprint (x,z in 2..7),
    # with holes at (4,1,4) and (5,1,5). observed-free air above the footprint.
    m = ObsMap.new((10, 6, 10), np.zeros(3, np.int64), 0.1)
    for x in range(2, 8):
        for z in range(2, 8):
            if (x, z) in [(4, 4), (5, 5)]:
                continue
            m.logodds[x, 1, z] = 5.0
            m.sem_label[x, 1, z] = FLOOR
            m.logodds[x, 3, z] = -5.0
    m.logodds[4, 3, 4] = -5.0
    m.logodds[5, 3, 5] = -5.0
    return m


def test_floor_fill_patches_holes_and_keeps_observation():
    m = _obs_holey_floor()
    ov = run_pipeline(m, [FloorFill(close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[4, 1, 4] and sem[4, 1, 4] == FLOOR
    assert occ[5, 1, 5] and sem[5, 1, 5] == FLOOR
    assert occ[2, 1, 2] and sem[2, 1, 2] == FLOOR
    assert not m.occupancy_mask()[4, 1, 4]      # L0 untouched
    assert not occ[0, 1, 0]                     # no leak far outside footprint


def test_floor_fill_noop_without_floor():
    m = ObsMap.new((6, 6, 6), np.zeros(3, np.int64), 0.1)
    assert run_pipeline(m, [FloorFill()]).voxels == []
