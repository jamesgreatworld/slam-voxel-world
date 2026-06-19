import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.stairs import StairsFill

STAIRS = 15


def _staircase():
    # 3 steps ascending in x: step at x=2->y=2, x=3->y=3, x=4->y=4, over z 2..5.
    # floor at y=1 (so fill should go down to y=1).
    m = ObsMap.new((10, 10, 10), np.zeros(3, np.int64), 0.1)
    for z in range(2, 6):
        for x in range(2, 5):
            y = x                     # step rises with x
            m.logodds[x, y, z] = 5.0
            m.sem_label[x, y, z] = STAIRS
        m.logodds[1, 1, z] = 5.0      # floor reference at y=1
        m.sem_label[1, 1, z] = 3
    return m


def test_stairs_solidified_down():
    m = _staircase()
    ov = run_pipeline(m, [StairsFill(max_depth_m=1.0)])
    occ, sem = compose_structure(m, ov)
    # the step at x=4 (top, y=4) must be solid down to y=1
    for y in range(1, 5):
        assert occ[4, y, 3]
    assert sem[4, 2, 3] == STAIRS          # filled cells labelled stairs
    assert not m.occupancy_mask()[4, 2, 3] # L0 untouched


def test_stairs_preserves_observed_free():
    m = _staircase()
    m.logodds[4, 2, 3] = -5.0              # observed-free under the top step
    ov = run_pipeline(m, [StairsFill(max_depth_m=1.0)])
    occ, sem = compose_structure(m, ov)
    assert not occ[4, 2, 3]                # observed-free not filled


def test_noop_without_stairs():
    m = ObsMap.new((6, 6, 6), np.zeros(3, np.int64), 0.1)
    assert run_pipeline(m, [StairsFill()]).voxels == []
