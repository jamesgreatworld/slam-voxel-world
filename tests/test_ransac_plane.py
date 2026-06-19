import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.plane import RansacPlaneFill

CEIL = 4


def _slanted_roof(hole=(7, 5), free_cell=None):
    # plane z = round(5 + 0.5*x) over x in 2..12, y in 2..8, labelled ceiling(4)
    m = ObsMap.new((20, 20, 20), np.zeros(3, np.int64), 0.1)
    for x in range(2, 13):
        z = int(round(5 + 0.5 * x))
        for y in range(2, 9):
            if (x, y) == hole:
                continue
            m.logodds[x, y, z] = 5.0
            m.sem_label[x, y, z] = CEIL
    if free_cell is not None:
        x, y = free_cell
        z = int(round(5 + 0.5 * x))
        m.logodds[x, y, z] = -5.0   # observed-free skylight opening on the plane
    return m


def test_fills_slanted_plane_hole():
    m = _slanted_roof(hole=(7, 5))
    ov = run_pipeline(m, [RansacPlaneFill(labels=(CEIL,), min_inliers=40, dist_thresh=1.5)])
    occ, sem = compose_structure(m, ov)
    zc = int(round(5 + 0.5 * 7))
    assert occ[7, 5, zc] and sem[7, 5, zc] == CEIL     # hole on slanted plane filled
    assert not m.occupancy_mask()[7, 5, zc]            # L0 untouched


def test_preserves_observed_free_opening():
    m = _slanted_roof(hole=None, free_cell=(8, 6))
    ov = run_pipeline(m, [RansacPlaneFill(labels=(CEIL,), min_inliers=40, dist_thresh=1.5)])
    occ, sem = compose_structure(m, ov)
    zc = int(round(5 + 0.5 * 8))
    assert not occ[8, 6, zc]                           # skylight (observed_free) not filled


def test_noop_below_min_inliers():
    m = ObsMap.new((10, 10, 10), np.zeros(3, np.int64), 0.1)
    m.logodds[2, 2, 2] = 5.0; m.sem_label[2, 2, 2] = CEIL   # a single voxel, no plane
    assert run_pipeline(m, [RansacPlaneFill(labels=(CEIL,), min_inliers=40)]).voxels == []
