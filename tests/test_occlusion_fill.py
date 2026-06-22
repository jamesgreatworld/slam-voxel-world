import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.occlusion import OcclusionFill

WALL = 19


def test_seals_enclosed_pinhole():
    # a solid 3x3x3 wall block with one UNKNOWN cell at the center (all 6 neighbors occupied)
    m = ObsMap.new((7, 7, 7), np.zeros(3, np.int64), 0.1)
    for x in range(2, 5):
        for y in range(2, 5):
            for z in range(2, 5):
                m.logodds[x, y, z] = 5.0; m.sem_label[x, y, z] = WALL
    m.logodds[3, 3, 3] = 0.0; m.sem_label[3, 3, 3] = 0   # unknown pinhole, 6 occ neighbors
    ov = run_pipeline(m, [OcclusionFill(min_neighbors=5)])
    occ, sem = compose_structure(m, ov)
    assert occ[3, 3, 3]                       # pinhole sealed
    assert not m.occupancy_mask()[3, 3, 3]    # L0 untouched


def test_does_not_fill_open_room():
    # a big empty room (observed-free) bounded by walls — must NOT be filled
    m = ObsMap.new((10, 10, 10), np.zeros(3, np.int64), 0.1)
    # walls around a 4x4x4 room of free air
    for x in range(2, 8):
        for y in range(2, 8):
            for z in range(2, 8):
                edge = x in (2, 7) or y in (2, 7) or z in (2, 7)
                if edge:
                    m.logodds[x, y, z] = 5.0; m.sem_label[x, y, z] = WALL
                else:
                    m.logodds[x, y, z] = -5.0   # observed-free interior
    ov = run_pipeline(m, [OcclusionFill(min_neighbors=5)])
    occ, sem = compose_structure(m, ov)
    assert not occ[4, 4, 4]                   # room interior stays open (it's observed_free anyway)
    assert not occ[5, 5, 5]


def test_skips_observed_free_cell():
    # an observed-free cell surrounded by occupied must NOT be filled (it's a known opening)
    m = ObsMap.new((7, 7, 7), np.zeros(3, np.int64), 0.1)
    for x in range(2, 5):
        for y in range(2, 5):
            for z in range(2, 5):
                m.logodds[x, y, z] = 5.0; m.sem_label[x, y, z] = WALL
    m.logodds[3, 3, 3] = -5.0                 # observed-free (a tiny window), NOT unknown
    ov = run_pipeline(m, [OcclusionFill(min_neighbors=5)])
    occ, sem = compose_structure(m, ov)
    assert not occ[3, 3, 3]                   # observed opening preserved
