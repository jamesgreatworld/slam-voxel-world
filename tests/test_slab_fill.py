import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.slab import SlabFill

FLOOR = 3
CEIL = 4


def _two_floor_obs():
    m = ObsMap.new((10, 20, 10), np.zeros(3, np.int64), 0.1)
    for x in range(2, 8):
        for z in range(2, 8):
            for Y in (3, 12):
                m.logodds[x, Y, z] = 5.0
                m.sem_label[x, Y, z] = FLOOR
            m.logodds[x, 5, z] = -5.0
            m.logodds[x, 14, z] = -5.0
    m.logodds[4, 3, 4] = 0.0
    m.sem_label[4, 3, 4] = 0
    return m


def test_slab_fill_multilevel_and_solidify():
    m = _two_floor_obs()
    ov = run_pipeline(m, [SlabFill(FLOOR, "floor", thickness_m=0.2, level_gap_m=0.5, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[3, 3, 3] and occ[3, 2, 3] and sem[3, 3, 3] == FLOOR
    assert occ[3, 12, 3] and occ[3, 11, 3]
    assert occ[4, 3, 4] and sem[4, 3, 4] == FLOOR
    assert not m.occupancy_mask()[3, 2, 3]
    assert not occ[3, 5, 3]


def test_slab_fill_ceiling_extrudes_up():
    m = ObsMap.new((8, 12, 8), np.zeros(3, np.int64), 0.1)
    for x in range(2, 6):
        for z in range(2, 6):
            m.logodds[x, 8, z] = 5.0
            m.sem_label[x, 8, z] = CEIL
            m.logodds[x, 5, z] = -5.0
    ov = run_pipeline(m, [SlabFill(CEIL, "ceiling", thickness_m=0.2, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[3, 8, 3] and occ[3, 9, 3] and sem[3, 9, 3] == CEIL


def test_slab_fill_stops_at_room_below():
    # floor surface at y=8; observed-free room air at y=5 (below). Solidify must
    # fill y=7,6 then STOP at the free cell y=5 — never fill y<=4 (below the room).
    m = ObsMap.new((6, 12, 6), np.zeros(3, np.int64), 0.1)
    for x in range(2, 5):
        for z in range(2, 5):
            m.logodds[x, 8, z] = 5.0; m.sem_label[x, 8, z] = FLOOR  # floor surface
            m.logodds[x, 10, z] = -5.0                              # free above -> footprint
            m.logodds[x, 5, z] = -5.0                               # room air below
    ov = run_pipeline(m, [SlabFill(FLOOR, "floor", thickness_m=1.0, close_radius=0)])  # T=10 deep
    occ, sem = compose_structure(m, ov)
    assert occ[3, 7, 3] and occ[3, 6, 3]      # solidified down to just above the room
    assert not occ[3, 5, 3]                   # stopped at observed-free room air
    assert not occ[3, 4, 3]                   # did NOT punch below the room


def test_slab_fill_skips_observed_free_below():
    m = ObsMap.new((6, 10, 6), np.zeros(3, np.int64), 0.1)
    for x in range(2, 5):
        for z in range(2, 5):
            m.logodds[x, 5, z] = 5.0
            m.sem_label[x, 5, z] = FLOOR   # floor surface at y=5
            m.logodds[x, 7, z] = -5.0      # free above -> footprint
    m.logodds[3, 4, 3] = -5.0              # observed-free directly below the floor
    ov = run_pipeline(m, [SlabFill(FLOOR, "floor", thickness_m=0.3, close_radius=1)])  # T=3 -> y=5,4,3
    occ, sem = compose_structure(m, ov)
    assert occ[3, 5, 3]            # surface present
    assert not occ[3, 4, 3]       # observed-free directly below is NOT filled (free-guard works)
    assert not occ[3, 3, 3]       # column stops at free cell — does NOT punch through to y=3
