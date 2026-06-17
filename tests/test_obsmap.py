import numpy as np
from m3_adapter.obsmap import ObsMap


def test_hit_makes_occupied_miss_makes_free():
    m = ObsMap.new((20, 5, 5), np.zeros(3, np.int64), 1.0)
    # one ray from (0.5,2.5,2.5) to (15.5,2.5,2.5)
    o = np.array([0.5, 2.5, 2.5]); p = np.array([[15.5, 2.5, 2.5]])
    m.integrate_frame(o, p, free_margin_m=1.0)
    occ = m.occupancy_mask(); free = m.observed_free_mask()
    assert occ[15, 2, 2]          # endpoint -> occupied
    assert free[5, 2, 2]          # traversed interior -> free
    assert not occ[5, 2, 2]


def test_repeated_miss_deletes_occupied():
    # a cell first seen occupied, then repeatedly seen free -> flips to free
    m = ObsMap.new((20, 5, 5), np.zeros(3, np.int64), 1.0)
    # frame 1: surface AT x=8 (so cell 8 becomes occupied)
    m.integrate_frame(np.array([0.5, 2.5, 2.5]), np.array([[8.5, 2.5, 2.5]]), free_margin_m=1.0)
    assert m.occupancy_mask()[8, 2, 2]
    # later frames: rays pass THROUGH x=8 to a farther surface -> miss on cell 8
    for _ in range(20):
        m.integrate_frame(np.array([0.5, 2.5, 2.5]), np.array([[18.5, 2.5, 2.5]]), free_margin_m=1.0)
    assert not m.occupancy_mask()[8, 2, 2]   # deleted (flipped occupied->free)
    assert m.observed_free_mask()[8, 2, 2]


def test_save_load_roundtrip(tmp_path):
    m = ObsMap.new((10, 10, 10), np.array([-3, -3, -3], np.int64), 0.5)
    m.integrate_frame(np.array([0.0, 0.0, 0.0]), np.array([[3.0, 0.0, 0.0]]), free_margin_m=0.5)
    p = tmp_path / "obsmap.npz"
    m.save(p)
    m2 = ObsMap.load(p)
    assert np.array_equal(m.logodds, m2.logodds)
    assert np.array_equal(m.vmin, m2.vmin)
    assert m2.voxel_size == 0.5 and m2.occ_thr == m.occ_thr
