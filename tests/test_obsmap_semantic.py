import numpy as np
from m3_adapter.obsmap import ObsMap


def _hit(m, label):
    # a ray that ends at cell (8,5,5) with the given label
    m.integrate_frame(np.array([0.5, 5.5, 5.5]), np.array([[8.5, 5.5, 5.5]]),
                      point_labels=np.array([label]), free_margin_m=1.0)


def test_majority_winner_simple():
    m = ObsMap.new((15, 12, 12), np.zeros(3, np.int64), 1.0)
    for L in (3, 3, 7, 3):     # 3 wins (3 votes vs 1)
        _hit(m, L)
    assert m.semantic_grid()[8, 5, 5] == 3


def test_majority_switches_with_opposition():
    m = ObsMap.new((15, 12, 12), np.zeros(3, np.int64), 1.0)
    for L in (3, 7, 9, 7, 7):  # Boyer-Moore: 7 ends as winner
        _hit(m, L)
    assert m.semantic_grid()[8, 5, 5] == 7


def test_semantic_save_load_roundtrip(tmp_path):
    m = ObsMap.new((15, 12, 12), np.zeros(3, np.int64), 1.0)
    _hit(m, 5); _hit(m, 5)
    p = tmp_path / "obsmap.npz"
    m.save(p)
    m2 = ObsMap.load(p)
    assert np.array_equal(m.sem_label, m2.sem_label)
    assert np.array_equal(m.sem_count, m2.sem_count)
    assert m2.semantic_grid()[8, 5, 5] == 5


def test_geometry_still_works_without_labels(tmp_path):
    # back-compat: integrate_frame with no labels behaves as before
    m = ObsMap.new((20, 5, 5), np.zeros(3, np.int64), 1.0)
    m.integrate_frame(np.array([0.5, 2.5, 2.5]), np.array([[15.5, 2.5, 2.5]]), free_margin_m=1.0)
    assert m.occupancy_mask()[15, 2, 2]
    assert m.observed_free_mask()[5, 2, 2]
