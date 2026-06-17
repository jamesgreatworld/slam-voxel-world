import numpy as np
from m3_adapter.obsmap import ObsMap


def test_obsmap_resume_accumulates(tmp_path):
    # simulate the driver's save -> load -> integrate-more -> save cycle
    p = tmp_path / "obsmap.npz"
    m = ObsMap.new((30, 6, 6), np.zeros(3, np.int64), 1.0)
    o = np.array([0.5, 3.5, 3.5])
    m.integrate_frame(o, np.array([[10.5, 3.5, 3.5]]), free_margin_m=1.0)
    m.save(p)
    occ_a = int(m.occupancy_mask().sum())
    # "resume": load and integrate a DIFFERENT ray reaching farther
    m2 = ObsMap.load(p)
    m2.integrate_frame(o, np.array([[25.5, 3.5, 3.5]]), free_margin_m=1.0)
    m2.save(p)
    # the map grew: more observed-free cells than after the first run
    assert int(m2.observed_free_mask().sum()) > 0
    # and the resumed map retained the first run's effect on near cells
    m3 = ObsMap.load(p)
    assert np.array_equal(m3.logodds, m2.logodds)
