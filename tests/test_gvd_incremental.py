import numpy as np
from m3_adapter.gvd.field import extract_gvd, compute_esdf, extract_gvd_local
from m3_adapter.obsmap import ObsMap


def test_local_gvd_matches_global_in_core_box():
    # two point obstacles along x; global GVD has the bisector plane x=15
    n = 31
    occ = np.zeros((n, n, n), dtype=bool)
    occ[5, 15, 15] = True
    occ[25, 15, 15] = True
    free = ~occ
    dist, parent = compute_esdf(occ, 1.0)
    g_global = extract_gvd(free, dist, parent, 1.0, d_min=1.0, theta_sep=2.0)
    # core box around the bisector, generous margin
    bbox_min = np.array([12, 8, 8]); bbox_max = np.array([19, 23, 23])
    g_local = extract_gvd_local(occ, free, 1.0, bbox_min, bbox_max, margin_vox=10,
                                d_min=1.0, theta_sep=2.0)
    # inside the core box, local == global
    sl = tuple(slice(bbox_min[a], bbox_max[a]) for a in range(3))
    assert np.array_equal(g_local[sl], g_global[sl])
    # outside the core box, local is empty
    g_local[sl] = False
    assert g_local.sum() == 0


def test_obsmap_dirty_bbox_tracks_touched_cells():
    m = ObsMap.new((20, 20, 20), np.zeros(3, np.int64), 1.0)
    assert m.pop_dirty_bbox() is None
    m.integrate_frame(np.array([2.5, 10.5, 10.5]), np.array([[8.5, 10.5, 10.5]]),
                      free_margin_m=1.0)
    bb = m.pop_dirty_bbox()
    assert bb is not None
    lo, hi = bb
    # touched cells run roughly x in [2..8], y=z=10 -> bbox covers them
    assert lo[0] <= 3 and hi[0] >= 8
    assert lo[1] <= 10 < hi[1]
    # popping again is empty
    assert m.pop_dirty_bbox() is None
