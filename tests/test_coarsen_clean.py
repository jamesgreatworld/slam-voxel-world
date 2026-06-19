import numpy as np
from m3_adapter.vlayer.coarsen import clean_coarse


def test_removes_isolated_small_component():
    occ = np.zeros((8, 8, 8), bool); sem = np.zeros((8, 8, 8), np.uint8)
    # a big blob (kept) and a lone isolated voxel (removed at min_component=2)
    occ[1:5, 1:5, 1:5] = True; sem[1:5, 1:5, 1:5] = 3
    occ[7, 7, 7] = True; sem[7, 7, 7] = 19
    oc, se = clean_coarse(occ, sem, min_component=2, close_radius=0)
    assert oc[2, 2, 2]                 # big blob kept
    assert not oc[7, 7, 7]             # isolated voxel removed
    assert se[7, 7, 7] == 0            # its sem cleared


def test_fills_small_hole_and_assigns_neighbor_sem():
    occ = np.ones((6, 6, 6), bool); sem = np.full((6, 6, 6), 3, np.uint8)
    occ[3, 3, 3] = False; sem[3, 3, 3] = 0      # a 1-cell hole inside a solid block
    oc, se = clean_coarse(occ, sem, min_component=1, close_radius=1)
    assert oc[3, 3, 3]                 # hole filled by closing
    assert se[3, 3, 3] == 3            # filled cell took neighbor sem (3), not 0


def test_keeps_clean_input_unchanged():
    occ = np.zeros((6, 6, 6), bool); sem = np.zeros((6, 6, 6), np.uint8)
    occ[1:5, 1:5, 1:5] = True; sem[1:5, 1:5, 1:5] = 3
    oc, se = clean_coarse(occ, sem, min_component=2, close_radius=1)
    assert int(oc.sum()) == int(occ.sum())     # solid cube: nothing removed/added
