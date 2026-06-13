import numpy as np
import pytest
from m3_adapter.voxel_gvd import flood_free_space


def _hollow_box(inner=19):
    """Solid shell cube. Returns (occupied_bool, interior_center_idx).
    Grid is (inner+2)^3; walls on every outer face; interior 1..inner free."""
    n = inner + 2
    occ = np.zeros((n, n, n), dtype=bool)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    center = (n // 2, n // 2, n // 2)
    return occ, center


def test_flood_fills_box_interior():
    occ, center = _hollow_box(inner=19)
    free = flood_free_space(occ, center)
    # interior is 19^3 free cells, all reachable from centre
    assert free.sum() == 19 ** 3
    assert free[center]
    # walls are never free
    assert not free[0, 0, 0]


def test_flood_seed_in_obstacle_raises():
    occ, _ = _hollow_box(inner=19)
    with pytest.raises(ValueError):
        flood_free_space(occ, (0, 0, 0))


def test_flood_does_not_leak_through_sealed_wall():
    # two 5^3 rooms side by side sharing a solid wall -> seed in room A
    # must NOT reach room B.
    occ = np.zeros((13, 7, 7), dtype=bool)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    occ[6, :, :] = True          # solid dividing wall at x=6
    free = flood_free_space(occ, (3, 3, 3))   # room A (x=1..5)
    assert free[3, 3, 3]
    assert not free[9, 3, 3]     # room B (x=7..11) unreachable
    # a 1-voxel hole in the divider DOES leak (demonstrates the risk)
    occ[6, 3, 3] = False
    free2 = flood_free_space(occ, (3, 3, 3))
    assert free2[9, 3, 3]
