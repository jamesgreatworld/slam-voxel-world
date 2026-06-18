import numpy as np
from m3_adapter.vlayer.coarsen import downsample_occupancy


def test_thin_floor_becomes_solid_coarse_block():
    occ = np.zeros((4, 4, 4), bool); sem = np.zeros((4, 4, 4), np.uint8)
    occ[:, 1, :] = True; sem[:, 1, :] = 3            # 1-cell-thick floor at y=1
    occ_c, sem_c, vmin_c, vs_c = downsample_occupancy(occ, sem, np.zeros(3, np.int64), 0.05, 2)
    assert occ_c.shape == (2, 2, 2)
    assert occ_c[0, 0, 0] and sem_c[0, 0, 0] == 3    # block covering y=0,1 -> solid floor
    assert not occ_c[0, 1, 0]                         # block covering y=2,3 -> empty
    assert abs(vs_c - 0.1) < 1e-9
    assert tuple(vmin_c) == (0, 0, 0)


def test_empty_region_stays_empty_occupied_block_solid():
    occ = np.zeros((4, 4, 4), bool); sem = np.zeros((4, 4, 4), np.uint8)
    occ[0, 0, 0] = True; sem[0, 0, 0] = 19
    occ_c, sem_c, _, _ = downsample_occupancy(occ, sem, np.zeros(3, np.int64), 0.05, 2)
    assert occ_c[0, 0, 0] and sem_c[0, 0, 0] == 19   # any fine occupied -> coarse solid
    assert not occ_c[1, 1, 1]                         # far empty block stays empty


def test_semantic_majority_among_occupied():
    occ = np.zeros((2, 2, 2), bool); sem = np.zeros((2, 2, 2), np.uint8)
    # one 2x2x2 block, 3 floor(3) + 1 wall(19) occupied -> majority floor
    occ[:] = True
    sem[:] = 3; sem[0, 0, 0] = 19
    occ_c, sem_c, _, _ = downsample_occupancy(occ, sem, np.zeros(3, np.int64), 0.05, 2)
    assert occ_c.shape == (1, 1, 1) and occ_c[0, 0, 0]
    assert sem_c[0, 0, 0] == 3


def test_vmin_alignment_negative():
    # vmin not a multiple of factor and negative -> pad_lo aligns to factor boundary
    occ = np.zeros((4, 4, 4), bool); sem = np.zeros((4, 4, 4), np.uint8)
    occ[0, 0, 0] = True
    vmin = np.array([-3, -3, -3], np.int64)          # -3 % 2 = 1 -> pad_lo=1
    occ_c, sem_c, vmin_c, vs_c = downsample_occupancy(occ, sem, vmin, 0.05, 2)
    # world pos of fine cell (0,..) = (0 + -3)*0.05 = -0.15; coarse vmin_c*vs_c must align:
    assert tuple(vmin_c) == (-2, -2, -2)             # (-3 - 1)//2 = -2
    assert abs(vs_c - 0.1) < 1e-9


def test_factor_one_is_identity():
    occ = np.zeros((3, 3, 3), bool); occ[1, 1, 1] = True
    sem = np.zeros((3, 3, 3), np.uint8); sem[1, 1, 1] = 5
    occ_c, sem_c, vmin_c, vs_c = downsample_occupancy(occ, sem, np.zeros(3, np.int64), 0.05, 1)
    assert occ_c[1, 1, 1] and sem_c[1, 1, 1] == 5 and abs(vs_c - 0.05) < 1e-9
