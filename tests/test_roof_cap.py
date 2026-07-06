"""RoofCap:屋面观测缺口补洞;楼梯井(observed-free)永不封。"""
import numpy as np

from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.generators.roof import RoofCap
from m3_adapter.vlayer.overlay import Overlay, compose_structure
from m3_adapter.vlayer.pipeline import LcContext

CEIL = 4


def _map_with_ring_roof():
    """20x10x20:y=8 的屋面是一个带 4x4 中央洞的环。"""
    m = ObsMap.new((20, 10, 20), vmin=(0, 0, 0), voxel_size=0.1)
    m.logodds[2:18, 8, 2:18] = m.l_max
    m.sem_label[2:18, 8, 2:18] = CEIL
    m.logodds[8:12, 8, 8:12] = 0.0        # 观测缺口(未知)
    m.sem_label[8:12, 8, 8:12] = 0
    return m


def _run(m):
    ctx = LcContext(obsmap=m, overlay=Overlay())
    ctx.overlay.voxels.extend(RoofCap().run(ctx))
    return compose_structure(m, ctx.overlay)


def test_observation_gap_capped():
    occ, sem = _run(_map_with_ring_roof())
    assert occ[9, 8, 9], "被屋面包围的观测缺口应补上"
    assert sem[9, 8, 9] == CEIL


def test_stairwell_free_never_capped():
    m = _map_with_ring_roof()
    m.logodds[8:12, 8, 8:12] = m.l_min     # 同一洞,但是 observed-free(楼梯井)
    occ, _ = _run(m)
    assert not occ[8:12, 8, 8:12].any(), "observed-free 开口永不封"


def test_outside_boundary_untouched():
    occ, _ = _run(_map_with_ring_roof())
    assert not occ[0, 8, 0], "观测边界之外不外推"
