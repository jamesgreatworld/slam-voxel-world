"""PlaneRegularize:形状先验 × 观测证据的概率规整(墙=矩形)。
离矩形边界越深的内部缺格越可能是洞(补);离矩形越远的外部有格越可能是噪声(删)。"""
import numpy as np

from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.generators.regularize import PlaneRegularize
from m3_adapter.vlayer.generators.wall import WallFill
from m3_adapter.vlayer.overlay import Overlay, compose_structure
from m3_adapter.vlayer.pipeline import LcContext

WALL = 19


def _wall(nx=40, ny=30, nz=12, z0=5, wall_h=28):
    m = ObsMap.new((nx, ny, nz), vmin=(0, 0, 0), voxel_size=0.05)
    m.logodds[2:38, 0:wall_h, z0] = m.l_max
    m.sem_label[2:38, 0:wall_h, z0] = WALL
    return m


def _run(m, gen):
    ctx = LcContext(obsmap=m, overlay=Overlay())
    ctx.overlay.voxels.extend(WallFill().run(ctx))
    ctx.overlay.voxels.extend(gen.run(ctx))
    return ctx.overlay


def test_deep_interior_unknown_hole_filled():
    m = _wall()
    # 墙中央挖一个 5x5 的"未观测"洞(非 free!):深处应按先验补上
    m.logodds[15:20, 10:15, 5] = 0.0
    m.sem_label[15:20, 10:15, 5] = 0
    ov = _run(m, PlaneRegularize())
    occ, sem = compose_structure(m, ov)
    assert occ[17, 12, 5], "矩形深处的未知洞应被先验补上"
    assert sem[17, 12, 5] == WALL


def test_observed_free_never_filled():
    m = _wall()
    # 同样位置但为 observed-free(窗):绝不许补
    m.logodds[15:20, 10:15, 5] = m.l_min
    m.sem_label[15:20, 10:15, 5] = 0
    ov = _run(m, PlaneRegularize())
    occ, _ = compose_structure(m, ov)
    assert not occ[15:20, 10:15, 5].any(), "observed-free 是证据,先验不得覆盖"


def test_far_outside_stray_removed_near_kept():
    m = _wall(ny=42)
    # 贴着矩形上沿的凸起(1格)保留;远离矩形 10 格的飘砖删除
    m.logodds[10, 28, 5] = m.l_max; m.sem_label[10, 28, 5] = WALL     # 近:d=1
    m.logodds[20, 38 - 1, 5] = m.l_max; m.sem_label[20, 38 - 1, 5] = WALL  # 远:d≈10
    ov = _run(m, PlaneRegularize())
    occ, _ = compose_structure(m, ov)
    assert occ[10, 28, 5], "贴边凸起(先验弱)应保留"
    assert not occ[20, 37, 5], "远离矩形的飘砖(先验强)应删除"


def test_no_wall_no_deltas():
    m = ObsMap.new((10, 10, 10), vmin=(0, 0, 0), voxel_size=0.05)
    ctx = LcContext(obsmap=m, overlay=Overlay())
    assert PlaneRegularize().run(ctx) == []
