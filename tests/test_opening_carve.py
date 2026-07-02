"""OpeningCarve(stage-2):墙面开口检测 + 矩形/拱形拟合 + 规整挖空。"""
import numpy as np

from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.generators.opening import OpeningCarve
from m3_adapter.vlayer.generators.wall import WallFill
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.pipeline import run_pipeline

WALL = 19


def _wall_map(nx=40, ny=30, nz=12, z0=5, wall_h=28):
    """一面 z=z0 的墙(x∈[2,37], y∈[0,wall_h)),前方整片 observed-free(房间侧)。"""
    m = ObsMap.new((nx, ny, nz), vmin=(0, 0, 0), voxel_size=0.05)
    m.logodds[2:38, 0:wall_h, z0] = m.l_max
    m.sem_label[2:38, 0:wall_h, z0] = WALL
    m.logodds[2:38, 0:wall_h, z0 - 2] = m.l_min       # room side free
    return m


def _open_door(m, z0=5, x0=10, x1=18, y1=20):
    """在墙上开一扇门:门洞格从墙变为 observed-free,边缘留几个毛刺墙格。"""
    m.logodds[x0:x1, 0:y1, z0] = m.l_min
    m.sem_label[x0:x1, 0:y1, z0] = 0
    for x, y in [(x0, 3), (x0 + 1, 7), (x1 - 1, 12)]:   # 毛刺:洞内残墙
        m.logodds[x, y, z0] = m.l_max
        m.sem_label[x, y, z0] = WALL
    return (x0, x1, y1)


def test_ragged_door_carved_to_clean_rect():
    m = _wall_map()
    x0, x1, y1 = _open_door(m)
    ov = run_pipeline(m, [WallFill(), OpeningCarve(min_area_m2=0.2)])
    occ, sem = compose_structure(m, ov)
    hole = ~occ[x0:x1, 0:y1, 5]
    assert hole.all(), "门洞内(含毛刺残墙)应被完整挖空成矩形"
    assert occ[x0 - 2, 5, 5] and occ[x1 + 1, 5, 5], "门框外墙体必须保留"


def test_no_free_no_carve():
    m = _wall_map()          # 完整墙,无开口
    deltas = OpeningCarve().run(_ctx(m, [WallFill()]))
    assert deltas == []


def test_carve_deltas_are_removes_with_provenance():
    m = _wall_map()
    _open_door(m)
    deltas = OpeningCarve(min_area_m2=0.2).run(_ctx(m, [WallFill()]))
    assert deltas, "应产生挖空 deltas"
    assert all(d.op == "remove" and d.generator == "opening_carve" for d in deltas)


def test_arch_opening_keeps_top_corners():
    """拱形开口:矩形下部 + 半圆顶。拟合应选拱形,矩形上两角的墙保留。"""
    m = _wall_map(ny=40, wall_h=36)
    x0, x1, ytop = 10, 26, 30            # 宽 16,半圆顶半径 8:y∈[22,30)
    cx = (x0 + x1 - 1) / 2.0
    for x in range(x0, x1):
        for y in range(0, ytop):
            in_rect = y < ytop - 8
            in_arc = (not in_rect) and ((x - cx) ** 2 + (y - (ytop - 8)) ** 2 <= 8 ** 2)
            if in_rect or in_arc:
                m.logodds[x, y, 5] = m.l_min
                m.sem_label[x, y, 5] = 0
    ov = run_pipeline(m, [WallFill(), OpeningCarve(min_area_m2=0.2)])
    occ, sem = compose_structure(m, ov)
    assert occ[x0, ytop - 2, 5] and occ[x1 - 1, ytop - 2, 5], "拱顶两角墙体应保留(选了拱形而非矩形)"
    assert not occ[int(cx), ytop - 2, 5], "拱顶中央应挖空"


def test_unenclosed_free_above_half_wall_not_carved():
    """半高墙:墙顶以上的整条 free 空域不被墙包围 → 不是开口,不许挖。"""
    m = _wall_map(wall_h=22)
    m.logodds[2:38, 22:28, 5] = m.l_min          # 墙顶之上整条 free(向上开放)
    deltas = OpeningCarve(min_area_m2=0.2).run(_ctx(m, [WallFill()]))
    assert deltas == [], "开放空域被误判为开口"


def test_room_sized_free_region_not_carved():
    """房间量级的巨型 free 区(远超门窗尺寸上限)不是开口。"""
    m = ObsMap.new((80, 40, 12), vmin=(0, 0, 0), voxel_size=0.05)
    m.logodds[2:78, 0:36, 5] = m.l_max           # 3.8m 宽 × 1.8m 高的墙
    m.sem_label[2:78, 0:36, 5] = WALL
    m.logodds[2:78, 0:36, 3] = m.l_min           # 房间侧 free
    # 墙中部 3.0m 宽 × 1.5m 高的巨洞(60×30 格,宽度 > max_extent 2.6m/52格)
    m.logodds[10:70, 3:33, 5] = m.l_min
    m.sem_label[10:70, 3:33, 5] = 0
    deltas = OpeningCarve(min_area_m2=0.2).run(_ctx(m, [WallFill()]))
    assert deltas == [], "巨型空域被误判为门窗"


def _ctx(m, pre_generators):
    """构造带前序 generator 产出的 LcContext(模拟 run_pipeline 到 stage-2 时的状态)。"""
    from m3_adapter.vlayer.pipeline import LcContext
    from m3_adapter.vlayer.overlay import Overlay
    ov = Overlay()
    ctx = LcContext(obsmap=m, overlay=ov)
    for g in pre_generators:
        ov.voxels.extend(g.run(ctx))
    return ctx
