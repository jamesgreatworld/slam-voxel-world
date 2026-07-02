"""ObsMap RGB 通道:命中端点累积观测颜色(运行均值),save/load 往返。"""
import numpy as np

from m3_adapter.obsmap import ObsMap


def _integrate_red_point(m, color=(200, 30, 30)):
    origin = np.array([0.05, 0.05, 0.05])
    pts = np.array([[0.95, 0.05, 0.05]])           # 沿 +x 的一条射线
    colors = np.array([color], dtype=np.uint8)
    m.integrate_frame(origin, pts, point_colors=colors, free_margin_m=0.05)


def test_rgb_recorded_at_hit_cell():
    m = ObsMap.new((20, 4, 4), vmin=(0, 0, 0), voxel_size=0.1)
    _integrate_red_point(m)
    assert m.rgb is not None
    hit = m.rgb[9, 0, 0]                            # 0.95/0.1 → cell 9
    assert tuple(hit) == (200, 30, 30)
    assert m.rgb_count[9, 0, 0] == 1
    # 传过的中途格没有颜色
    assert m.rgb_count[4, 0, 0] == 0


def test_rgb_running_mean():
    m = ObsMap.new((20, 4, 4), vmin=(0, 0, 0), voxel_size=0.1)
    _integrate_red_point(m, (100, 0, 0))
    _integrate_red_point(m, (200, 0, 0))
    assert m.rgb_count[9, 0, 0] == 2
    assert abs(int(m.rgb[9, 0, 0, 0]) - 150) <= 1   # 运行均值


def test_rgb_save_load_roundtrip(tmp_path):
    m = ObsMap.new((20, 4, 4), vmin=(0, 0, 0), voxel_size=0.1)
    _integrate_red_point(m)
    p = tmp_path / "m.npz"
    m.save(p)
    m2 = ObsMap.load(p)
    assert m2.rgb is not None
    assert tuple(m2.rgb[9, 0, 0]) == tuple(m.rgb[9, 0, 0])


def test_colorless_map_stays_lean_and_loads():
    m = ObsMap.new((20, 4, 4), vmin=(0, 0, 0), voxel_size=0.1)
    origin = np.array([0.05, 0.05, 0.05])
    m.integrate_frame(origin, np.array([[0.95, 0.05, 0.05]]))
    assert m.rgb is None                            # 不用颜色就不分配


def test_old_npz_without_rgb_loads(tmp_path):
    m = ObsMap.new((10, 4, 4), vmin=(0, 0, 0), voxel_size=0.1)
    p = tmp_path / "old.npz"
    m.save(p)                                       # rgb=None → 不写 rgb 键
    m2 = ObsMap.load(p)
    assert m2.rgb is None
