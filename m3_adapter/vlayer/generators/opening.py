"""stage-2 结构先验:OpeningCarve —— 门/窗开口的形状拟合与规整挖空(四阶段流程的 ②)。

原理:传感器射线只有穿过门窗才会在**墙平面本身**留下 observed-free,所以
"墙平面上的 free 连通域" 即开口。观测到的开口边缘是毛糙的(噪声残墙、
未观测的门楣被 WallFill 补上等);本 generator 对每个开口拟合规整形状——
**矩形**(鲁棒修剪的包围盒)或**拱形**(矩形 + 半圆顶),按 IoU 择优——
然后按拟合形状发 `remove` deltas,跨墙厚挖穿,把开口规整化。

只读 L0;通过 ctx.overlay 读取 wall_fill 已产出的补墙格(stage 依赖),
remove 同时作用于观测残墙与补墙(输出层纠正,L0 不变)。对任何场景通用。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.generators.wall import WALL_LABEL, projection_peaks
from m3_adapter.vlayer.overlay import VoxelDelta

_CC8 = ndimage.generate_binary_structure(2, 2)


class OpeningCarve:
    stage = 2
    depends_on = ["wall_fill"]
    default_binding = "persistent"

    def __init__(self, min_area_m2: float = 0.35, max_area_m2: float = 4.0,
                 max_extent_m: float = 2.6, enclosure_min: float = 0.5,
                 thickness_m: float = 0.10,
                 min_wall_cells: int = 20, min_height: int = 4, min_run: int = 4,
                 trim_fill: float = 0.3):
        # 真实的墙不是满矩形(L 形/半段墙):bbox 内的大片房间 free 不是开口。
        # 三条从"门窗是什么"推出的通用判据把它们滤掉:
        #   max_area/max_extent —— 门窗有界(巨型 free 区不是开口);
        #   enclosure_min —— 开口被实体包围(墙环绕/地在下),墙尽头的 free 不被包围;
        #   挖除只作用于墙格(见 _carve_plane),绝不误伤地板/天花板/物体。
        self.min_area_m2 = float(min_area_m2)
        self.max_area_m2 = float(max_area_m2)
        self.max_extent_m = float(max_extent_m)
        self.enclosure_min = float(enclosure_min)
        self.thickness_m = float(thickness_m)
        self.min_wall_cells = int(min_wall_cells)
        self.min_height = int(min_height); self.min_run = int(min_run)
        self.trim_fill = float(trim_fill)
        self.id = "opening_carve"; self.generator = "opening_carve"

    # ------------------------------------------------------------------ run
    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        vs = obs.voxel_size
        wall = occ & (sem == WALL_LABEL)
        if not wall.any():
            return []
        min_cells = max(4, int(round(self.min_area_m2 / (vs * vs))))
        self._vs_cells = (int(round(self.max_area_m2 / (vs * vs))),
                          int(round(self.max_extent_m / vs)))
        T = max(1, int(np.ceil(self.thickness_m / vs - 1e-9)))
        added = {d.idx for d in ctx.overlay.voxels
                 if d.generator == "wall_fill" and d.op == "add"}
        deltas = []
        emitted = set()
        nx, ny, nz = occ.shape
        xcount = wall.sum(axis=(1, 2)); zcount = wall.sum(axis=(0, 1))
        # 统一把每面墙整理成 plane2d[v, h](v = y 竖直向上),再映射回体素坐标。
        for c0 in projection_peaks(xcount, self.min_wall_cells):
            added2d = {(y, z) for (x, y, z) in added if x == c0}
            self._carve_plane(wall[c0, :, :], free[c0, :, :], occ[c0, :, :],
                              added2d, min_cells,
                              lambda v, h, dc, c0=c0: (c0 + dc, v, h),
                              wall, added, T, (nx, ny, nz), deltas, emitted)
        for c0 in projection_peaks(zcount, self.min_wall_cells):
            added2d = {(y, x) for (x, y, z) in added if z == c0}
            self._carve_plane(wall[:, :, c0].T, free[:, :, c0].T, occ[:, :, c0].T,
                              added2d, min_cells,
                              lambda v, h, dc, c0=c0: (h, v, c0 + dc),
                              wall, added, T, (nx, ny, nz), deltas, emitted)
        return deltas

    # ------------------------------------------------------- per-wall plane
    def _carve_plane(self, wall2d, free2d, occ2d, added2d, min_cells, to_xyz,
                     wall3d, added, T, dims, deltas, emitted):
        if int(wall2d.sum()) < self.min_wall_cells:
            return
        vs_cells = self._vs_cells        # (max_cells, max_ext) 由 run() 预计算
        max_cells, max_ext = vs_cells
        vv, hh = np.nonzero(wall2d)
        v0, v1, h0, h1 = int(vv.min()), int(vv.max()), int(hh.min()), int(hh.max())
        # 与 WallFill 相同的结构判定:高且延展,滤掉贯穿投影产生的伪平面
        if (v1 - v0 + 1) < self.min_height or (h1 - h0 + 1) < self.min_run:
            return
        inside = np.zeros_like(free2d, dtype=bool)
        inside[v0:v1 + 1, h0:h1 + 1] = True
        # 实体面 = 观测占据(任意标签,含开口下沿的地板)∪ wall_fill 补墙
        solid2d = occ2d.copy()
        for (a, b) in added2d:
            if 0 <= a < solid2d.shape[0] and 0 <= b < solid2d.shape[1]:
                solid2d[a, b] = True
        lbl, n = ndimage.label(free2d & inside, structure=_CC8)
        nx, ny, nz = dims
        for cid in range(1, n + 1):
            comp = lbl == cid
            csize = int(comp.sum())
            if csize < min_cells or csize > max_cells:
                continue                          # 太小=噪声;太大=房间空域,非门窗
            rv0, rv1, rh0, rh1 = self._robust_rect(comp)
            if (rv1 - rv0 + 1) > max_ext or (rh1 - rh0 + 1) > max_ext:
                continue                          # 门窗尺寸有界
            # 包围判据:开口四周应是实体(墙环绕/地在下);墙尽头的 free 不被包围
            border = ndimage.binary_dilation(comp, structure=_CC8) & ~comp
            nb = int(border.sum())
            if nb == 0 or (border & solid2d).sum() / float(nb) < self.enclosure_min:
                continue
            local = comp[rv0:rv1 + 1, rh0:rh1 + 1]
            template = self._fit_template(local)
            for lv, lh in np.argwhere(template):
                V, H = rv0 + int(lv), rh0 + int(lh)
                for dc in range(-T, T + 1):
                    x, y, z = to_xyz(V, H, dc)
                    if not (0 <= x < nx and 0 <= y < ny and 0 <= z < nz):
                        continue
                    idx = (int(x), int(y), int(z))
                    if idx in emitted:
                        continue
                    # 只挖墙:观测残墙(label 19)或 wall_fill 补墙,不碰地板/天花板/物体
                    if wall3d[idx] or idx in added:
                        emitted.add(idx)
                        deltas.append(VoxelDelta(idx, "remove", 0,
                                                 self.generator, self.default_binding))

    # ------------------------------------------------------------ 形状拟合
    def _robust_rect(self, comp):
        """开口的鲁棒包围盒:迭代剔除填充率 < trim_fill 的边缘行/列(去毛边外飘)。"""
        v0, v1 = int(np.nonzero(comp.any(axis=1))[0].min()), int(np.nonzero(comp.any(axis=1))[0].max())
        h0, h1 = int(np.nonzero(comp.any(axis=0))[0].min()), int(np.nonzero(comp.any(axis=0))[0].max())
        changed = True
        while changed and v1 > v0 and h1 > h0:
            changed = False
            w = h1 - h0 + 1; hgt = v1 - v0 + 1
            if comp[v0, h0:h1 + 1].sum() < self.trim_fill * w:
                v0 += 1; changed = True; continue
            if comp[v1, h0:h1 + 1].sum() < self.trim_fill * w:
                v1 -= 1; changed = True; continue
            if comp[v0:v1 + 1, h0].sum() < self.trim_fill * hgt:
                h0 += 1; changed = True; continue
            if comp[v0:v1 + 1, h1].sum() < self.trim_fill * hgt:
                h1 -= 1; changed = True
        return v0, v1, h0, h1

    def _fit_template(self, local):
        """在修剪后的包围盒内选形状:满矩形 vs 拱形(矩形 + 半圆顶),IoU 高者胜。
        v 轴向上(行号大 = 高),拱顶在高 v 侧。"""
        H, W = local.shape
        rect = np.ones((H, W), dtype=bool)
        iou_rect = local.sum() / float(H * W)
        r = W / 2.0
        if H <= r:                                   # 开口比半圆还矮:只用矩形
            return rect
        vv, hh = np.mgrid[0:H, 0:W]
        cv = H - r                                   # 圆心行(拱脚)
        ch = (W - 1) / 2.0
        arch = (vv < cv) | (((hh - ch) ** 2 + (vv - cv) ** 2) <= r * r)
        inter = (local & arch).sum()
        union = (local | arch).sum()
        iou_arch = inter / float(union) if union else 0.0
        return arch if iou_arch > iou_rect else rect
