"""stage-2 结构先验:PlaneRegularize —— 形状先验 × 观测证据的概率规整。

思想(用户提出):墙面本质是矩形平面,"多一块砖 / 少一块砖"是概率问题——
  · 矩形内部越深的位置,存在砖的先验越强 → 未观测的深处缺格按先验**补上**;
  · 矩形外部越远的位置,存在砖的先验越弱 → 远离矩形的观测残砖按后验**删掉**。
后验 = 观测 log-odds(L0 本就是概率)+ 形状先验 log-odds(±k·d,d=到矩形边界
的距离,cap 封顶)。贴边凸起(d 小,先验弱,观测强)保留;深处未知(观测 0,
先验强)补齐;远处飘砖(观测强但先验更强地否定)移除。

铁律:observed-free 是证据,先验永不覆盖(门窗/开口不补);L0 只读,
补/删都只写 overlay。对任何场景通用。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.generators.wall import WALL_LABEL, projection_peaks
from m3_adapter.vlayer.overlay import VoxelDelta


class PlaneRegularize:
    stage = 2
    depends_on = ["wall_fill"]
    default_binding = "persistent"

    def __init__(self, k_per_cell: float = 0.3, prior_cap: float = 3.0,
                 trim_fill: float = 0.3, min_wall_cells: int = 20,
                 min_height: int = 4, min_run: int = 4):
        # k_per_cell:每格距离贡献的先验 log-odds。与 ObsMap 阈值(occ_thr=0.85,
        # l_max=3.5)配合:补格需 d ≥ occ_thr/k ≈ 3 格深;删强观测格需
        # k·d > l_max - occ_thr ≈ 2.65 → d ≥ 9 格远。
        self.k = float(k_per_cell); self.cap = float(prior_cap)
        self.trim_fill = float(trim_fill)
        self.min_wall_cells = int(min_wall_cells)
        self.min_height = int(min_height); self.min_run = int(min_run)
        self.id = "plane_regularize"; self.generator = "plane_regularize"

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        lo = obs.logodds
        wall = occ & (sem == WALL_LABEL)
        if not wall.any():
            return []
        deltas = []
        emitted = set()
        xcount = wall.sum(axis=(1, 2)); zcount = wall.sum(axis=(0, 1))
        for c0 in projection_peaks(xcount, self.min_wall_cells):
            self._regularize_plane(wall[c0, :, :], free[c0, :, :], lo[c0, :, :],
                                   lambda v, h, c0=c0: (c0, v, h), deltas, emitted)
        for c0 in projection_peaks(zcount, self.min_wall_cells):
            self._regularize_plane(wall[:, :, c0].T, free[:, :, c0].T, lo[:, :, c0].T,
                                   lambda v, h, c0=c0: (h, v, c0), deltas, emitted)
        return deltas

    def _regularize_plane(self, wall2d, free2d, lo2d, to_xyz, deltas, emitted):
        if int(wall2d.sum()) < self.min_wall_cells:
            return
        vv, hh = np.nonzero(wall2d)
        v0, v1, h0, h1 = int(vv.min()), int(vv.max()), int(hh.min()), int(hh.max())
        if (v1 - v0 + 1) < self.min_height or (h1 - h0 + 1) < self.min_run:
            return
        rv0, rv1, rh0, rh1 = self._robust_rect(wall2d, v0, v1, h0, h1)
        # 形状先验:内部为 +k·(到矩形边界的最小距离+1),外部为 -k·出界距离;cap 封顶
        V, H = wall2d.shape
        iv, ih = np.mgrid[0:V, 0:H]
        d_in = np.minimum.reduce([iv - rv0, rv1 - iv, ih - rh0, rh1 - ih]) + 1
        inside = d_in > 0
        d_out = np.maximum.reduce([rv0 - iv, iv - rv1, rh0 - ih, ih - rh1])
        prior = np.where(inside, np.minimum(d_in * self.k, self.cap),
                         -np.minimum(d_out * self.k, self.cap))
        post = lo2d + prior
        # 补:矩形内、未占据、非 observed-free、后验过占据阈
        fill = inside & ~wall2d & ~free2d & (post >= 0.85)
        # 删:矩形外的墙格、后验跌破占据阈
        cut = ~inside & wall2d & (post < 0.85)
        for v, h in np.argwhere(fill):
            idx = to_xyz(int(v), int(h))
            if idx not in emitted:
                emitted.add(idx)
                deltas.append(VoxelDelta(idx, "add", WALL_LABEL,
                                         self.generator, self.default_binding))
        for v, h in np.argwhere(cut):
            idx = to_xyz(int(v), int(h))
            if idx not in emitted:
                emitted.add(idx)
                deltas.append(VoxelDelta(idx, "remove", 0,
                                         self.generator, self.default_binding))

    def _robust_rect(self, mask, v0, v1, h0, h1):
        """墙的鲁棒矩形:迭代剔除填充率 < trim_fill 的边缘行/列(甩掉毛边)。"""
        changed = True
        while changed and v1 > v0 and h1 > h0:
            changed = False
            w = h1 - h0 + 1; hgt = v1 - v0 + 1
            if mask[v0, h0:h1 + 1].sum() < self.trim_fill * w:
                v0 += 1; changed = True; continue
            if mask[v1, h0:h1 + 1].sum() < self.trim_fill * w:
                v1 -= 1; changed = True; continue
            if mask[v0:v1 + 1, h0].sum() < self.trim_fill * hgt:
                h0 += 1; changed = True; continue
            if mask[v0:v1 + 1, h1].sum() < self.trim_fill * hgt:
                h1 -= 1; changed = True
        return v0, v1, h0, h1
