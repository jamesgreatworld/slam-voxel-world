"""stage-2 结构先验:RoofCap —— 天花板/屋面板的洞填补(闭合屋面)。

室内数据集永远观测不到屋顶外表面,但天花板的**内侧**是观测过的:每个
ceiling 层位的 2D 覆盖图是"带洞的环"。洞分两种:
  · 观测缺口(相机没扫到的死角)——被观测过的屋面包围 → 用
    binary_fill_holes 补上(这是补洞,不是外推:边界完全由证据界定);
  · 真实开口(楼梯井/挑空)——射线穿过它们,observed-free → 永不填。

只在 fill_holes 意义下工作,绝不扩张观测边界之外,与 PlaneRegularize 的
evidence-bounded 原则一致。只读 L0。对任何场景通用。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.generators.slab import _cluster_levels
from m3_adapter.vlayer.overlay import VoxelDelta

CEILING_LABEL = 4


class RoofCap:
    stage = 2
    depends_on = ["slab_fill_ceiling"]
    default_binding = "persistent"

    def __init__(self, level_gap_m: float = 0.5, band_cells: int = 2):
        self.level_gap_m = float(level_gap_m)
        self.band_cells = int(band_cells)
        self.id = "roof_cap"; self.generator = "roof_cap"

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label
        free = obs.observed_free_mask()
        vs = obs.voxel_size
        ceil_ = occ & (sem == CEILING_LABEL)
        if not ceil_.any():
            return []
        gap = max(1, int(round(self.level_gap_m / vs)))
        ys = np.argwhere(ceil_)[:, 1]
        deltas = []
        emitted = set()
        for Y in _cluster_levels(ys, gap):
            ylo = max(0, Y - self.band_cells)
            yhi = min(occ.shape[1], Y + self.band_cells + 1)
            foot = ceil_[:, ylo:yhi, :].any(axis=1)          # 该层位的 2D 覆盖
            filled = ndimage.binary_fill_holes(foot)
            holes = filled & ~foot
            for x, z in np.argwhere(holes):
                idx = (int(x), int(Y), int(z))
                if idx in emitted:
                    continue
                # 楼梯井/挑空是 observed-free,永不封;已占据的不重复写
                if free[idx] or occ[idx]:
                    continue
                emitted.add(idx)
                deltas.append(VoxelDelta(idx, "add", CEILING_LABEL,
                                         self.generator, self.default_binding))
        return deltas
