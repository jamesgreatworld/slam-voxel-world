"""stage 1 结构先验插件:从 observed-free 反推房间足迹,在主地板高度补洞,
产出 binding=persistent 的地板体素 add 项。只读 L0。

限制(见 spec §12):当前用单一主峰地板高度 Y0;错层/多层留作后续(按峰分区)。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.overlay import VoxelDelta

FLOOR_LABEL = 3


class FloorFill:
    id = "floor_fill"
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, close_radius: int = 2):
        self.close_radius = int(close_radius)

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask()
        sem = obs.sem_label
        floor = occ & (sem == FLOOR_LABEL)
        if not floor.any():
            return []
        # 1) 主地板高度 Y0 = floor 体素 y 索引(axis 1)的众数
        ys = np.argwhere(floor)[:, 1]
        y0 = int(np.bincount(ys).argmax())
        # 2) 房间足迹:observed-free 投影到 XZ ∪ 已有 floor 的 XZ,再闭运算补缝
        free = obs.observed_free_mask()
        foot = free.any(axis=1) | floor.any(axis=1)        # (nx, nz) bool
        if self.close_radius > 0:
            st = ndimage.generate_binary_structure(2, 1)
            foot = ndimage.binary_closing(foot, structure=st,
                                          iterations=self.close_radius)
        # 3) 在足迹内、Y0 处补未占据格
        deltas = []
        for x, z in np.argwhere(foot):
            if not occ[x, y0, z]:
                deltas.append(VoxelDelta((int(x), int(y0), int(z)), "add",
                                         FLOOR_LABEL, self.id, self.default_binding))
        return deltas
