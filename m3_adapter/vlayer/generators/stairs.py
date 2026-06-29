"""stage-1 结构先验:把楼梯(label 15)向下实心化成阶梯实体。
每个含楼梯体素的 (x,z) 列,从该列最高楼梯体素向下填,在以下任一处停止:
观测面/已占据格、observed_free(开口)、max_depth、或**地面下界**。只读 L0。

地面下界 = 场景自身 floor(label 3)观测的最低 y:地板以下是"未知"(从未观测),
既非 occupied 也非 free,若不设界,向下填充会**穿过地面溢出到地下**。该下界由数据
导出,对任何场景成立(非针对单一场景的硬编码)。"""
from __future__ import annotations

import numpy as np

from m3_adapter.vlayer.overlay import VoxelDelta

STAIRS_LABEL = 15
FLOOR_LABEL = 3


class StairsFill:
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, max_depth_m: float = 1.5):
        self.max_depth_m = float(max_depth_m)
        self.id = "stairs_fill"; self.generator = "stairs_fill"

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        vs = obs.voxel_size; ny = occ.shape[1]
        stairs = occ & (sem == STAIRS_LABEL)
        if not stairs.any():
            return []
        depth = max(1, int(round(self.max_depth_m / vs)))
        # ground floor = lowest floor observation; never solidify below it.
        floor_ys = np.argwhere(occ & (sem == FLOOR_LABEL))[:, 1]
        ground_y = int(floor_ys.min()) if floor_ys.size else 0
        deltas = []
        # per (x,z) column, fill downward from the top stair voxel
        cols = np.argwhere(stairs.any(axis=1))      # (x,z) pairs
        for x, z in cols:
            ys = np.nonzero(stairs[x, :, z])[0]
            top = int(ys.max())
            lo = max(ground_y, top - depth)
            for y in range(top - 1, lo - 1, -1):
                if occ[x, y, z] or free[x, y, z]:
                    break            # stop at first observed surface/opening below
                deltas.append(VoxelDelta((int(x), int(y), int(z)), "add",
                                         STAIRS_LABEL, self.generator, self.default_binding))
        return deltas
