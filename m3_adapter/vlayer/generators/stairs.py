"""stage-1 结构先验:把楼梯(label 15)向下实心化成阶梯实体。
每个含楼梯体素的 (x,z) 列,从该列最高楼梯体素向下填到 max_depth(或网格底),
跳过 observed_free(开口)与已占据格。只读 L0。"""
from __future__ import annotations

import numpy as np

from m3_adapter.vlayer.overlay import VoxelDelta

STAIRS_LABEL = 15


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
        deltas = []
        # per (x,z) column, fill downward from the top stair voxel
        cols = np.argwhere(stairs.any(axis=1))      # (x,z) pairs
        for x, z in cols:
            ys = np.nonzero(stairs[x, :, z])[0]
            top = int(ys.max())
            lo = max(0, top - depth)
            for y in range(top - 1, lo - 1, -1):
                if occ[x, y, z] or free[x, y, z]:
                    break            # stop at first observed surface/opening below
                deltas.append(VoxelDelta((int(x), int(y), int(z)), "add",
                                         STAIRS_LABEL, self.generator, self.default_binding))
        return deltas
