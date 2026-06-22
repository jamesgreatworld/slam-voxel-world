"""stage-1 结构先验:封堵被占据近乎包围的"未知"针孔(传感器遮挡小孔)。
仅填 (非占据 且 非 observed_free) 且 >= min_neighbors 个面邻居被占据的格子。
绝不填 observed_free,也不会填开放房间(房间空气邻居多为非占据)。只读 L0。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.overlay import VoxelDelta


class OcclusionFill:
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, min_neighbors: int = 5, fill_label: int = 19):
        self.min_neighbors = int(min_neighbors)
        self.fill_label = int(fill_label)
        self.id = "occlusion_fill"; self.generator = "occlusion_fill"

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        if not occ.any():
            return []
        # count occupied face-neighbors via 6-connectivity convolution
        k = np.zeros((3, 3, 3), np.uint8)
        k[1, 1, 0] = k[1, 1, 2] = k[1, 0, 1] = k[1, 2, 1] = k[0, 1, 1] = k[2, 1, 1] = 1
        nb = ndimage.convolve(occ.astype(np.uint8), k, mode="constant", cval=0)
        unknown = (~occ) & (~free)
        cand = unknown & (nb >= self.min_neighbors)
        deltas = []
        for x, y, z in np.argwhere(cand):
            # sem = majority of occupied face-neighbors (fallback fill_label)
            lab = self.fill_label
            deltas.append(VoxelDelta((int(x), int(y), int(z)), "add",
                                     lab, self.generator, self.default_binding))
        return deltas
