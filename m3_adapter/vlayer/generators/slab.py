"""stage-1 结构先验:水平板(floor/ceiling)补洞 + 实心化,多层逐峰。只读 L0。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.overlay import VoxelDelta


def _cluster_levels(y_indices, gap: int):
    if len(y_indices) == 0:
        return []
    bc = np.bincount(y_indices)
    u = np.unique(y_indices)
    clusters, cur = [], [int(u[0])]
    for v in u[1:]:
        if int(v) - cur[-1] > gap:
            clusters.append(cur); cur = []
        cur.append(int(v))
    clusters.append(cur)
    return [int(max(c, key=lambda y: int(bc[y]))) for c in clusters]


class SlabFill:
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, label, side, thickness_m: float = 0.6,
                 level_gap_m: float = 0.5, close_radius: int = 2):
        # thickness_m default 0.6m (~3 coarse blocks at 0.2m): fills the slab into a
        # SOLID body, not a thin sheet. observed_free 守护把向下/上填充自然停在下层
        # 房间空气处,故不会穿进相邻房间(0.15m 太薄,粗化到 0.2m 会整层消失)。
        assert side in ("floor", "ceiling")
        self.label = int(label); self.side = side
        self.thickness_m = float(thickness_m); self.level_gap_m = float(level_gap_m)
        self.close_radius = int(close_radius)
        self.id = f"slab_fill_{side}"; self.generator = "slab_fill"
        self._struct = ndimage.generate_binary_structure(2, 1)

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        vs = obs.voxel_size; ny = occ.shape[1]
        surf = occ & (sem == self.label)
        if not surf.any():
            return []
        T = max(1, int(np.ceil(self.thickness_m / vs - 1e-9)))
        gap = max(1, int(round(self.level_gap_m / vs)))
        ys = np.argwhere(surf)[:, 1]
        deltas = []
        for Y in _cluster_levels(ys, gap):
            w = max(gap, 3)
            ylo = max(0, Y - w); yhi = min(ny, Y + w + 1)
            foot = free[:, ylo:yhi, :].any(axis=1) | surf[:, max(0, Y - 1):Y + 2, :].any(axis=1)
            if self.close_radius > 0:
                foot = ndimage.binary_closing(foot, structure=self._struct, iterations=self.close_radius)
            yy_list = [Y - k for k in range(T)] if self.side == "floor" else [Y + k for k in range(T)]
            for x, z in np.argwhere(foot):
                for yy in yy_list:
                    if 0 <= yy < ny and not occ[x, yy, z] and not free[x, yy, z]:
                        deltas.append(VoxelDelta((int(x), int(yy), int(z)), "add",
                                                 self.label, self.generator, self.default_binding))
        return deltas
