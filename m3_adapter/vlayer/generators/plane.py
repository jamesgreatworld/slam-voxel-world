"""stage-1 结构先验:RANSAC 拟合任意朝向平面(斜顶/斜墙),沿真平面栅格化补洞。
只读 L0;observed_free(开口)永不填。"""
from __future__ import annotations

import numpy as np

from m3_adapter.vlayer.overlay import VoxelDelta


class RansacPlaneFill:
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, labels=(4,), dist_thresh: float = 1.5, min_inliers: int = 80,
                 max_planes: int = 4, iters: int = 200, seed: int = 0):
        self.labels = tuple(int(l) for l in labels)
        self.dist_thresh = float(dist_thresh)
        self.min_inliers = int(min_inliers)
        self.max_planes = int(max_planes)
        self.iters = int(iters)
        self.seed = int(seed)
        self.id = "ransac_plane"; self.generator = "ransac_plane"

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        nx, ny, nz = occ.shape
        mask = occ & np.isin(sem, np.asarray(self.labels))
        pts = np.argwhere(mask).astype(np.float64)
        if len(pts) < self.min_inliers:
            return []
        rng = np.random.default_rng(self.seed)
        remaining = pts
        deltas = []
        fill_label = self.labels[0]
        for _p in range(self.max_planes):
            if len(remaining) < self.min_inliers:
                break
            best_n = None; best_d = 0.0; best_cnt = 0
            for _i in range(self.iters):
                tri = remaining[rng.choice(len(remaining), 3, replace=False)]
                n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                nn = np.linalg.norm(n)
                if nn < 1e-6:
                    continue
                n = n / nn; d = float(n @ tri[0])
                cnt = int((np.abs(remaining @ n - d) < self.dist_thresh).sum())
                if cnt > best_cnt:
                    best_cnt = cnt; best_n = n; best_d = d
            if best_n is None or best_cnt < self.min_inliers:
                break
            inl_sel = np.abs(remaining @ best_n - best_d) < self.dist_thresh
            inl = remaining[inl_sel]
            # rasterize: solve for the axis most aligned with the normal over the
            # inlier footprint in the other two axes
            ax = int(np.argmax(np.abs(best_n)))
            o0, o1 = [a for a in range(3) if a != ax]
            lo = inl.min(axis=0).astype(int); hi = inl.max(axis=0).astype(int)
            for u in range(lo[o0], hi[o0] + 1):
                for v in range(lo[o1], hi[o1] + 1):
                    w = (best_d - best_n[o0] * u - best_n[o1] * v) / best_n[ax]
                    c = [0, 0, 0]; c[o0] = u; c[o1] = v; c[ax] = int(round(w))
                    x, y, z = c
                    if 0 <= x < nx and 0 <= y < ny and 0 <= z < nz \
                       and not occ[x, y, z] and not free[x, y, z]:
                        deltas.append(VoxelDelta((x, y, z), "add", fill_label,
                                                 self.generator, self.default_binding))
            remaining = remaining[~inl_sel]
        return deltas
