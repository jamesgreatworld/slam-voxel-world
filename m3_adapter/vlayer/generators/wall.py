"""stage-1 结构先验:把墙(label=19)当作"平整、竖直、轴对齐、大延展的平面"识别——
将墙体素投影到 X / Z 轴,峰即一面墙(法向沿该轴),天然按朝向认领一次(去重)。
每面:面内闭运算补小洞 + 朝墙背(非 observed_free 侧)增厚成实心墙。门窗 free 永不填。只读 L0。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.overlay import VoxelDelta

WALL_LABEL = 19


class WallFill:
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, thickness_m: float = 0.10, min_wall_cells: int = 20,
                 min_height: int = 4, min_run: int = 4, close_radius: int = 2):
        self.thickness_m = float(thickness_m); self.min_wall_cells = int(min_wall_cells)
        self.min_height = int(min_height); self.min_run = int(min_run)
        self.close_radius = int(close_radius)
        self.id = "wall_fill"; self.generator = "wall_fill"
        self._struct = ndimage.generate_binary_structure(2, 1)

    def _peaks(self, count):
        """坐标轴投影计数里的局部峰(>=min_wall_cells),仅合并紧邻(<=1)取最大。
        合并窗取 1:厚墙(连续索引)并成一峰,但相距 2 格的两面平行墙各自保留。"""
        n = len(count); peaks = []
        for i in range(n):
            if count[i] < self.min_wall_cells:
                continue
            lo = max(0, i - 1); hi = min(n, i + 2)
            if count[i] < count[lo:hi].max():
                continue
            if peaks and i - peaks[-1] <= 1:
                if count[i] > count[peaks[-1]]:
                    peaks[-1] = i
                continue
            peaks.append(i)
        return peaks

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        vs = obs.voxel_size; nx, ny, nz = occ.shape
        wall = occ & (sem == WALL_LABEL)
        if not wall.any():
            return []
        T = max(1, int(np.ceil(self.thickness_m / vs - 1e-9)))
        deltas = []
        xcount = wall.sum(axis=(1, 2)); zcount = wall.sum(axis=(0, 1))
        for x0 in self._peaks(xcount):
            self._fill_plane(wall[x0, :, :], free, occ, (nx, ny, nz), T, deltas,
                             "x", x0,
                             free[x0 - 1, :, :] if x0 - 1 >= 0 else None,
                             free[x0 + 1, :, :] if x0 + 1 < nx else None)
        for z0 in self._peaks(zcount):
            self._fill_plane(wall[:, :, z0], free, occ, (nx, ny, nz), T, deltas,
                             "z", z0,
                             free[:, :, z0 - 1] if z0 - 1 >= 0 else None,
                             free[:, :, z0 + 1] if z0 + 1 < nz else None)
        return deltas

    def _fill_plane(self, plane, free, occ, dims, T, deltas, const_axis, c0, free_lo, free_hi):
        nx, ny, nz = dims
        if int(plane.sum()) < self.min_wall_cells:
            return
        aa, bb = np.nonzero(plane)
        a0, a1, b0, b1 = int(aa.min()), int(aa.max()), int(bb.min()), int(bb.max())
        # structural prior: tall (height in y) + extended (run in the other horizontal axis)
        if const_axis == "x":            # plane axes a=y, b=z
            height = a1 - a0 + 1; run = b1 - b0 + 1
        else:                            # plane axes a=x, b=y
            height = b1 - b0 + 1; run = a1 - a0 + 1
        if height < self.min_height or run < self.min_run:
            return
        sub = plane[a0:a1 + 1, b0:b1 + 1]
        filled = ndimage.binary_closing(sub, structure=self._struct, iterations=self.close_radius)
        # thicken toward the BACK = away from the side with more observed_free (the room)
        flo = int(free_lo[plane].sum()) if free_lo is not None else 0
        fhi = int(free_hi[plane].sum()) if free_hi is not None else 0
        back = -1 if fhi > flo else 1    # +side is room -> back is -; tie -> +1
        fa, fb = np.nonzero(sub | filled)
        for ia, ib in zip(fa, fb):
            A = a0 + int(ia); B = b0 + int(ib)
            for k in range(T):
                c = c0 + k * back
                if const_axis == "x":
                    x, y, z = c, A, B
                else:
                    x, y, z = A, B, c
                if not (0 <= x < nx and 0 <= y < ny and 0 <= z < nz):
                    continue
                if occ[x, y, z] or free[x, y, z]:
                    continue
                deltas.append(VoxelDelta((int(x), int(y), int(z)), "add",
                                         WALL_LABEL, self.generator, self.default_binding))
