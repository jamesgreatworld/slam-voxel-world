"""stage-1 结构先验:曼哈顿竖直墙(label=19)面内补小洞 + 沿法向增厚成实心墙。
门/窗/通道 = observed_free,永不填。只读 L0。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.overlay import VoxelDelta

WALL_LABEL = 19


class WallFill:
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, thickness_m: float = 0.10, min_wall_cells: int = 20, close_radius: int = 2):
        self.thickness_m = float(thickness_m); self.min_wall_cells = int(min_wall_cells)
        self.close_radius = int(close_radius)
        self.id = "wall_fill"; self.generator = "wall_fill"
        self._struct = ndimage.generate_binary_structure(2, 1)

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        vs = obs.voxel_size; nx, ny, nz = occ.shape
        wall = occ & (sem == WALL_LABEL)
        if not wall.any():
            return []
        T = max(1, int(np.ceil(self.thickness_m / vs - 1e-9)))
        deltas = []

        def _do_slice(plane2d, to_world):
            # plane2d bool (a,b); to_world(A,B,d)->(x,y,z). Close small holes in-plane,
            # then for every plane cell extrude T voxels along the slice's constant axis.
            if int(plane2d.sum()) < self.min_wall_cells:
                return
            aa, bb = np.nonzero(plane2d)
            a0, a1, b0, b1 = aa.min(), aa.max(), bb.min(), bb.max()
            sub = plane2d[a0:a1 + 1, b0:b1 + 1]
            filled = ndimage.binary_closing(sub, structure=self._struct, iterations=self.close_radius)
            fa, fb = np.nonzero(filled)
            for ia, ib in zip(fa, fb):
                A = a0 + int(ia); B = b0 + int(ib)
                for d in range(T):
                    x, y, z = to_world(A, B, d)
                    if not (0 <= x < nx and 0 <= y < ny and 0 <= z < nz):
                        continue
                    if occ[x, y, z] or free[x, y, z]:
                        continue
                    deltas.append(VoxelDelta((int(x), int(y), int(z)), "add",
                                             WALL_LABEL, self.generator, self.default_binding))

        # walls running along Z: fixed x, plane in (y,z), thicken along +x
        for x in range(nx):
            _do_slice(wall[x, :, :], lambda A, B, d, x=x: (x + d, A, B))
        # walls running along X: fixed z, plane in (x,y), thicken along +z
        for z in range(nz):
            _do_slice(wall[:, :, z], lambda A, B, d, z=z: (A, B, z + d))
        return deltas
