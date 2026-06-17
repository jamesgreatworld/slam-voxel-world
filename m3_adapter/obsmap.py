"""obsmap.py — persistent, incrementally-mutable log-odds observation map
(OctoMap/Voxblox model). One grid holds occupied/observed-free/unknown via
per-cell log-odds; ray hits increment, ray misses (pass-through) decrement.
Supports save/load (resume) and incremental add/delete/modify. (SP-B.)"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

# OctoMap-style defaults (log-odds)
L_HIT = 0.85
L_MISS = -0.4
L_MIN = -2.0
L_MAX = 3.5
OCC_THR = 0.85      # >= -> occupied (~prob 0.7)
FREE_THR = -0.4     # <= -> observed-free


@dataclass
class ObsMap:
    logodds: np.ndarray          # float32 (nx,ny,nz)
    vmin: np.ndarray             # int64 (3,)
    voxel_size: float
    l_hit: float = L_HIT
    l_miss: float = L_MISS
    l_min: float = L_MIN
    l_max: float = L_MAX
    occ_thr: float = OCC_THR
    free_thr: float = FREE_THR

    def __post_init__(self):
        # Transient dirty-region state; not persisted by save/load.
        self._dirty_min = None
        self._dirty_max = None

    @classmethod
    def new(cls, shape, vmin, voxel_size):
        return cls(np.zeros(tuple(int(s) for s in shape), dtype=np.float32),
                   np.asarray(vmin, dtype=np.int64), float(voxel_size))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        return cls(d["logodds"].astype(np.float32), d["vmin"].astype(np.int64),
                   float(d["voxel_size"]),
                   float(d["l_hit"]), float(d["l_miss"]), float(d["l_min"]),
                   float(d["l_max"]), float(d["occ_thr"]), float(d["free_thr"]))

    def save(self, path):
        np.savez_compressed(
            path, logodds=self.logodds, vmin=self.vmin, voxel_size=self.voxel_size,
            l_hit=self.l_hit, l_miss=self.l_miss, l_min=self.l_min, l_max=self.l_max,
            occ_thr=self.occ_thr, free_thr=self.free_thr)

    def integrate_frame(self, origin_m, points_m, free_margin_m=0.10):
        """Update log-odds from one frame: traversed cells get l_miss, the
        surface endpoint cell gets l_hit, clamped to [l_min, l_max]."""
        origin_m = np.asarray(origin_m, dtype=np.float64)
        pts = np.asarray(points_m, dtype=np.float64)
        dirs = pts - origin_m[None, :]
        lens = np.linalg.norm(dirs, axis=1)
        keep = lens > free_margin_m
        dirs, lens, pts = dirs[keep], lens[keep], pts[keep]
        if len(lens) == 0:
            return
        units = dirs / lens[:, None]
        nx, ny, nz = self.logodds.shape
        vmin = self.vmin
        vs = self.voxel_size
        # --- misses: sample interior cells along each ray, accumulate counts ---
        # accumulate per-voxel MISS counts then apply once (avoid double-applying
        # within a frame). Use a dict/flat-index accumulation.
        max_steps = int(np.ceil(lens.max() / vs)) + 1
        CH = 4096
        miss_idx_list = []
        for s in range(0, len(lens), CH):
            u = units[s:s+CH]; L = lens[s:s+CH]
            steps = np.arange(max_steps) * vs
            valid = steps[None, :] < (L[:, None] - free_margin_m)
            p = origin_m[None, None, :] + steps[None, :, None] * u[:, None, :]
            vc = np.floor(p / vs).astype(np.int64) - vmin
            inb = ((vc[..., 0] >= 0) & (vc[..., 0] < nx) &
                   (vc[..., 1] >= 0) & (vc[..., 1] < ny) &
                   (vc[..., 2] >= 0) & (vc[..., 2] < nz))
            m = valid & inb
            miss_idx_list.append(vc[m])
        # --- hits: endpoint voxel of each ray ---
        hvc = np.floor(pts / vs).astype(np.int64) - vmin
        hinb = ((hvc[:, 0] >= 0) & (hvc[:, 0] < nx) &
                (hvc[:, 1] >= 0) & (hvc[:, 1] < ny) &
                (hvc[:, 2] >= 0) & (hvc[:, 2] < nz))
        hit_idx = hvc[hinb]
        # apply: one miss-step and one hit per unique cell per frame
        touched_parts = []
        if miss_idx_list:
            miss = np.concatenate(miss_idx_list, axis=0)
            miss = np.unique(miss, axis=0)
            self.logodds[miss[:, 0], miss[:, 1], miss[:, 2]] += self.l_miss
            touched_parts.append(miss)
        if len(hit_idx):
            hit = np.unique(hit_idx, axis=0)
            self.logodds[hit[:, 0], hit[:, 1], hit[:, 2]] += self.l_hit
            touched_parts.append(hit)
        np.clip(self.logodds, self.l_min, self.l_max, out=self.logodds)
        # --- dirty-box tracking ---
        if touched_parts:
            touched = np.concatenate(touched_parts, axis=0)
            t_min = touched.min(axis=0)
            t_max = touched.max(axis=0) + 1  # max is exclusive
            if self._dirty_min is None:
                self._dirty_min = t_min.copy()
                self._dirty_max = t_max.copy()
            else:
                np.minimum(self._dirty_min, t_min, out=self._dirty_min)
                np.maximum(self._dirty_max, t_max, out=self._dirty_max)

    def pop_dirty_bbox(self):
        """Return (min_idx, max_idx) int arrays covering all voxels touched since
        the last call (max_idx is exclusive), then clear the dirty state.
        Returns None if nothing has been touched since the last call."""
        if self._dirty_min is None:
            return None
        lo = self._dirty_min.copy()
        hi = self._dirty_max.copy()
        self._dirty_min = None
        self._dirty_max = None
        return lo, hi

    def occupancy_mask(self):
        return self.logodds >= self.occ_thr

    def observed_free_mask(self):
        return self.logodds <= self.free_thr
