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
    sem_label: np.ndarray = field(default=None)   # uint8 winning super_id per cell (0=unknown)
    sem_count: np.ndarray = field(default=None)   # uint16 winner's running vote count
    # Observed colour (lazy): running-mean RGB per cell + observation count.
    # Allocated on the first integrate_frame(point_colors=...) call so colour-
    # less maps stay lean; old .npz files without these keys load fine.
    rgb: np.ndarray = field(default=None)         # uint8 (nx,ny,nz,3) running mean
    rgb_count: np.ndarray = field(default=None)   # uint8 (nx,ny,nz), capped at 255

    def __post_init__(self):
        # Transient dirty-region state; not persisted by save/load.
        self._dirty_min = None
        self._dirty_max = None
        # Allocate semantic grids if not provided (covers new() and fresh load()).
        if self.sem_label is None:
            self.sem_label = np.zeros(self.logodds.shape, dtype=np.uint8)
        if self.sem_count is None:
            self.sem_count = np.zeros(self.logodds.shape, dtype=np.uint16)

    @classmethod
    def new(cls, shape, vmin, voxel_size):
        return cls(np.zeros(tuple(int(s) for s in shape), dtype=np.float32),
                   np.asarray(vmin, dtype=np.int64), float(voxel_size))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        obj = cls(d["logodds"].astype(np.float32), d["vmin"].astype(np.int64),
                  float(d["voxel_size"]),
                  float(d["l_hit"]), float(d["l_miss"]), float(d["l_min"]),
                  float(d["l_max"]), float(d["occ_thr"]), float(d["free_thr"]))
        # Restore semantic grids if present (back-compat: old npz files without them
        # keep the freshly-allocated zero arrays from __post_init__).
        if "sem_label" in d:
            obj.sem_label = d["sem_label"].astype(np.uint8)
            obj.sem_count = d["sem_count"].astype(np.uint16)
        if "rgb" in d:
            obj.rgb = d["rgb"].astype(np.uint8)
            obj.rgb_count = d["rgb_count"].astype(np.uint8)
        return obj

    def save(self, path):
        extra = {}
        if self.rgb is not None:
            extra["rgb"] = self.rgb
            extra["rgb_count"] = self.rgb_count
        np.savez_compressed(
            path, logodds=self.logodds, vmin=self.vmin, voxel_size=self.voxel_size,
            l_hit=self.l_hit, l_miss=self.l_miss, l_min=self.l_min, l_max=self.l_max,
            occ_thr=self.occ_thr, free_thr=self.free_thr,
            sem_label=self.sem_label, sem_count=self.sem_count, **extra)

    def integrate_frame(self, origin_m, points_m, point_labels=None, free_margin_m=0.10,
                        point_colors=None):
        """Update log-odds from one frame: traversed cells get l_miss, the
        surface endpoint cell gets l_hit, clamped to [l_min, l_max].

        Args:
            origin_m: sensor origin in metres (3,).
            points_m: hit points in metres (N, 3).
            point_labels: optional uint array (N,) of super_id labels per hit
                point.  When provided, the semantic grids are updated via a
                Boyer-Moore streaming majority-vote per cell.  Label 0 means
                unknown and is skipped.  Must be aligned with rows of points_m
                BEFORE the keep-filter (the same keep mask is applied).
            free_margin_m: rays shorter than this are discarded.
            point_colors: optional uint8 array (N, 3) of observed RGB per hit
                point (aligned like point_labels).  Updates the per-cell
                running-mean colour channel (allocated lazily).
        """
        origin_m = np.asarray(origin_m, dtype=np.float64)
        pts = np.asarray(points_m, dtype=np.float64)
        dirs = pts - origin_m[None, :]
        lens = np.linalg.norm(dirs, axis=1)
        keep = lens > free_margin_m
        dirs, lens, pts = dirs[keep], lens[keep], pts[keep]
        # Apply the same keep-filter to labels so they remain aligned with pts.
        lab = None
        if point_labels is not None:
            lab = np.asarray(point_labels)[keep].astype(np.uint8)
        col = None
        if point_colors is not None:
            col = np.asarray(point_colors)[keep].astype(np.uint8)
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
        # --- semantic update (Boyer-Moore streaming majority vote) ---
        if lab is not None:
            hit_cells = hvc[hinb]        # (M,3) in-bounds hit voxel indices
            hit_labels = lab[hinb]       # (M,) aligned labels after keep+hinb
            for (x, y, z), L in zip(hit_cells, hit_labels):
                if L == 0:
                    continue
                cur = self.sem_label[x, y, z]
                cnt = self.sem_count[x, y, z]
                if cur == L:
                    if cnt < 65535:
                        self.sem_count[x, y, z] = cnt + 1
                elif cnt == 0:
                    self.sem_label[x, y, z] = L
                    self.sem_count[x, y, z] = 1
                else:
                    self.sem_count[x, y, z] = cnt - 1   # Boyer-Moore: opposing vote
        # --- colour update (vectorised per-cell running mean) ---
        if col is not None and hinb.any():
            if self.rgb is None:
                self.rgb = np.zeros(self.logodds.shape + (3,), dtype=np.uint8)
                self.rgb_count = np.zeros(self.logodds.shape, dtype=np.uint8)
            cells = hvc[hinb]
            colors = col[hinb].astype(np.float64)
            # average this frame's hits per unique cell, then fold into the
            # running mean: mean_{n+1} = mean_n + (frame_mean - mean_n)/(n+1)
            uc, inv = np.unique(cells, axis=0, return_inverse=True)
            csum = np.zeros((len(uc), 3), dtype=np.float64)
            np.add.at(csum, inv, colors)
            cmean = csum / np.bincount(inv)[:, None]
            ix, iy, iz = uc[:, 0], uc[:, 1], uc[:, 2]
            n = self.rgb_count[ix, iy, iz].astype(np.float64)
            cur = self.rgb[ix, iy, iz].astype(np.float64)
            upd = cur + (cmean - cur) / (n + 1.0)[:, None]
            self.rgb[ix, iy, iz] = np.clip(np.rint(upd), 0, 255).astype(np.uint8)
            self.rgb_count[ix, iy, iz] = np.minimum(n + 1, 255).astype(np.uint8)
        # --- dirty-box tracking ---
        # touched_parts may hold only zero-row arrays when every traversed/
        # endpoint voxel of this frame fell outside the grid (e.g. the camera
        # pose looks out of the mapped volume). Guard on actual row count, not
        # list non-emptiness, before reducing.
        if touched_parts:
            touched = np.concatenate(touched_parts, axis=0)
            if len(touched):
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

    def semantic_grid(self):
        """uint8 super_id per cell (winning label); meaningful only where occupied."""
        return self.sem_label
