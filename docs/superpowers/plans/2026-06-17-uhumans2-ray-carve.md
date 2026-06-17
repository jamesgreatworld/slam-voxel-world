# uHumans2 Ray-Carve Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `m3_adapter/uhumans2_carve.py` — a batch ray-carving tool that reads a uHumans2 rosbag, carves "observed-free" voxels along every depth ray, and writes an `observed_free.npz` mask aligned to an existing `.vxw` grid.

**Architecture:** The tool reuses pose/depth helpers from `uhumans2_to_vxw.py` (imported directly) and grid alignment from `m3_adapter/gvd/field.py::densify_occupancy`. The core `carve_frame()` function iterates rays in chunks to bound memory, marks voxels along each ray as free, stopping `free_margin_m` before the surface. Output is a compressed NumPy archive with keys `mask`, `vmin`, `voxel_size`.

**Tech Stack:** Python 3.11, NumPy, rosbags (ROS2 reader), pixi, pytest

---

## File Structure

- **Create:** `m3_adapter/uhumans2_carve.py` — tool + `carve_frame()` + `main()`
- **Create:** `tests/test_uhumans2_carve.py` — unit tests for `carve_frame()` only (no rosbag)

---

### Task 1: Write failing tests

**Files:**
- Create: `tests/test_uhumans2_carve.py`

- [ ] **Step 1: Write both test cases (they'll fail — module doesn't exist yet)**

```python
"""Unit tests for uhumans2_carve.carve_frame — no rosbag required."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))
sys.path.insert(0, str(_PROJECT_ROOT / "m3_adapter"))

from uhumans2_carve import carve_frame  # noqa: E402


def test_single_ray_marks_interior_not_surface():
    grid = np.zeros((20, 5, 5), dtype=bool)
    vmin = np.zeros(3, dtype=np.int64)
    vs = 1.0
    # ray from (0.5,2.5,2.5) to (15.5,2.5,2.5) along +x, margin 1.0
    carve_frame(grid, vmin, vs, np.array([0.5, 2.5, 2.5]),
                np.array([[15.5, 2.5, 2.5]]), free_margin_m=1.0)
    # cells along the ray interior are free
    assert grid[3, 2, 2] and grid[10, 2, 2]
    # the surface end (x~15) within margin is NOT free
    assert not grid[15, 2, 2]
    # off-ray cells untouched
    assert not grid[3, 0, 0]


def test_out_of_bounds_safe():
    grid = np.zeros((5, 5, 5), dtype=bool)
    # ray pointing far outside the small grid: must not crash, must not mark oob
    carve_frame(grid, np.zeros(3, np.int64), 1.0, np.array([2.5, 2.5, 2.5]),
                np.array([[100.0, 2.5, 2.5]]), free_margin_m=0.5)
    assert grid[3, 2, 2]  # in-bounds part still marked
```

- [ ] **Step 2: Run test to verify it fails (ModuleNotFoundError expected)**

```
pixi run pytest tests/test_uhumans2_carve.py -q
```
Expected: `ModuleNotFoundError: No module named 'uhumans2_carve'`

---

### Task 2: Implement `carve_frame` and `main`

**Files:**
- Create: `m3_adapter/uhumans2_carve.py`

- [ ] **Step 3: Write the full module**

```python
"""uhumans2_carve — batch ray-carving tool for uHumans2 rosbag.

For each depth frame (camera origin → surface points), marks the voxels the
ray passes through as "observed-free" in a 3-D boolean mask aligned to an
existing .vxw grid. Voxels no ray entered stay unknown (False).

Usage:
  pixi run python m3_adapter/uhumans2_carve.py \\
    F:/hydra_ws/datasets/uhumans2/apartment_scene/uHumans2_apartment_s1_00h_ros2 \\
    out/uhumans2_apt.vxw \\
    --pixel-stride 4 --max-frames 500

Output: <vxw_dir>/observed_free.npz  with keys:
  mask        bool ndarray (nx, ny, nz)  — True = observed-free
  vmin        int64 (3,)                 — world-voxel coord of mask[0,0,0]
  voxel_size  float64 scalar             — metres per voxel side
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import vxw_format as vxw  # noqa: E402
from m3_adapter.common import ros_zup_to_vxw_yup  # noqa: E402
from m3_adapter.gvd.field import densify_occupancy  # noqa: E402
from m3_adapter.uhumans2_to_vxw import (  # noqa: E402
    collect_odom,
    collect_tf_static,
    unproject_depth,
    _interp_T_world_body,
    _chain_tf,
)

log = logging.getLogger("uhumans2_carve")


# ---------------------------------------------------------------------------
# Core carving function (pure numpy, no I/O)
# ---------------------------------------------------------------------------

def carve_frame(
    free_grid: np.ndarray,
    vmin: np.ndarray,
    voxel_size: float,
    origin_m: np.ndarray,
    points_m: np.ndarray,
    free_margin_m: float = 0.10,
) -> None:
    """Mark voxels along each ray origin_m->point as observed-free in free_grid.

    Args:
        free_grid: bool (nx, ny, nz), mutated in place.
        vmin:      world-voxel index of free_grid[0, 0, 0], shape (3,) int64.
        voxel_size: metres per voxel side — must match how vmin was derived.
        origin_m:  (3,) camera origin in world metres (same frame as points_m).
        points_m:  (N, 3) surface points in world metres.
        free_margin_m: stop this many metres short of each surface so the
                       surface voxel itself is not marked free.
    """
    origin_m = np.asarray(origin_m, dtype=np.float64)
    pts = np.asarray(points_m, dtype=np.float64)
    dirs = pts - origin_m[None, :]
    lens = np.linalg.norm(dirs, axis=1)
    keep = lens > free_margin_m
    dirs, lens = dirs[keep], lens[keep]
    if len(lens) == 0:
        return
    units = dirs / lens[:, None]
    max_steps = int(np.ceil(lens.max() / voxel_size)) + 1
    nx, ny, nz = free_grid.shape
    vmin = np.asarray(vmin, dtype=np.int64)

    # chunk over rays to bound memory (max_steps can be large)
    CH = 4096
    for s in range(0, len(lens), CH):
        u = units[s:s + CH]
        L = lens[s:s + CH]
        steps = np.arange(max_steps) * voxel_size            # (S,)
        valid = steps[None, :] < (L[:, None] - free_margin_m)  # (n, S)
        p = origin_m[None, None, :] + steps[None, :, None] * u[:, None, :]  # (n, S, 3)
        vc = np.floor(p / voxel_size).astype(np.int64) - vmin
        inb = (
            (vc[..., 0] >= 0) & (vc[..., 0] < nx) &
            (vc[..., 1] >= 0) & (vc[..., 1] < ny) &
            (vc[..., 2] >= 0) & (vc[..., 2] < nz)
        )
        m = valid & inb
        sel = vc[m]
        free_grid[sel[:, 0], sel[:, 1], sel[:, 2]] = True


# ---------------------------------------------------------------------------
# CLI main
# ---------------------------------------------------------------------------

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(levelname)-5s %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path,
                    help="rosbag2 directory (must contain metadata.yaml)")
    ap.add_argument("vxw_dir", type=Path,
                    help="path to .vxw directory — grid alignment source; "
                         "observed_free.npz is written here")
    ap.add_argument("--pixel-stride", type=int, default=4,
                    help="downsample depth pixels by this factor (default 4)")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="stop after this many depth frames (default: all)")
    ap.add_argument("--free-margin", type=float, default=0.10,
                    help="metres to stop short of surface (default 0.10)")
    ap.add_argument("--depth-min", type=float, default=0.2,
                    help="minimum valid depth in metres (default 0.2)")
    ap.add_argument("--depth-max", type=float, default=15.0,
                    help="maximum valid depth in metres (default 15.0)")
    args = ap.parse_args()

    if not (args.bag_dir / "metadata.yaml").is_file():
        log.error("not a rosbag2 directory: %s", args.bag_dir)
        sys.exit(1)

    # ---- load .vxw for grid alignment ----
    log.info("[1/4] loading .vxw for grid alignment: %s", args.vxw_dir)
    world = vxw.read_world(args.vxw_dir)
    occ, vmin = densify_occupancy(world, pad=1)
    voxel_size = float(world.manifest.voxel_size_meters)
    free_grid = np.zeros(occ.shape, dtype=bool)
    log.info("      grid shape=%s  vmin=%s  voxel_size=%.4fm",
             free_grid.shape, vmin.tolist(), voxel_size)

    # ---- scan bag: odom + tf_static + camera_info ----
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)

    log.info("[2/4] scanning bag for odom + tf_static + depth camera_info")
    t0 = time.perf_counter()
    with Reader(str(args.bag_dir)) as reader:
        odom_t, odom_p, odom_q, world_frame, body_frame = collect_odom(reader, ts)
        tf_static = collect_tf_static(reader, ts)
        dci_conn = [c for c in reader.connections
                    if c.topic == "/tesse/depth_cam/camera_info"][0]
        for _, _, raw in reader.messages(connections=[dci_conn]):
            dci = ts.deserialize_cdr(raw, dci_conn.msgtype)
            fx, fy = float(dci.k[0]), float(dci.k[4])
            cx, cy = float(dci.k[2]), float(dci.k[5])
            cam_frame = dci.header.frame_id
            break

    T_body_cam = _chain_tf(tf_static, body_frame, cam_frame)
    if T_body_cam is None:
        log.warning("could not resolve %s → %s via tf_static; using identity",
                    body_frame, cam_frame)
        T_body_cam = np.eye(4)

    log.info("      odom %d kfr  t=[%.3f,%.3f]  body=%s  cam=%s",
             len(odom_t), odom_t[0], odom_t[-1], body_frame, cam_frame)
    log.info("      tf_static entries: %d  T_body_cam t=%s",
             len(tf_static), T_body_cam[:3, 3].tolist())
    log.info("      depth K  fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx, fy, cx, cy)
    log.info("      bag scan done in %.2fs", time.perf_counter() - t0)

    # ---- frame loop — depth + pose only, no seg ----
    log.info("[3/4] carving depth frames")
    t0 = time.perf_counter()
    frames_used = 0
    frames_skipped = 0
    stride = args.pixel_stride

    with Reader(str(args.bag_dir)) as reader:
        dconn = [c for c in reader.connections
                 if c.topic == "/tesse/depth_cam/mono/image_raw"][0]

        for _c, _t, draw in reader.messages(connections=[dconn]):
            dm = ts.deserialize_cdr(draw, dconn.msgtype)
            if dm.encoding != "32FC1":
                log.error("unexpected depth encoding: %s", dm.encoding)
                sys.exit(2)

            depth = np.frombuffer(dm.data, dtype=np.float32).reshape(
                dm.height, dm.width)
            if stride > 1:
                depth = depth[::stride, ::stride]

            t_frame = dm.header.stamp.sec + dm.header.stamp.nanosec * 1e-9
            T_world_body = _interp_T_world_body(t_frame, odom_t, odom_p, odom_q)
            if T_world_body is None:
                frames_skipped += 1
                continue

            T_world_cam = T_world_body @ T_body_cam

            xyz_cam, _valid = unproject_depth(
                depth,
                fx / stride, fy / stride,
                cx / stride, cy / stride,
                z_min=args.depth_min, z_max=args.depth_max,
            )
            if xyz_cam.shape[0] == 0:
                frames_skipped += 1
                continue

            # transform points to world frame (homogeneous)
            xyz_h = np.concatenate(
                [xyz_cam, np.ones((xyz_cam.shape[0], 1), dtype=np.float32)], axis=1
            )
            xyz_world = (xyz_h @ T_world_cam.T.astype(np.float32))[:, :3]
            xyz_world = ros_zup_to_vxw_yup(xyz_world.astype(np.float64))

            # camera origin: T_world_cam[:3,3] also needs the same swap
            origin_ros = T_world_cam[:3, 3:4].T  # (1, 3)
            origin_vxw = ros_zup_to_vxw_yup(origin_ros)[0]  # (3,)

            carve_frame(free_grid, vmin, voxel_size, origin_vxw, xyz_world,
                        free_margin_m=args.free_margin)

            frames_used += 1
            if frames_used % 50 == 0:
                free_count = int(free_grid.sum())
                log.info("  used=%d skipped=%d  free_voxels=%d",
                         frames_used, frames_skipped, free_count)
            if args.max_frames is not None and frames_used >= args.max_frames:
                break

    log.info("      frames used=%d skipped=%d  in %.2fs",
             frames_used, frames_skipped, time.perf_counter() - t0)

    # ---- save ----
    log.info("[4/4] saving observed_free.npz")
    out_path = args.vxw_dir / "observed_free.npz"
    free_count = int(free_grid.sum())
    total_vox = int(free_grid.size)
    free_frac = free_count / total_vox if total_vox > 0 else 0.0

    np.savez_compressed(
        out_path,
        mask=free_grid,
        vmin=vmin,
        voxel_size=np.float64(voxel_size),
    )

    log.info("RESULT  frames=%d  free_voxels=%d / %d  free_fraction=%.4f  out=%s",
             frames_used, free_count, total_vox, free_frac, out_path)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the tests**

```
pixi run pytest tests/test_uhumans2_carve.py -q
```
Expected: both tests pass, `2 passed`

- [ ] **Step 5: Commit**

```bash
git add m3_adapter/uhumans2_carve.py tests/test_uhumans2_carve.py
git commit -m "feat(gvd): uhumans2 ray-carve tool -> observed_free.npz (SP-OF task 1)"
```

---

## Self-Review

**Spec coverage:**
- `carve_frame` as spec: yes, exact code from spec.
- `main()`: argparse args match spec (`bag_dir`, `vxw_dir`, `--pixel-stride`, `--max-frames`, `--free-margin`, `--depth-min`, `--depth-max`).
- Grid alignment via `densify_occupancy(world, pad=1)`: yes.
- Output keys `mask`, `vmin`, `voxel_size`: yes.
- Camera origin swap via `ros_zup_to_vxw_yup`: yes, passed as (1,3) and `[0]` extracted.
- Stats printed: frames used, free voxel count, free fraction.
- Tests: `test_single_ray_marks_interior_not_surface` and `test_out_of_bounds_safe` — both from spec.
- No seg/labels/entity code: confirmed absent.
- Commit message: matches spec exactly.

**Placeholder scan:** None found.

**Type consistency:** `carve_frame` signature consistent between plan, implementation, and tests.
