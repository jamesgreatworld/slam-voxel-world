# SP-A 批式 3D GVD Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在占据体素 `.vxw` 上,用种子洪泛界定室内自由空间 + scipy EDT(距离场+parent)+ parent 分叉提取 3D GVD 骨架,输出"原几何 + 发光骨架"叠加 `.vxw`,用现有 Godot 查看器验证。

**Architecture:** 两文件,严格按 `docs/architecture.md §11` adapter 约定。`m3_adapter/voxel_gvd.py` = 纯算法(numpy 进/出,无 IO,SP-B 增量化可直接复用);`m3_adapter/gvd_to_vxw.py` = CLI 驱动(读 .vxw → 稠密化 → 种子 → 调算法 → 写叠加 .vxw + 统计)。ESDF/parent 全程瞬态,核心 `.vxw` 格式零改动(见 `docs/vision.md §4`)。

**Tech Stack:** Python 3.11, numpy, scipy.ndimage(`label` 做 6 连通洪泛、`distance_transform_edt(return_indices=True)` 一次拿距离场+最近障碍索引),vxw_format 读写。

**已核实的代码事实(实现时据此,勿臆测):**
- `vxw.read_world(path) -> World`;`World.manifest`(`.voxel_size_meters` / `.chunk_extent` / `.bounds_chunks_min` / `.bounds_chunks_max` / `.spawn_hint` 可选 `[x,y,z,yaw]`)、`World.palette`、`World.chunks`(dict,键=chunk 坐标 tuple)。
- `Chunk.voxels`:结构化 numpy 数组 `(extent,extent,extent)`,字段 `material_id/semantic_id/state/color_palette_idx`(均 u1);`VOXEL_DTYPE` 在 `vxw_format`。
- `vxw.Material(id, name, color_rgb, flags, emission_rgb=(0,0,0), emission_energy=0.0, ...)`——发光材质设 `emission_rgb` + `emission_energy>0`。
- `vxw.Palette` 是可变 dataclass:`.materials`(list,≤256)、`.color_lut`(list of RGB,≤256)、`.semantic_classes`;直接 append 即可。
- `vxw.Chunk(coord, voxels, encoding=Encoding.RLE, compression=Compression.GZIP)`;`vxw.write_world(path, world)` 落盘并清旧 chunk。
- chunk 坐标 = `floor_divide(world_voxel, extent)`,local = `world_voxel - chunk*extent`。
- 占据判据:`voxels["material_id"] != 0`(air=0)。
- scipy 1.17.1 已在 pixi 环境(经 scikit-learn 间接安装),`import scipy` 可用;本计划仍显式声明为直接依赖。

**算法澄清(重要,实现据此):** ESDF 必须对**真实障碍集 `occupied`** 算距离,不是对 `~free`。否则墙洞处会把"未洪泛到的外部自由空间"误当障碍,污染距离场/parent。所以 `compute_esdf(occupied, ...)` 在全网格(`~occupied`)上算"到最近障碍的距离+最近障碍索引",`extract_gvd` 再用 `free` 掩码限定只在洪泛到的室内取 GVD。

---

### Task 1: voxel_gvd.flood_free_space + scipy 依赖

**Files:**
- Create: `m3_adapter/voxel_gvd.py`
- Modify: `pixi.toml`(`[dependencies]` 加 scipy)
- Test: `tests/test_gvd.py`

- [ ] **Step 1: pixi.toml 显式声明 scipy**

在 `[dependencies]` 段(numpy 那几行附近)加一行:
```toml
scipy = ">=1.11,<2"
```

- [ ] **Step 2: 验证 scipy 可导入(已装,无需重解析)**

Run: `pixi run python -c "import scipy; from scipy import ndimage; print(scipy.__version__)"`
Expected: 打印 `1.17.1`(或 ≥1.11 的版本),无 ImportError。

- [ ] **Step 3: 写失败测试**

创建 `tests/test_gvd.py`:
```python
import numpy as np
import pytest
from m3_adapter.voxel_gvd import flood_free_space


def _hollow_box(inner=19):
    """Solid shell cube. Returns (occupied_bool, interior_center_idx).
    Grid is (inner+2)^3; walls on every outer face; interior 1..inner free."""
    n = inner + 2
    occ = np.zeros((n, n, n), dtype=bool)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    center = (n // 2, n // 2, n // 2)
    return occ, center


def test_flood_fills_box_interior():
    occ, center = _hollow_box(inner=19)
    free = flood_free_space(occ, center)
    # interior is 19^3 free cells, all reachable from centre
    assert free.sum() == 19 ** 3
    assert free[center]
    # walls are never free
    assert not free[0, 0, 0]


def test_flood_seed_in_obstacle_raises():
    occ, _ = _hollow_box(inner=19)
    with pytest.raises(ValueError):
        flood_free_space(occ, (0, 0, 0))


def test_flood_does_not_leak_through_sealed_wall():
    # two 5^3 rooms side by side sharing a solid wall -> seed in room A
    # must NOT reach room B.
    occ = np.zeros((13, 7, 7), dtype=bool)
    occ[0, :, :] = occ[-1, :, :] = True
    occ[:, 0, :] = occ[:, -1, :] = True
    occ[:, :, 0] = occ[:, :, -1] = True
    occ[6, :, :] = True          # solid dividing wall at x=6
    free = flood_free_space(occ, (3, 3, 3))   # room A (x=1..5)
    assert free[3, 3, 3]
    assert not free[9, 3, 3]     # room B (x=7..11) unreachable
    # a 1-voxel hole in the divider DOES leak (demonstrates the risk)
    occ[6, 3, 3] = False
    free2 = flood_free_space(occ, (3, 3, 3))
    assert free2[9, 3, 3]
```

- [ ] **Step 4: 运行测试,确认失败**

Run: `pixi run pytest tests/test_gvd.py -q`
Expected: FAIL / ImportError(`voxel_gvd` 还没有 `flood_free_space`)。

- [ ] **Step 5: 写实现**

创建 `m3_adapter/voxel_gvd.py`:
```python
"""voxel_gvd.py — pure-numpy 3D GVD primitives (SP-A).

No file I/O, no vxw_format dependency: arrays in, arrays out, so the same
functions serve the batch adapter (gvd_to_vxw.py) and a future incremental
mapper (SP-B). See docs/superpowers/specs/2026-06-13-spa-batch-gvd-design.md.

Pipeline: flood_free_space -> compute_esdf -> extract_gvd.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage


def flood_free_space(occupied: np.ndarray, seed: tuple[int, int, int]) -> np.ndarray:
    """6-connected free-space region containing `seed`.

    Args:
        occupied: bool array (nx,ny,nz); True = obstacle.
        seed: index into the array; must be a free (non-obstacle) cell.

    Returns:
        bool array, True where free space is 6-connected to `seed`.

    Raises:
        ValueError: if `seed` is inside an obstacle.
    """
    free = ~occupied
    if not free[tuple(seed)]:
        raise ValueError(f"seed {seed} is inside an obstacle")
    structure = ndimage.generate_binary_structure(3, 1)  # 6-connectivity
    labels, _ = ndimage.label(free, structure=structure)
    return labels == labels[tuple(seed)]
```

- [ ] **Step 6: 运行测试,确认通过**

Run: `pixi run pytest tests/test_gvd.py -q`
Expected: 3 passed。

- [ ] **Step 7: Commit**

```bash
git add m3_adapter/voxel_gvd.py tests/test_gvd.py pixi.toml
git commit -m "feat(gvd): flood_free_space + scipy dep (SP-A task 1)"
```

---

### Task 2: voxel_gvd.compute_esdf

**Files:**
- Modify: `m3_adapter/voxel_gvd.py`
- Test: `tests/test_gvd.py`

- [ ] **Step 1: 写失败测试(追加到 tests/test_gvd.py)**

```python
from m3_adapter.voxel_gvd import compute_esdf


def test_esdf_box_center_distance():
    occ, center = _hollow_box(inner=21)   # interior 1..21, center at 11
    dist_m, parent = compute_esdf(occ, voxel_size=0.5)
    # centre is 11 voxels from the nearest wall (index 0 or 22) -> 11 * 0.5 m
    assert dist_m[center] == pytest.approx(11 * 0.5, abs=1e-6)
    # parent is the (3, nx,ny,nz) index of the nearest obstacle voxel
    assert parent.shape == (3,) + occ.shape
    pcoord = parent[:, center[0], center[1], center[2]]
    # nearest obstacle must actually BE an obstacle
    assert occ[pcoord[0], pcoord[1], pcoord[2]]
```

- [ ] **Step 2: 运行,确认失败**

Run: `pixi run pytest tests/test_gvd.py::test_esdf_box_center_distance -q`
Expected: FAIL(`compute_esdf` 未定义)。

- [ ] **Step 3: 写实现(追加到 m3_adapter/voxel_gvd.py)**

```python
def compute_esdf(
    occupied: np.ndarray, voxel_size: float
) -> tuple[np.ndarray, np.ndarray]:
    """Euclidean distance field to the nearest obstacle, + the obstacle index.

    Computed over the WHOLE grid against the true obstacle set `occupied`
    (not against the flooded free set — see plan's algorithm note). The
    caller restricts to flooded free space in extract_gvd.

    Args:
        occupied: bool array (nx,ny,nz); True = obstacle.
        voxel_size: metres per voxel.

    Returns:
        (dist_m, parent):
            dist_m: float array (nx,ny,nz), metres to nearest obstacle.
            parent: int array (3, nx,ny,nz), index of that nearest obstacle
                    voxel ("basis point" in Hydra terms).
    """
    dist_vox, parent = ndimage.distance_transform_edt(
        ~occupied, return_indices=True
    )
    return dist_vox * voxel_size, parent
```

- [ ] **Step 4: 运行,确认通过**

Run: `pixi run pytest tests/test_gvd.py -q`
Expected: 4 passed。

- [ ] **Step 5: Commit**

```bash
git add m3_adapter/voxel_gvd.py tests/test_gvd.py
git commit -m "feat(gvd): compute_esdf distance+parent (SP-A task 2)"
```

---

### Task 3: voxel_gvd.extract_gvd

**Files:**
- Modify: `m3_adapter/voxel_gvd.py`
- Test: `tests/test_gvd.py`

- [ ] **Step 1: 写失败测试(追加)**

```python
from m3_adapter.voxel_gvd import extract_gvd


def test_gvd_is_bisector_between_two_obstacles():
    # two point obstacles along x; GVD = perpendicular bisector plane x=10
    n = 21
    occ = np.zeros((n, n, n), dtype=bool)
    occ[2, 10, 10] = True
    occ[18, 10, 10] = True
    free = ~occ
    dist_m, parent = compute_esdf(occ, voxel_size=1.0)
    gvd = extract_gvd(free, dist_m, parent, voxel_size=1.0, d_min=1.0, theta_sep=2.0)
    # midplane x=10 (equidistant to both) contains GVD voxels
    assert gvd[10].any()
    # a cell clearly nearer obstacle A is not on the GVD
    assert not gvd[4, 10, 10]


def test_gvd_respects_free_mask_and_dmin():
    occ, center = _hollow_box(inner=21)
    free = flood_free_space(occ, center)
    dist_m, parent = compute_esdf(occ, voxel_size=0.5)
    gvd = extract_gvd(free, dist_m, parent, voxel_size=0.5, d_min=0.20, theta_sep=0.40)
    # every GVD voxel is free and at least d_min from any obstacle
    assert gvd[~free].sum() == 0
    assert (dist_m[gvd] >= 0.20 - 1e-9).all()
    # the medial axis of a box room is non-empty
    assert gvd.any()
```

- [ ] **Step 2: 运行,确认失败**

Run: `pixi run pytest tests/test_gvd.py -k gvd_is_bisector -q`
Expected: FAIL(`extract_gvd` 未定义)。

- [ ] **Step 3: 写实现(追加)**

```python
def extract_gvd(
    free: np.ndarray,
    dist_m: np.ndarray,
    parent: np.ndarray,
    voxel_size: float,
    d_min: float = 0.20,
    theta_sep: float = 0.40,
) -> np.ndarray:
    """GVD voxels = free cells equidistant to >=2 distinct obstacles.

    A free candidate cell (dist >= d_min) is on the GVD if some 6-neighbour
    (also free) has a nearest-obstacle parent at least `theta_sep` metres away
    from this cell's parent — i.e. two distance wavefronts collide here. This
    is the Hydra GVD definition; parent SPACING replaces Hydra's parent-vector
    angle (simpler, more robust to voxelisation jaggies).

    Vectorised over the 3 axes via paired lo/hi slices (no np.roll wrap-around).

    Args:
        free: bool array (nx,ny,nz), True = flooded interior free space.
        dist_m: metres to nearest obstacle (compute_esdf).
        parent: (3,nx,ny,nz) nearest-obstacle index (compute_esdf).
        voxel_size: metres per voxel.
        d_min: drop near-surface noise ridges below this clearance (m).
        theta_sep: min parent spacing to count as "different obstacles" (m).

    Returns:
        bool array (nx,ny,nz), True = GVD skeleton voxel.
    """
    px = parent[0].astype(np.float64)
    py = parent[1].astype(np.float64)
    pz = parent[2].astype(np.float64)
    cand = free & (dist_m >= d_min)
    theta_vox2 = (theta_sep / voxel_size) ** 2
    gvd = np.zeros(free.shape, dtype=bool)
    for axis in range(3):
        lo = [slice(None)] * 3
        hi = [slice(None)] * 3
        lo[axis] = slice(0, -1)
        hi[axis] = slice(1, None)
        lo = tuple(lo)
        hi = tuple(hi)
        d2 = (px[lo] - px[hi]) ** 2 + (py[lo] - py[hi]) ** 2 + (pz[lo] - pz[hi]) ** 2
        both_free = free[lo] & free[hi]
        sep = both_free & (d2 >= theta_vox2)
        gvd[lo] |= cand[lo] & sep
        gvd[hi] |= cand[hi] & sep
    return gvd
```

- [ ] **Step 4: 运行,确认通过**

Run: `pixi run pytest tests/test_gvd.py -q`
Expected: 6 passed。

- [ ] **Step 5: Commit**

```bash
git add m3_adapter/voxel_gvd.py tests/test_gvd.py
git commit -m "feat(gvd): extract_gvd parent-divergence skeleton (SP-A task 3)"
```

---

### Task 4: gvd_to_vxw IO 层(densify / resolve_seed / overlay)

**Files:**
- Create: `m3_adapter/gvd_to_vxw.py`
- Test: `tests/test_gvd.py`

- [ ] **Step 1: 写失败的集成测试(追加)**

```python
import vxw_format as vxw
from m3_adapter.common import build_concrete_palette
from m3_adapter.gvd_to_vxw import densify_occupancy, resolve_seed, overlay_gvd_into_world


def _make_box_vxw(tmp_path, extent=32, vsize=0.5):
    """A hollow box room in chunk (0,0,0): walls at the 0 and 20 faces."""
    pal = build_concrete_palette()
    arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
    occ = np.zeros((21, 21, 21), dtype=bool)
    occ[0] = occ[20] = True
    occ[:, 0] = occ[:, 20] = True
    occ[:, :, 0] = occ[:, :, 20] = True
    ii = np.argwhere(occ)
    arr["material_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    arr["semantic_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    chunks = {(0, 0, 0): vxw.Chunk(coord=(0, 0, 0), voxels=arr)}
    man = vxw.Manifest(
        world_id="test", voxel_size_meters=vsize, chunk_extent=extent,
        bounds_chunks_min=(0, 0, 0), bounds_chunks_max=(1, 1, 1),
    )
    w = vxw.World(manifest=man, palette=pal, chunks=chunks)
    p = tmp_path / "box.vxw"
    vxw.write_world(p, w)
    return p


def test_densify_and_overlay_roundtrip(tmp_path):
    p = _make_box_vxw(tmp_path)
    world = vxw.read_world(p)
    occ, vmin = densify_occupancy(world, pad=1)
    # box walls span world-voxels 0..20 -> with pad=1, vmin = (-1,-1,-1)
    assert tuple(int(x) for x in vmin) == (-1, -1, -1)
    assert occ.dtype == bool and occ.any()

    seed = resolve_seed(occ, vmin, voxel_size=0.5)  # auto: deepest interior point
    assert not occ[seed]

    free = flood_free_space(occ, seed)
    dist_m, parent = compute_esdf(occ, 0.5)
    gvd = extract_gvd(free, dist_m, parent, 0.5, d_min=0.20, theta_sep=0.40)
    assert gvd.any()

    n_mat_before = len(world.palette.materials)
    overlay_gvd_into_world(world, gvd, vmin)
    # a new glowing material was appended
    assert len(world.palette.materials) == n_mat_before + 1
    gvd_mat = world.palette.materials[-1]
    assert gvd_mat.emission_energy > 0
    # the overlaid world round-trips and the gvd material id appears in chunks
    out = tmp_path / "box_gvd.vxw"
    vxw.write_world(out, world)
    w2 = vxw.read_world(out)
    found = any(
        (ch.voxels["material_id"] == gvd_mat.id).any() for ch in w2.chunks.values()
    )
    assert found
```

- [ ] **Step 2: 运行,确认失败**

Run: `pixi run pytest tests/test_gvd.py::test_densify_and_overlay_roundtrip -q`
Expected: FAIL(`gvd_to_vxw` 模块/函数未定义)。

- [ ] **Step 3: 写实现**

创建 `m3_adapter/gvd_to_vxw.py`(本 task 只写三个 IO 辅助 + 导入;CLI main 在 Task 5):
```python
"""gvd_to_vxw.py — batch 3D GVD adapter (SP-A).

Reads a .vxw, densifies occupancy over its tight bbox, floods interior free
space from a seed, computes ESDF + GVD (m3_adapter.voxel_gvd), and writes a
new .vxw = original geometry + GVD skeleton voxels in a glowing material, so
the existing Godot viewer shows the skeleton with zero viewer changes.

ESDF/parent are transient (numpy only); the core .vxw format is untouched
(see docs/vision.md §4). CLI entry: see main().
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

import vxw_format as vxw
from m3_adapter.voxel_gvd import flood_free_space, compute_esdf, extract_gvd

GVD_MATERIAL_NAME = "gvd_skeleton"
GVD_COLOR = (0, 255, 255)  # cyan
GVD_EMISSION_ENERGY = 3.0


def densify_occupancy(world, pad: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Dense bool occupancy grid over the world's tight bbox, with `pad`
    voxels of free margin on every side (so flood can surround geometry).

    Returns (occupied_bool (nx,ny,nz), vmin (3,) world-voxel coord of index 0).
    """
    extent = world.manifest.chunk_extent
    parts = []
    for ccoord, chunk in world.chunks.items():
        v = chunk.voxels
        mask = v["material_id"] != 0
        if not mask.any():
            continue
        idx = np.argwhere(mask)
        base = np.array(ccoord, dtype=np.int64) * extent
        parts.append(idx + base)
    if not parts:
        raise ValueError("world has no occupied voxels")
    allc = np.concatenate(parts, axis=0)
    vmin = allc.min(axis=0) - pad
    vmax = allc.max(axis=0) + pad
    dims = (vmax - vmin + 1).astype(int)
    occ = np.zeros(tuple(int(d) for d in dims), dtype=bool)
    rel = allc - vmin
    occ[rel[:, 0], rel[:, 1], rel[:, 2]] = True
    return occ, vmin


def _check_free(occupied: np.ndarray, idx: tuple, source: str) -> None:
    dims = occupied.shape
    if any(i < 0 or i >= d for i, d in zip(idx, dims)):
        raise ValueError(f"{source} seed {idx} is outside the world bbox {dims}")
    if occupied[idx]:
        raise ValueError(f"{source} seed {idx} is inside an obstacle")


def resolve_seed(
    occupied: np.ndarray,
    vmin: np.ndarray,
    voxel_size: float,
    seed_metres=None,
    spawn_hint=None,
) -> tuple[int, int, int]:
    """Pick the flood seed as a dense index. Priority: explicit --seed (metres)
    > manifest spawn_hint (metres) > auto (deepest interior point).
    """
    if seed_metres is not None:
        wv = np.round(np.asarray(seed_metres, dtype=float) / voxel_size).astype(int)
        idx = tuple(int(x) for x in (wv - vmin))
        _check_free(occupied, idx, "explicit --seed")
        return idx
    if spawn_hint is not None and len(spawn_hint) >= 3:
        wv = np.round(np.asarray(spawn_hint[:3], dtype=float) / voxel_size).astype(int)
        idx = tuple(int(x) for x in (wv - vmin))
        _check_free(occupied, idx, "spawn_hint")
        return idx
    # auto: the non-obstacle cell furthest from any obstacle = likely room centre
    dist = ndimage.distance_transform_edt(~occupied)
    idx = tuple(int(x) for x in np.unravel_index(int(np.argmax(dist)), dist.shape))
    return idx


def overlay_gvd_into_world(world, gvd: np.ndarray, vmin: np.ndarray) -> None:
    """Append a glowing GVD material to the palette and write each GVD voxel
    into the world's chunks (creating chunks for room interiors that had no
    geometry). Recomputes manifest bounds_chunks to cover any new chunks.
    Mutates `world` in place.
    """
    extent = world.manifest.chunk_extent
    pal = world.palette
    new_mat_id = max(m.id for m in pal.materials) + 1
    if new_mat_id > 255:
        raise ValueError("palette is full (256 materials); cannot add GVD material")
    new_color_idx = len(pal.color_lut)
    pal.materials.append(
        vxw.Material(
            id=new_mat_id,
            name=GVD_MATERIAL_NAME,
            color_rgb=GVD_COLOR,
            flags=("gvd", "emit"),
            emission_rgb=GVD_COLOR,
            emission_energy=GVD_EMISSION_ENERGY,
        )
    )
    pal.color_lut.append(GVD_COLOR)

    gvd_idx = np.argwhere(gvd)
    if len(gvd_idx) == 0:
        return
    world_v = gvd_idx + vmin
    cc = np.floor_divide(world_v, extent)
    local = (world_v - cc * extent).astype(np.uint8)
    for ck in np.unique(cc, axis=0):
        m = np.all(cc == ck, axis=1)
        ckey = tuple(int(x) for x in ck)
        if ckey in world.chunks:
            arr = world.chunks[ckey].voxels
        else:
            arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
            world.chunks[ckey] = vxw.Chunk(
                coord=ckey, voxels=arr,
                encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP,
            )
        loc = local[m]
        arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = new_mat_id
        arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 0
        arr["color_palette_idx"][loc[:, 0], loc[:, 1], loc[:, 2]] = new_color_idx

    keys = np.array(list(world.chunks.keys()))
    world.manifest.bounds_chunks_min = tuple(int(x) for x in keys.min(axis=0))
    world.manifest.bounds_chunks_max = tuple(int(x) + 1 for x in keys.max(axis=0))
```

- [ ] **Step 4: 运行,确认通过**

Run: `pixi run pytest tests/test_gvd.py -q`
Expected: 7 passed。

- [ ] **Step 5: Commit**

```bash
git add m3_adapter/gvd_to_vxw.py tests/test_gvd.py
git commit -m "feat(gvd): gvd_to_vxw IO layer densify/seed/overlay (SP-A task 4)"
```

---

### Task 5: gvd_to_vxw.main CLI + 端到端

**Files:**
- Modify: `m3_adapter/gvd_to_vxw.py`
- Test: `tests/test_gvd.py`

- [ ] **Step 1: 写失败的端到端测试(追加)**

```python
def test_cli_end_to_end_on_box(tmp_path):
    from m3_adapter.gvd_to_vxw import run_gvd
    src = _make_box_vxw(tmp_path)
    out = tmp_path / "box_gvd.vxw"
    stats = run_gvd(
        str(src), str(out), seed_metres=None,
        d_min=0.20, theta_sep=0.40, pad=1,
    )
    # stats dict reports the run
    assert stats["gvd_voxels"] > 0
    assert 0.0 < stats["free_fraction"] <= 1.0
    # output world loads and carries the glowing skeleton material
    w = vxw.read_world(out)
    names = [m.name for m in w.palette.materials]
    assert "gvd_skeleton" in names
```

- [ ] **Step 2: 运行,确认失败**

Run: `pixi run pytest tests/test_gvd.py::test_cli_end_to_end_on_box -q`
Expected: FAIL(`run_gvd` 未定义)。

- [ ] **Step 3: 写实现(追加 run_gvd + main 到 m3_adapter/gvd_to_vxw.py)**

```python
def run_gvd(
    input_vxw: str,
    output_vxw: str,
    seed_metres=None,
    d_min: float = 0.20,
    theta_sep: float = 0.40,
    pad: int = 1,
) -> dict:
    """Full batch GVD pipeline. Returns a stats dict (also printed by main)."""
    t0 = time.perf_counter()
    world = vxw.read_world(Path(input_vxw))
    vsize = world.manifest.voxel_size_meters

    occ, vmin = densify_occupancy(world, pad=pad)
    t_dense = time.perf_counter()

    seed = resolve_seed(
        occ, vmin, vsize, seed_metres=seed_metres,
        spawn_hint=world.manifest.spawn_hint,
    )
    free = flood_free_space(occ, seed)
    free_fraction = float(free.sum()) / float(free.size)
    t_flood = time.perf_counter()

    dist_m, parent = compute_esdf(occ, vsize)
    gvd = extract_gvd(free, dist_m, parent, vsize, d_min=d_min, theta_sep=theta_sep)
    t_gvd = time.perf_counter()

    overlay_gvd_into_world(world, gvd, vmin)
    vxw.write_world(Path(output_vxw), world)
    t_write = time.perf_counter()

    stats = {
        "voxel_size_m": vsize,
        "bbox_dims": tuple(int(x) for x in occ.shape),
        "seed_idx": tuple(int(x) for x in seed),
        "free_cells": int(free.sum()),
        "free_fraction": free_fraction,
        "gvd_voxels": int(gvd.sum()),
        "t_densify_s": round(t_dense - t0, 2),
        "t_flood_s": round(t_flood - t_dense, 2),
        "t_gvd_s": round(t_gvd - t_flood, 2),
        "t_write_s": round(t_write - t_gvd, 2),
        "t_total_s": round(t_write - t0, 2),
        "leak_warning": free_fraction > 0.5,
    }
    print(f"[gvd] bbox {stats['bbox_dims']} voxel {vsize} m")
    print(f"[gvd] seed (dense idx) {stats['seed_idx']}")
    print(
        f"[gvd] flood free: {stats['free_cells']} cells "
        f"= {free_fraction:.1%} of bbox"
    )
    if stats["leak_warning"]:
        print(
            "[gvd] WARN: flood fills >50% of bbox — likely leaking through "
            "wall/ceiling holes; inspect the result, consider --seed or the "
            "2.5D fallback (design §4)."
        )
    print(f"[gvd] GVD skeleton voxels: {stats['gvd_voxels']}")
    print(
        f"[gvd] timing s: densify={stats['t_densify_s']} flood={stats['t_flood_s']} "
        f"gvd={stats['t_gvd_s']} write={stats['t_write_s']} total={stats['t_total_s']}"
    )
    return stats


def _parse_seed(s: str):
    parts = s.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--seed needs 'x,y,z' in metres")
    return [float(p) for p in parts]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_vxw", type=str)
    ap.add_argument("output_vxw", type=str)
    ap.add_argument(
        "--seed", type=_parse_seed, default=None,
        help="interior seed point 'x,y,z' in metres (overrides spawn_hint/auto)",
    )
    ap.add_argument("--d-min", type=float, default=0.20,
                    help="min clearance (m) for a GVD voxel (drops surface noise)")
    ap.add_argument("--theta-sep", type=float, default=0.40,
                    help="min parent spacing (m) to count as different obstacles")
    ap.add_argument("--pad", type=int, default=1,
                    help="free-voxel margin around geometry for flooding")
    args = ap.parse_args()
    run_gvd(
        args.input_vxw, args.output_vxw, seed_metres=args.seed,
        d_min=args.d_min, theta_sep=args.theta_sep, pad=args.pad,
    )


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 运行,确认通过**

Run: `pixi run pytest tests/test_gvd.py -q`
Expected: 8 passed。

- [ ] **Step 5: Commit**

```bash
git add m3_adapter/gvd_to_vxw.py tests/test_gvd.py
git commit -m "feat(gvd): run_gvd pipeline + CLI main (SP-A task 5)"
```

---

### Task 6: 真实数据运行 + 判读(去风险实验本体)

**Files:**
- 无代码改动;产出 `out/apt_gvd.vxw` + 截图 + 结果记录(写入提交信息,PNG 不入库)

- [ ] **Step 1: 在两层公寓上跑 GVD**

Run:
```powershell
pixi run python m3_adapter/gvd_to_vxw.py out/uhumans2_apt_full.vxw out/apt_gvd.vxw --seed 3.875,1.2,5.575
```
(种子取自 apt_ent 的 spawn_hint;apt_full 自身无 spawn_hint。)
Expected:打印 bbox≈`(547,483,185)`、flood 占比、GVD 体素数、各阶段耗时;`out/apt_gvd.vxw` 生成。**记录** flood 占比与是否出现 `WARN`。

- [ ] **Step 2: 若 Step 1 报种子在障碍内 / 越界**

改用自动种子重试:
```powershell
pixi run python m3_adapter/gvd_to_vxw.py out/uhumans2_apt_full.vxw out/apt_gvd.vxw
```
(不带 --seed → 取"离障碍最深点"。)记录哪种种子可用。

- [ ] **Step 3: headless 截图**

Run:
```powershell
F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer --position -10000,-10000 --resolution 1280x720 --quit-after 90 -- --world=F:/slam-voxel-world/out/apt_gvd.vxw --snapshot=F:/slam-voxel-world/out/apt_gvd.png
```
Expected:PNG 生成,无 push_error;查看器正常加载(GVD 体素是青色发光)。

- [ ] **Step 4: 按设计 §4 判读(人工)**

看截图回答:
1. 走廊/房间中部是否出现**连续**青色骨架带?
2. 楼梯井骨架是否**连通两层**?
3. flood 占比是否合理(无 WARN,或 WARN 经判断为误报)?

- **全满足 → SP-A 成功**:进 Step 5 记录成功,SP-A 收口,后续进 SP-D(骨架→稀疏图)或 SP-B(增量)。
- **洪泛漏穿/骨架碎且调参(--d-min 0.1~0.3 / --theta-sep 0.3~0.6)救不回 → 触发回退**:记录失败证据,另立"2.5D 分层 GVD"设计(按楼层切片 + 楼梯层间连接);本实验的价值正是**用一次运行换来这个判断**。

- [ ] **Step 5: 记录结果(提交一行结论,不提交 PNG/vxw 产物)**

把 Step 4 的判读结论(成功/回退 + 关键数字:flood 占比、GVD 体素数、用的种子与参数)写进 architecture.md §10.3 的 adapter 表新增一行,或追加到 SP-A 设计文档末尾的"运行结果"小节。然后:
```bash
git add docs/
git commit -m "docs(gvd): SP-A first-run result on uhumans2 apartment (SP-A task 6)"
```
(`out/apt_gvd.vxw`、`out/apt_gvd.png` 是实验产物,不入库。)

---

## Self-Review 结论(已自查)

- **Spec 覆盖**:种子洪泛(三级策略)✓ Task 4-5;EDT 距离+parent ✓ Task 2;parent 分叉 GVD(d_min/θ_sep)✓ Task 3;叠加发光 .vxw ✓ Task 4;统计+WARN ✓ Task 5;真实数据判读+回退预案 ✓ Task 6;`voxel_gvd.py` 纯算法 / `gvd_to_vxw.py` IO 分离 ✓;核心格式零改动 ✓;合成场景 pytest(洪泛/ESDF/GVD)✓。
- **类型一致性**:`flood_free_space(occupied, seed)` / `compute_esdf(occupied, voxel_size)->(dist_m, parent)` / `extract_gvd(free, dist_m, parent, voxel_size, d_min, theta_sep)` / `densify_occupancy(world, pad)->(occ, vmin)` / `resolve_seed(occupied, vmin, voxel_size, seed_metres, spawn_hint)` / `overlay_gvd_into_world(world, gvd, vmin)` / `run_gvd(...)->dict` —— 各 task 调用签名逐一核对一致。
- **算法正确性要点**:ESDF 对 `occupied`(真障碍)算、`extract_gvd` 用 `free` 掩码限定——避免墙洞处距离场污染;GVD 用 lo/hi 切片对比(非 np.roll),无环绕 bug。
- **已知风险**:洪泛漏穿是数据现实风险,Task 6 的 WARN 指标 + 2.5D 回退是预案,不是失败。
