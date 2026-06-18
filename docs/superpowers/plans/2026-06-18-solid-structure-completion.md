# 实心化结构补全 实现计划(SlabFill 多层 + WallFill 曼哈顿)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** 在 vlayer 加两个 stage-1 generator,把观测的薄表面**补洞 + 增厚成实心方块**:`SlabFill`(floor+ceiling,多层逐峰,向下/上增厚)、`WallFill`(label=19 曼哈顿墙,补面内小洞 + 沿法向增厚,**门窗 observed_free 不填**)。floor_fill 迁移为 `SlabFill(3,"floor")`。

**Architecture:** 沿用现有 `m3_adapter/vlayer`(overlay/compose/pipeline)。两 generator 产 `add` VoxelDelta(generator=slab_fill/wall_fill,binding=persistent),经现有 `compose_structure` → `occupancy_to_vxw`。ObsMap(L0)只读。

**Tech Stack:** Python、numpy、scipy.ndimage、pytest(`pixi run python -m pytest`)。语义:floor=3、ceiling=4、wall=19。

**Setup:** main 上做。commit 结尾 `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`。

---

### Task 1: SlabFill(多层 + 实心化)+ floor_fill 迁移

**Files:**
- Create: `m3_adapter/vlayer/generators/slab.py`
- Modify: `m3_adapter/vlayer/generators/floor.py`(改为薄包装,保持 `FloorFill` 可用 + 旧测试绿)
- Test: `tests/test_slab_fill.py`

- [ ] **Step 1: 写失败测试 `tests/test_slab_fill.py`**
```python
import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.slab import SlabFill

FLOOR = 3
CEIL = 4


def _two_floor_obs():
    # 10 x 20 x 10 grid (vs 0.1). Floor level A at y=3, level B at y=12 (0.9m gap).
    m = ObsMap.new((10, 20, 10), np.zeros(3, np.int64), 0.1)
    for x in range(2, 8):
        for z in range(2, 8):
            for Y in (3, 12):
                m.logodds[x, Y, z] = 5.0
                m.sem_label[x, Y, z] = FLOOR
            m.logodds[x, 5, z] = -5.0     # free above level A
            m.logodds[x, 14, z] = -5.0    # free above level B
    m.logodds[4, 3, 4] = 0.0              # an unobserved hole in level A
    m.sem_label[4, 3, 4] = 0
    return m


def test_slab_fill_multilevel_and_solidify():
    m = _two_floor_obs()
    ov = run_pipeline(m, [SlabFill(FLOOR, "floor", thickness_m=0.2, level_gap_m=0.5, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    # both levels solidified DOWN by thickness 0.2m=2 voxels (Y and Y-1)
    assert occ[3, 3, 3] and occ[3, 2, 3] and sem[3, 3, 3] == FLOOR
    assert occ[3, 12, 3] and occ[3, 11, 3]
    # unobserved hole in level A patched
    assert occ[4, 3, 4] and sem[4, 3, 4] == FLOOR
    # L0 untouched, free not filled
    assert not m.occupancy_mask()[3, 2, 3]
    assert not occ[3, 5, 3]               # the observed-free cell above stays empty


def test_slab_fill_ceiling_extrudes_up():
    m = ObsMap.new((8, 12, 8), np.zeros(3, np.int64), 0.1)
    for x in range(2, 6):
        for z in range(2, 6):
            m.logodds[x, 8, z] = 5.0
            m.sem_label[x, 8, z] = CEIL
            m.logodds[x, 5, z] = -5.0     # free below the ceiling (room air)
    ov = run_pipeline(m, [SlabFill(CEIL, "ceiling", thickness_m=0.2, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[3, 8, 3] and occ[3, 9, 3] and sem[3, 9, 3] == CEIL   # extruded UP
```

- [ ] **Step 2: 跑测试确认失败** — `pixi run python -m pytest tests/test_slab_fill.py -v`(ModuleNotFoundError)

- [ ] **Step 3: 实现 `m3_adapter/vlayer/generators/slab.py`**
```python
"""stage-1 结构先验:水平板(floor/ceiling)补洞 + 实心化,多层逐峰。只读 L0。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.overlay import VoxelDelta


def _cluster_levels(y_indices, gap: int):
    """把 Y 索引按 > gap 的间隔聚成多层;每层返回出现最多的代表 Y。"""
    if len(y_indices) == 0:
        return []
    u = np.unique(y_indices)
    clusters, cur = [], [int(u[0])]
    for v in u[1:]:
        if int(v) - cur[-1] > gap:
            clusters.append(cur); cur = []
        cur.append(int(v))
    clusters.append(cur)
    return [int(max(c, key=lambda y: int(np.count_nonzero(y_indices == y)))) for c in clusters]


class SlabFill:
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, label, side, thickness_m: float = 0.15,
                 level_gap_m: float = 0.5, close_radius: int = 2):
        assert side in ("floor", "ceiling")
        self.label = int(label); self.side = side
        self.thickness_m = float(thickness_m); self.level_gap_m = float(level_gap_m)
        self.close_radius = int(close_radius)
        self.id = f"slab_fill_{side}"; self.generator = "slab_fill"

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        vs = obs.voxel_size; ny = occ.shape[1]
        surf = occ & (sem == self.label)
        if not surf.any():
            return []
        T = max(1, int(round(self.thickness_m / vs)))
        gap = max(1, int(round(self.level_gap_m / vs)))
        ys = np.argwhere(surf)[:, 1]
        deltas = []
        for Y in _cluster_levels(ys, gap):
            ylo = max(0, Y - gap); yhi = min(ny, Y + gap + 1)
            foot = free[:, ylo:yhi, :].any(axis=1) | surf[:, max(0, Y - 1):Y + 2, :].any(axis=1)
            if self.close_radius > 0:
                st = ndimage.generate_binary_structure(2, 1)
                foot = ndimage.binary_closing(foot, structure=st, iterations=self.close_radius)
            yy_list = [Y - k for k in range(T)] if self.side == "floor" else [Y + k for k in range(T)]
            for x, z in np.argwhere(foot):
                for yy in yy_list:
                    if 0 <= yy < ny and not occ[x, yy, z] and not free[x, yy, z]:
                        deltas.append(VoxelDelta((int(x), int(yy), int(z)), "add",
                                                 self.label, self.generator, self.default_binding))
        return deltas
```

- [ ] **Step 4: floor_fill 迁移** — 把 `m3_adapter/vlayer/generators/floor.py` 改为:
```python
"""floor_fill 已并入 SlabFill;保留 FloorFill 名以兼容现有调用/测试。"""
from m3_adapter.vlayer.generators.slab import SlabFill


def FloorFill(close_radius: int = 2):
    return SlabFill(3, "floor", close_radius=close_radius)
```

- [ ] **Step 5: 跑测试** — `pixi run python -m pytest tests/test_slab_fill.py tests/test_floor_fill.py -v` → 全 PASS(含迁移后旧 floor_fill 测试)。

- [ ] **Step 6: Commit**
```bash
git add m3_adapter/vlayer/generators/slab.py m3_adapter/vlayer/generators/floor.py tests/test_slab_fill.py
git commit -m "feat(vlayer): SlabFill — multi-level floor/ceiling fill + solidify; floor_fill -> SlabFill"
```

---

### Task 2: WallFill(曼哈顿 + 保留开口 + 增厚)

**Files:**
- Create: `m3_adapter/vlayer/generators/wall.py`
- Test: `tests/test_wall_fill.py`

- [ ] **Step 1: 写失败测试 `tests/test_wall_fill.py`**
```python
import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.wall import WallFill

WALL = 19


def _wall_with_door_and_hole():
    # A wall at fixed x=5, spanning y 2..8, z 2..8 (a y-z plane).
    m = ObsMap.new((10, 12, 12), np.zeros(3, np.int64), 0.1)
    for y in range(2, 9):
        for z in range(2, 9):
            m.logodds[5, y, z] = 5.0
            m.sem_label[5, y, z] = WALL
    # doorway: observed-free (we saw THROUGH it), y 2..4, z 5..6 — must NOT be filled
    for y in range(2, 5):
        for z in range(5, 7):
            m.logodds[5, y, z] = -5.0
            m.sem_label[5, y, z] = 0
    # a small UNobserved hole (unknown, logodds 0) at (5,6,6) — should be patched
    m.logodds[5, 6, 6] = 0.0
    m.sem_label[5, 6, 6] = 0
    return m


def test_wall_fill_patches_hole_preserves_door_and_thickens():
    m = _wall_with_door_and_hole()
    ov = run_pipeline(m, [WallFill(thickness_m=0.2, min_wall_cells=10, close_radius=1)])
    occ, sem = compose_structure(m, ov)
    assert occ[5, 6, 6] and sem[5, 6, 6] == WALL     # unobserved hole patched
    assert not occ[5, 3, 5]                          # doorway (observed_free) NOT filled
    assert occ[6, 6, 6] and sem[6, 6, 6] == WALL     # thickened along +x (T=2)
    assert not m.occupancy_mask()[5, 6, 6]           # L0 untouched
```

- [ ] **Step 2: 跑测试确认失败** — `pixi run python -m pytest tests/test_wall_fill.py -v`

- [ ] **Step 3: 实现 `m3_adapter/vlayer/generators/wall.py`**
```python
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

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask(); sem = obs.sem_label; free = obs.observed_free_mask()
        vs = obs.voxel_size; nx, ny, nz = occ.shape
        wall = occ & (sem == WALL_LABEL)
        if not wall.any():
            return []
        T = max(1, int(round(self.thickness_m / vs)))
        st = ndimage.generate_binary_structure(2, 1)
        deltas = []

        def _do_slice(plane2d, to_world, normal_axis_len, set_along):
            # plane2d: bool (a,b) at a fixed slice; to_world(ia,ib,d)->(x,y,z);
            # closing in-plane, thicken along the slice's constant axis by T.
            if int(plane2d.sum()) < self.min_wall_cells:
                return
            aa, bb = np.nonzero(plane2d)
            a0, a1, b0, b1 = aa.min(), aa.max(), bb.min(), bb.max()
            sub = plane2d[a0:a1 + 1, b0:b1 + 1]
            filled = ndimage.binary_closing(sub, structure=st, iterations=self.close_radius)
            fa, fb = np.nonzero(filled)
            for ia, ib in zip(fa, fb):
                A = a0 + ia; B = b0 + ib
                for d in range(T):
                    x, y, z = to_world(A, B, d)
                    if not (0 <= x < nx and 0 <= y < ny and 0 <= z < nz):
                        continue
                    if occ[x, y, z] or free[x, y, z]:
                        continue
                    deltas.append(VoxelDelta((int(x), int(y), int(z)), "add",
                                             WALL_LABEL, self.generator, self.default_binding))

        # walls running along Z: fixed x, plane in (y, z), thicken along +x
        for x in range(nx):
            _do_slice(wall[x, :, :], lambda A, B, d, x=x: (x + d, A, B), nx, "x")
        # walls running along X: fixed z, plane in (x, y), thicken along +z
        for z in range(nz):
            _do_slice(wall[:, :, z], lambda A, B, d, z=z: (A, B, z + d), nz, "z")
        return deltas
```

- [ ] **Step 4: 跑测试** — `pixi run python -m pytest tests/test_wall_fill.py -v` → PASS。

- [ ] **Step 5: Commit**
```bash
git add m3_adapter/vlayer/generators/wall.py tests/test_wall_fill.py
git commit -m "feat(vlayer): WallFill — manhattan wall hole-fill + normal thicken, openings preserved"
```

---

### Task 3: 全量回归 + 公寓实心补全冒烟 + 目视

**Files:** 无(验证 + 重生成展示世界)

- [ ] **Step 1: 全量回归**
```
pixi run python -m pytest tests/test_slab_fill.py tests/test_wall_fill.py tests/test_floor_fill.py tests/test_vlayer_overlay.py tests/test_vlayer_compose.py tests/test_vlayer_pipeline.py tests/test_vlayer_export.py tests/test_obsmap.py -q
```
Expected: 全 PASS。

- [ ] **Step 2: 公寓实心补全(语义着色)**
```
pixi run python -c "
from pathlib import Path
from m3_adapter.obsmap import ObsMap
from m3_adapter.uhumans2_to_vxw import _resolve_hydra_paths, load_label_space, build_palette
from m3_adapter.vlayer.generators.slab import SlabFill
from m3_adapter.vlayer.generators.wall import WallFill
from m3_adapter.vlayer.export import obsmap_to_completed_vxw
yaml_path,_ = _resolve_hydra_paths(Path('F:/hydra_ws'),'apartment')
pal = build_palette(load_label_space(yaml_path))
m = ObsMap.load('out/uhumans2_apt_full.vxw/obsmap.npz')
gens=[SlabFill(3,'floor'), SlabFill(4,'ceiling'), WallFill()]
ov = obsmap_to_completed_vxw(m,'out/apt_solid.vxw',generators=gens,palette=pal)
print('added voxels:', len(ov.voxels), 'L0 occ:', int(m.occupancy_mask().sum()))
"
```
Expected: added > 0;`out/apt_solid.vxw/` 生成。

- [ ] **Step 3: 目视(人工)**
```
"/f/Godot/Godot_v4.6.3-stable_win64.exe" --path "F:/slam-voxel-world/godot_viewer" -- --world=F:/slam-voxel-world/out/apt_solid.vxw
```
确认:楼板/天花板厚实(非薄片)、二楼那层也补上、墙更完整但门窗仍开。看完可留 `apt_solid.vxw` 供展示。

- [ ] **Step 4: Commit(若调了默认参数)**
```bash
git add -A
git commit -m "test(vlayer): solid structure completion full regression + apartment smoke"
```

---

## 后续(本计划之外)
- **多分辨率粗化 L1 导出**(compose 细 → 降采样粗块,MC 厚块感 + 吞小洞)—— 用户已认可,下一轮。
- WallFill x/z 双切片去重、按真实法向选增厚方向、斜墙 RANSAC。
- SlabFill 填到"下一层之间"的精确 slab(替代固定 thickness)。
