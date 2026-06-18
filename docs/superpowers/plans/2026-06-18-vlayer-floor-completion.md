# 分层地图:虚拟覆盖层 + Lc 流水线 + floor_fill 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地 spec(`docs/superpowers/specs/2026-06-18-layered-map-architecture-design.md`)的"本期"垂直切片:虚拟覆盖层(结构差异)+ Lc 插件流水线 + `floor_fill` 首插件 + 导出合成,使导出的 `.vxw` 拥有**实心连续地板**(解决"屋子悬浮 / 人掉地"),且观测 `obsmap.npz` 零修改。

**Architecture:** 新增 `m3_adapter/vlayer/` 子包:`overlay.py`(结构体素差异 + 持久化 + compose)、`pipeline.py`(`Generator` 契约 + `LcContext` + 按 `(stage, depends_on)` 拓扑序运行器)、`generators/floor.py`(stage 1 的 `floor_fill`)。导出新增 `obsmap_to_completed_vxw`:跑流水线 → `compose_structure(obsmap, overlay)` 得 `(occ_mask, sem_grid)` → 现有 `occupancy_to_vxw` 物化。ObsMap 只读。

**Tech Stack:** Python 3、numpy、scipy.ndimage(形态学,项目已用)、pytest(`pixi run python -m pytest`)、现有 `m3_adapter/obsmap.py` / `obsmap_export.occupancy_to_vxw` / `vxw_format`。

**Setup(执行前):** 当前在 `main`。先建分支:`git switch -c feat/vlayer-floor-completion`。所有 commit 落在该分支。

**范围说明:** 本计划只做"floor 垂直切片 + 框架"。stage 2–5 现有步骤的"归位适配"、Godot 编辑改写回 overlay、entities.json 在 Godot 端写 provenance —— 留作后续计划(无新行为,风险低)。本计划 Task 6 只做 Python 导出端给 observed 物体打 provenance(小、低风险、属本期)。

---

### Task 1: 覆盖层数据结构 + 持久化(`vlayer/overlay.py`)

**Files:**
- Create: `m3_adapter/vlayer/__init__.py`
- Create: `m3_adapter/vlayer/overlay.py`
- Test: `tests/test_vlayer_overlay.py`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_vlayer_overlay.py
import numpy as np
from m3_adapter.vlayer.overlay import VoxelDelta, Overlay


def test_add_and_iter():
    ov = Overlay()
    ov.add_voxel((1, 2, 3), sem=3, generator="floor_fill", binding="persistent")
    assert len(ov.voxels) == 1
    d = ov.voxels[0]
    assert d.idx == (1, 2, 3) and d.op == "add" and d.sem == 3
    assert d.generator == "floor_fill" and d.binding == "persistent"


def test_save_load_roundtrip(tmp_path):
    ov = Overlay()
    ov.add_voxel((1, 2, 3), sem=3, generator="floor_fill", binding="persistent")
    ov.add_voxel((4, 5, 6), sem=0, generator="manual", binding="independent", op="remove")
    npz = tmp_path / "overlay.npz"
    js = tmp_path / "overlay.json"
    ov.save(npz, js)
    ov2 = Overlay.load(npz, js)
    assert len(ov2.voxels) == 2
    assert ov2.voxels[0].idx == (1, 2, 3) and ov2.voxels[0].generator == "floor_fill"
    assert ov2.voxels[1].op == "remove" and ov2.voxels[1].binding == "independent"
```

- [ ] **Step 2: 跑测试确认失败**

Run: `pixi run python -m pytest tests/test_vlayer_overlay.py -v`
Expected: FAIL(ModuleNotFoundError: m3_adapter.vlayer.overlay)

- [ ] **Step 3: 实现**

```python
# m3_adapter/vlayer/__init__.py
# (empty — package marker)
```

```python
# m3_adapter/vlayer/overlay.py
"""L1 结构覆盖层:相对 L0 观测的稀疏体素差异(add/remove/replace),
带 provenance(generator/binding)。永不修改 L0。

持久化:overlay.npz 存并列数组(idx/op/sem/gen_id/binding_id),
overlay.json 存 id→字符串图例。"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np

_OPS = ("add", "remove", "replace")


@dataclass
class VoxelDelta:
    idx: tuple          # (x, y, z) 整数网格索引(同 ObsMap 数组索引)
    op: str             # "add" | "remove" | "replace"
    sem: int = 0        # super_id(add/replace 用)
    generator: str = "manual"
    binding: str = "persistent"   # live | persistent | independent


@dataclass
class Overlay:
    voxels: list = field(default_factory=list)   # list[VoxelDelta]

    def add_voxel(self, idx, sem=0, generator="manual",
                  binding="persistent", op="add") -> None:
        assert op in _OPS, f"bad op {op}"
        self.voxels.append(VoxelDelta(
            tuple(int(v) for v in idx), op, int(sem), str(generator), str(binding)))

    def save(self, npz_path, json_path) -> None:
        n = len(self.voxels)
        idx = np.zeros((n, 3), np.int32)
        op = np.zeros(n, np.uint8)
        sem = np.zeros(n, np.uint8)
        gens, binds = [], []
        gen_legend, bind_legend = {}, {}
        gen_id = np.zeros(n, np.uint16)
        bind_id = np.zeros(n, np.uint8)
        for i, d in enumerate(self.voxels):
            idx[i] = d.idx
            op[i] = _OPS.index(d.op)
            sem[i] = d.sem
            gid = gen_legend.setdefault(d.generator, len(gen_legend))
            bid = bind_legend.setdefault(d.binding, len(bind_legend))
            gen_id[i] = gid
            bind_id[i] = bid
        np.savez_compressed(npz_path, idx=idx, op=op, sem=sem,
                            gen_id=gen_id, bind_id=bind_id)
        with open(json_path, "w") as f:
            json.dump({"gen_legend": {v: k for k, v in gen_legend.items()},
                       "bind_legend": {v: k for k, v in bind_legend.items()}}, f)

    @classmethod
    def load(cls, npz_path, json_path) -> "Overlay":
        d = np.load(npz_path)
        with open(json_path) as f:
            leg = json.load(f)
        gl = {int(k): v for k, v in leg["gen_legend"].items()}
        bl = {int(k): v for k, v in leg["bind_legend"].items()}
        ov = cls()
        for i in range(len(d["op"])):
            ov.voxels.append(VoxelDelta(
                tuple(int(v) for v in d["idx"][i]),
                _OPS[int(d["op"][i])],
                int(d["sem"][i]),
                gl[int(d["gen_id"][i])],
                bl[int(d["bind_id"][i])]))
        return ov
```

- [ ] **Step 4: 跑测试确认通过**

Run: `pixi run python -m pytest tests/test_vlayer_overlay.py -v`
Expected: PASS(2 passed)

- [ ] **Step 5: 提交**

```bash
git add m3_adapter/vlayer/__init__.py m3_adapter/vlayer/overlay.py tests/test_vlayer_overlay.py
git commit -m "feat(vlayer): structure overlay data structure + persistence"
```

---

### Task 2: compose —— 把覆盖差异叠加到观测(`vlayer/overlay.py`)

**Files:**
- Modify: `m3_adapter/vlayer/overlay.py`(追加 `compose_structure`)
- Test: `tests/test_vlayer_compose.py`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_vlayer_compose.py
import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.overlay import Overlay, compose_structure


def _obs():
    m = ObsMap.new((6, 6, 6), np.zeros(3, np.int64), 0.1)
    # one observed occupied floor cell at (2,1,2)
    m.logodds[2, 1, 2] = 5.0
    m.sem_label[2, 1, 2] = 3
    return m


def test_completed_add_fills_only_unknown():
    m = _obs()
    ov = Overlay()
    ov.add_voxel((3, 1, 2), sem=3, generator="floor_fill", binding="persistent")  # empty -> fill
    ov.add_voxel((2, 1, 2), sem=9, generator="floor_fill", binding="persistent")  # occupied -> NOT overridden
    occ, sem = compose_structure(m, ov)
    assert occ[3, 1, 2] and sem[3, 1, 2] == 3        # gap filled
    assert occ[2, 1, 2] and sem[2, 1, 2] == 3        # observation kept (not overwritten to 9)
    assert not m.occupancy_mask()[3, 1, 2]           # L0 untouched


def test_authored_add_and_remove_override():
    m = _obs()
    ov = Overlay()
    ov.add_voxel((2, 1, 2), op="remove", generator="manual", binding="independent")  # hide observed
    ov.add_voxel((4, 4, 4), sem=7, generator="manual", binding="independent")        # authored add
    occ, sem = compose_structure(m, ov)
    assert not occ[2, 1, 2]                          # removed in output
    assert occ[4, 4, 4] and sem[4, 4, 4] == 7
    assert m.occupancy_mask()[2, 1, 2]               # L0 untouched
```

- [ ] **Step 2: 跑测试确认失败**

Run: `pixi run python -m pytest tests/test_vlayer_compose.py -v`
Expected: FAIL(ImportError: cannot import name 'compose_structure')

- [ ] **Step 3: 实现(追加到 `m3_adapter/vlayer/overlay.py` 末尾)**

```python
def compose_structure(obsmap, overlay):
    """返回 (occ_mask, sem_grid):ObsMap 占据/语义贴上 overlay 结构差异后的输出态。
    不修改 obsmap。优先级:completed 的 add 只填未占据格(观测优先);
    manual/independent(authored)的 add 强制写;remove 抹掉;replace 改语义。"""
    occ = obsmap.occupancy_mask().copy()
    sem = obsmap.sem_label.copy()
    for d in overlay.voxels:
        x, y, z = d.idx
        if d.op == "add":
            authored = d.generator == "manual" or d.binding == "independent"
            if occ[x, y, z] and not authored:
                continue                     # completed 不覆盖观测
            occ[x, y, z] = True
            sem[x, y, z] = d.sem
        elif d.op == "remove":
            occ[x, y, z] = False
        elif d.op == "replace":
            sem[x, y, z] = d.sem
    return occ, sem
```

- [ ] **Step 4: 跑测试确认通过**

Run: `pixi run python -m pytest tests/test_vlayer_compose.py -v`
Expected: PASS(2 passed)

- [ ] **Step 5: 提交**

```bash
git add m3_adapter/vlayer/overlay.py tests/test_vlayer_compose.py
git commit -m "feat(vlayer): compose_structure overlays diffs onto observation (L0 read-only)"
```

---

### Task 3: Generator 契约 + LcContext + 流水线运行器(`vlayer/pipeline.py`)

**Files:**
- Create: `m3_adapter/vlayer/pipeline.py`
- Test: `tests/test_vlayer_pipeline.py`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_vlayer_pipeline.py
import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.overlay import VoxelDelta
from m3_adapter.vlayer.pipeline import LcContext, run_pipeline


class _Gen:
    def __init__(self, gid, stage, deps, coord):
        self.id = gid; self.stage = stage; self.depends_on = deps
        self.default_binding = "persistent"; self._coord = coord
    def run(self, ctx):
        return [VoxelDelta(self._coord, "add", 3, self.id, "persistent")]


def test_runs_in_stage_then_dep_order():
    m = ObsMap.new((4, 4, 4), np.zeros(3, np.int64), 0.1)
    # stage 2 gen depends on a stage 1 gen; lower stage runs first regardless of input order
    g_late = _Gen("late", 2, ["early"], (2, 0, 0))
    g_early = _Gen("early", 1, [], (1, 0, 0))
    ov = run_pipeline(m, [g_late, g_early])
    order = [d.generator for d in ov.voxels]
    assert order == ["early", "late"]


def test_dep_within_same_stage():
    m = ObsMap.new((4, 4, 4), np.zeros(3, np.int64), 0.1)
    a = _Gen("a", 1, ["b"], (0, 0, 0))   # a depends on b -> b first
    b = _Gen("b", 1, [], (1, 0, 0))
    ov = run_pipeline(m, [a, b])
    assert [d.generator for d in ov.voxels] == ["b", "a"]
```

- [ ] **Step 2: 跑测试确认失败**

Run: `pixi run python -m pytest tests/test_vlayer_pipeline.py -v`
Expected: FAIL(ModuleNotFoundError: m3_adapter.vlayer.pipeline)

- [ ] **Step 3: 实现**

```python
# m3_adapter/vlayer/pipeline.py
"""Lc 先验推理流水线:按 (stage, depends_on) 拓扑序运行 generator 插件,
把各插件产出的结构差异累积进一个 Overlay。引擎本身无持久状态。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from m3_adapter.vlayer.overlay import Overlay


class Generator(Protocol):
    id: str
    stage: int
    depends_on: list
    default_binding: str
    def run(self, ctx: "LcContext") -> list: ...   # -> list[VoxelDelta]


@dataclass
class LcContext:
    obsmap: object        # 只读 L0(约定:generator 不得修改)
    overlay: Overlay


def _toposort(generators):
    """先按 stage 升序,再在依赖约束下做稳定 Kahn 排序。"""
    by_id = {g.id: g for g in generators}
    indeg = {g.id: 0 for g in generators}
    for g in generators:
        for dep in g.depends_on:
            if dep in by_id:
                indeg[g.id] += 1
    ordered = []
    remaining = list(generators)
    while remaining:
        ready = [g for g in remaining
                 if all(dep not in by_id or by_id[dep] not in remaining
                        for dep in g.depends_on)]
        if not ready:
            raise ValueError("generator dependency cycle: "
                             + ",".join(g.id for g in remaining))
        ready.sort(key=lambda g: g.stage)      # 同就绪集里低 stage 先
        nxt = ready[0]
        ordered.append(nxt)
        remaining.remove(nxt)
    return ordered


def run_pipeline(obsmap, generators) -> Overlay:
    ctx = LcContext(obsmap=obsmap, overlay=Overlay())
    for g in _toposort(generators):
        ctx.overlay.voxels.extend(g.run(ctx))
    return ctx.overlay
```

- [ ] **Step 4: 跑测试确认通过**

Run: `pixi run python -m pytest tests/test_vlayer_pipeline.py -v`
Expected: PASS(2 passed)

- [ ] **Step 5: 提交**

```bash
git add m3_adapter/vlayer/pipeline.py tests/test_vlayer_pipeline.py
git commit -m "feat(vlayer): Generator contract + LcContext + topo pipeline runner"
```

---

### Task 4: floor_fill 插件(stage 1)(`vlayer/generators/floor.py`)

**Files:**
- Create: `m3_adapter/vlayer/generators/__init__.py`
- Create: `m3_adapter/vlayer/generators/floor.py`
- Test: `tests/test_floor_fill.py`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_floor_fill.py
import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.pipeline import run_pipeline
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.generators.floor import FloorFill

FLOOR = 3


def _obs_holey_floor():
    # 10x6x10 grid. Floor plane at y=1 over a 6x6 footprint (x,z in 2..7),
    # but with a hole at (4,1,4) and (5,1,5). Free space above the footprint.
    m = ObsMap.new((10, 6, 10), np.zeros(3, np.int64), 0.1)
    for x in range(2, 8):
        for z in range(2, 8):
            if (x, z) in [(4, 4), (5, 5)]:
                continue                       # holes in observed floor
            m.logodds[x, 1, z] = 5.0
            m.sem_label[x, 1, z] = FLOOR
            # observed-free air above the floor (rays passed through)
            m.logodds[x, 3, z] = -5.0
    # also free above the holes (so footprint covers them)
    m.logodds[4, 3, 4] = -5.0
    m.logodds[5, 3, 5] = -5.0
    return m


def test_floor_fill_patches_holes_and_keeps_observation():
    m = _obs_holey_floor()
    ov = run_pipeline(m, [FloorFill(close_radius=1)])
    occ, sem = compose_structure(m, ov)
    # holes now filled at y0=1 with floor label
    assert occ[4, 1, 4] and sem[4, 1, 4] == FLOOR
    assert occ[5, 1, 5] and sem[5, 1, 5] == FLOOR
    # an originally-observed floor cell unchanged
    assert occ[2, 1, 2] and sem[2, 1, 2] == FLOOR
    # L0 untouched
    assert not m.occupancy_mask()[4, 1, 4]
    # no floor leaked far outside the footprint
    assert not occ[0, 1, 0]


def test_floor_fill_noop_without_floor():
    m = ObsMap.new((6, 6, 6), np.zeros(3, np.int64), 0.1)
    assert run_pipeline(m, [FloorFill()]).voxels == []
```

- [ ] **Step 2: 跑测试确认失败**

Run: `pixi run python -m pytest tests/test_floor_fill.py -v`
Expected: FAIL(ModuleNotFoundError: m3_adapter.vlayer.generators.floor)

- [ ] **Step 3: 实现**

```python
# m3_adapter/vlayer/generators/__init__.py
# (empty — package marker)
```

```python
# m3_adapter/vlayer/generators/floor.py
"""stage 1 结构先验插件:从 observed-free 反推房间足迹,在主地板高度补洞,
产出 binding=persistent 的地板体素 add 项。只读 L0。

限制(见 spec §12):当前用单一主峰地板高度 Y0;错层/多层留作后续(按峰分区)。"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from m3_adapter.vlayer.overlay import VoxelDelta

FLOOR_LABEL = 3


class FloorFill:
    id = "floor_fill"
    stage = 1
    depends_on: list = []
    default_binding = "persistent"

    def __init__(self, close_radius: int = 2):
        self.close_radius = int(close_radius)

    def run(self, ctx) -> list:
        obs = ctx.obsmap
        occ = obs.occupancy_mask()
        sem = obs.sem_label
        floor = occ & (sem == FLOOR_LABEL)
        if not floor.any():
            return []
        # 1) 主地板高度 Y0 = floor 体素 y 索引(axis 1)的众数
        ys = np.argwhere(floor)[:, 1]
        y0 = int(np.bincount(ys).argmax())
        # 2) 房间足迹:observed-free 投影到 XZ ∪ 已有 floor 的 XZ,再闭运算补缝
        free = obs.observed_free_mask()
        foot = free.any(axis=1) | floor.any(axis=1)        # (nx, nz) bool
        if self.close_radius > 0:
            st = ndimage.generate_binary_structure(2, 1)
            foot = ndimage.binary_closing(foot, structure=st,
                                          iterations=self.close_radius)
        # 3) 在足迹内、Y0 处补未占据格
        deltas = []
        for x, z in np.argwhere(foot):
            if not occ[x, y0, z]:
                deltas.append(VoxelDelta((int(x), int(y0), int(z)), "add",
                                         FLOOR_LABEL, self.id, self.default_binding))
        return deltas
```

- [ ] **Step 4: 跑测试确认通过**

Run: `pixi run python -m pytest tests/test_floor_fill.py -v`
Expected: PASS(2 passed)

- [ ] **Step 5: 提交**

```bash
git add m3_adapter/vlayer/generators/__init__.py m3_adapter/vlayer/generators/floor.py tests/test_floor_fill.py
git commit -m "feat(vlayer): floor_fill generator — observed-free footprint -> patched floor"
```

---

### Task 5: 导出合成 `obsmap_to_completed_vxw`(`vlayer/export.py`)

**Files:**
- Create: `m3_adapter/vlayer/export.py`
- Test: `tests/test_vlayer_export.py`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_vlayer_export.py
import numpy as np
import vxw_format as vxw
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.generators.floor import FloorFill
from m3_adapter.vlayer.export import obsmap_to_completed_vxw

FLOOR = 3


def _obs_holey_floor():
    m = ObsMap.new((10, 6, 10), np.zeros(3, np.int64), 0.1)
    for x in range(2, 8):
        for z in range(2, 8):
            if (x, z) in [(4, 4), (5, 5)]:
                continue
            m.logodds[x, 1, z] = 5.0
            m.sem_label[x, 1, z] = FLOOR
            m.logodds[x, 3, z] = -5.0
    m.logodds[4, 3, 4] = -5.0
    m.logodds[5, 3, 5] = -5.0
    return m


def test_export_writes_solid_floor(tmp_path):
    m = _obs_holey_floor()
    out = tmp_path / "completed.vxw"
    base_occ = int(m.occupancy_mask().sum())
    overlay = obsmap_to_completed_vxw(m, out, generators=[FloorFill(close_radius=1)])
    # overlay added the 2 holes (at least)
    assert len(overlay.voxels) >= 2
    # exported world readable and has more occupied voxels than raw observation
    world = vxw.read_world(out)
    assert world is not None
    # overlay sidecar persisted next to world
    assert (out / "overlay.npz").exists() and (out / "overlay.json").exists()
    # observation file NOT created by export (L0 untouched / not written here)
    assert int(m.occupancy_mask().sum()) == base_occ
```

- [ ] **Step 2: 跑测试确认失败**

Run: `pixi run python -m pytest tests/test_vlayer_export.py -v`
Expected: FAIL(ModuleNotFoundError: m3_adapter.vlayer.export)

- [ ] **Step 3: 实现**

```python
# m3_adapter/vlayer/export.py
"""把 ObsMap(L0)经 Lc 流水线补全后物化为可渲染 .vxw。
流程:run_pipeline -> compose_structure -> occupancy_to_vxw。
overlay 作为 sidecar(overlay.npz/json)持久化到世界目录。L0 只读。"""
from __future__ import annotations

from pathlib import Path

from m3_adapter.obsmap_export import occupancy_to_vxw
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.pipeline import run_pipeline


def obsmap_to_completed_vxw(obsmap, out_path, generators,
                            palette=None, chunk_extent=32):
    """运行补全流水线并导出 .vxw。返回产生的 Overlay。

    palette 为 None 时按非语义模式导出(全部 material_id=1);
    传入 vxw.Palette 则语义着色。"""
    out_path = Path(out_path)
    overlay = run_pipeline(obsmap, generators)
    occ, sem = compose_structure(obsmap, overlay)
    occupancy_to_vxw(
        occ, obsmap.vmin, obsmap.voxel_size, out_path,
        chunk_extent=chunk_extent,
        semantic_grid=(sem if palette is not None else None),
        palette=palette,
    )
    overlay.save(out_path / "overlay.npz", out_path / "overlay.json")
    return overlay
```

- [ ] **Step 4: 跑测试确认通过**

Run: `pixi run python -m pytest tests/test_vlayer_export.py -v`
Expected: PASS(1 passed)

- [ ] **Step 5: 提交**

```bash
git add m3_adapter/vlayer/export.py tests/test_vlayer_export.py
git commit -m "feat(vlayer): obsmap_to_completed_vxw — pipeline+compose -> .vxw with overlay sidecar"
```

---

### Task 6: observed 物体打 provenance(Python 导出端)

**Files:**
- Modify: `m3_adapter/uhumans2_to_vxw.py`(`extract_entities` 内构造 `vxw.Entity` 处,约 457-465 行)
- Test: `tests/test_entity_provenance.py`

- [ ] **Step 1: 写失败测试**

```python
# tests/test_entity_provenance.py
import numpy as np
from m3_adapter.uhumans2_to_vxw import extract_entities

# label 5 (chair) 是 _OBJECT_LABELS 之一;造一团该类体素
def test_extracted_entity_has_observed_provenance():
    n = 60
    rng = np.random.default_rng(0)
    vc = (rng.random((n, 3)) * 3).astype(np.int64) + np.array([10, 10, 10])
    lbl = np.full(n, 5, dtype=np.int64)
    names = {5: "chair"}
    ents, _keep = extract_entities(vc, lbl, 0.1, names,
                                   dbscan_eps_voxels=3.0, min_samples=5)
    assert len(ents) >= 1
    prov = ents[0].custom_meta.get("provenance")
    assert prov == {"generator": "cluster", "binding": "live"}
```

- [ ] **Step 2: 跑测试确认失败**

Run: `pixi run python -m pytest tests/test_entity_provenance.py -v`
Expected: FAIL(KeyError/AssertionError:custom_meta 无 provenance)

- [ ] **Step 3: 实现 —— 在 `extract_entities` 构造 Entity 处补 custom_meta**

在 `m3_adapter/uhumans2_to_vxw.py` 的 `entities.append(vxw.Entity(...))` 调用中,加入 `custom_meta`:

```python
            entities.append(vxw.Entity(
                id=str(uuid.uuid4()),
                label=int(lbl),
                label_name=label_names.get(int(lbl), "?"),
                position=tuple(float(c) for c in center),
                rotation=(qx, qy, qz, qw),
                bbox_dims=tuple(float(c) for c in dims),
                voxel_count=int(len(vc_c)),
                custom_meta={"provenance": {"generator": "cluster", "binding": "live"}},
            ))
```

(若该 `vxw.Entity` 已有其它 custom_meta 来源,改为先建 dict 再设 `["provenance"]`,避免覆盖。当前该构造处无 custom_meta,直接传即可。)

- [ ] **Step 4: 跑测试确认通过**

Run: `pixi run python -m pytest tests/test_entity_provenance.py -v`
Expected: PASS(1 passed)

- [ ] **Step 5: 提交**

```bash
git add m3_adapter/uhumans2_to_vxw.py tests/test_entity_provenance.py
git commit -m "feat(vlayer): tag extracted entities with provenance{cluster,live}"
```

---

### Task 7: 全量回归 + 真实公寓冒烟

**Files:** 无新增(验证)

- [ ] **Step 1: 跑全部相关测试**

Run: `pixi run python -m pytest tests/test_vlayer_overlay.py tests/test_vlayer_compose.py tests/test_vlayer_pipeline.py tests/test_floor_fill.py tests/test_vlayer_export.py tests/test_entity_provenance.py tests/test_obsmap.py -v`
Expected: 全 PASS

- [ ] **Step 2: 真实公寓冒烟(用本会话重建的 ObsMap)**

Run:
```bash
pixi run python -c "
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.generators.floor import FloorFill
from m3_adapter.vlayer.export import obsmap_to_completed_vxw
m = ObsMap.load('out/uhumans2_apt_full.vxw/obsmap.npz')
before = int(m.occupancy_mask().sum())
ov = obsmap_to_completed_vxw(m, 'out/_floor_smoke.vxw', generators=[FloorFill(close_radius=2)])
print('overlay voxels added:', len(ov.voxels), 'base occ:', before)
"
```
Expected: 打印 `overlay voxels added: >0`;`out/_floor_smoke.vxw/` 生成,含 `overlay.npz/json`。

- [ ] **Step 3: Godot 目视(可选)**

Run:
```bash
"/f/Godot/Godot_v4.6.3-stable_win64_console.exe" --path "F:/slam-voxel-world/godot_viewer" --position -10000,-10000 --resolution 1280x720 --quit-after 300 -- --world=F:/slam-voxel-world/out/_floor_smoke.vxw --snapshot=F:/slam-voxel-world/out/_floor_smoke.png
```
Expected: world_loaded 体素数 > 原始;地板更连续。看 `_floor_smoke.png` 确认无悬浮孔洞。看完 `rm -rf out/_floor_smoke.vxw out/_floor_smoke.png`。

- [ ] **Step 4: 提交(若冒烟需微调 close_radius 等默认值)**

```bash
git add -A
git commit -m "test(vlayer): full regression + apartment floor-fill smoke"
```

---

## 后续计划(本计划之外,各自成 spec/plan)
- stage 2–5 现有步骤(extract_entities / objects / scene_graph / gvd)正式适配到 `Generator` 契约(薄封装,行为不变)。
- Godot `entity_edit_controller`/`entity_inspector` 物体编辑写回 `entities.json`(扩 op/ref);结构体素编辑写回 `overlay.*`;导出器 `obsmap_to_world` 全面走 compose(物体侧)。
- 生死/冲突规则的事件化(在线流式 merge_observation 触发 + Godot "保留?"提示)。
- 其余 Phase ④ generator:墙面平面拟合 / 遮挡恢复 / 模板拟合 / 物理验证(stage 6)。
