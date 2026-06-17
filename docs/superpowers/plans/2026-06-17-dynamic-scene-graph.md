# Dynamic Scene Graph (DSG) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a hierarchical Dynamic Scene Graph (Building > Rooms > {Places, Objects}) with support-parenting (cup-on-table), CRUD + query, JSON roundtrip, pipeline integration, and CLI flag.

**Architecture:** `SceneGraph` holds a dict of `SceneNode` keyed by string id. `build_scene_graph()` assembles the hierarchy from `PlacesGraph` + list of `ObjectNode`. Objects resting on other objects (bbox y-overlap test) are support-parented to the supporting object; others fall through to their room. Pipeline wires it as an optional stage after objects are extracted. CLI exposes `--scene-graph`.

**Tech Stack:** Python 3.11, dataclasses, numpy, pixi (pytest runner)

---

### Task 1: Write scene_graph.py (SceneNode + SceneGraph + build_scene_graph)

**Files:**
- Create: `m3_adapter/gvd/scene_graph.py`

- [ ] **Step 1: Write the file**

Exact content as provided in spec (SceneNode dataclass, SceneGraph class with CRUD/query/JSON, build_scene_graph function). See spec Part A and Part B.

- [ ] **Step 2: Verify syntax**

Run: `pixi run python -c "from m3_adapter.gvd.scene_graph import SceneGraph, SceneNode, build_scene_graph; print('OK')"`
Expected: `OK`

---

### Task 2: Write tests/test_scene_graph.py

**Files:**
- Create: `tests/test_scene_graph.py`

- [ ] **Step 1: Write the test file** (exact content from spec)

- [ ] **Step 2: Run scene graph tests**

Run: `pixi run pytest tests/test_scene_graph.py -q`
Expected: 2 passed

---

### Task 3: Add scene_graph: bool to GvdConfig + pipeline integration

**Files:**
- Modify: `m3_adapter/gvd/pipeline.py`

- [ ] **Step 1: Add field to GvdConfig**

Add `scene_graph: bool = False` after `object_min_voxels`.
Add enforcement: if scene_graph, set objects=True (inside `run()`).

- [ ] **Step 2: Add scene graph build + JSON output in run()**

After objects extracted + linked, before render.stamp_skeleton:
```python
sg = None
if cfg.scene_graph:
    from m3_adapter.gvd.scene_graph import build_scene_graph
    sg = build_scene_graph(graph_obj, objs)
    import json
    Path(cfg.output_vxw).with_suffix(".scene_graph.json").write_text(
        json.dumps(sg.to_dict(), indent=2))
    stats_scene = {layer: sum(1 for n in sg.nodes.values() if n.layer == layer)
                   for layer in ("building","room","place","object")}
    print(f"[gvd] scene_graph: {len(sg.nodes)} nodes {stats_scene}")
```

- [ ] **Step 3: Add scene_graph_nodes to stats dict**

In the stats dict at end of run(), add: `"scene_graph_nodes": len(sg.nodes) if sg else 0,`

---

### Task 4: Add --scene-graph CLI flag

**Files:**
- Modify: `m3_adapter/gvd_to_vxw.py`

- [ ] **Step 1: Add argparse flag**

After `--object-min-voxels` argument, add:
```python
ap.add_argument("--scene-graph", action="store_true",
                help="build hierarchical Dynamic Scene Graph (implies --objects --graph --rooms)")
```

- [ ] **Step 2: Wire into GvdConfig construction**

In the cfg construction block, update to pass scene_graph:
```python
cfg = GvdConfig(
    ...,
    scene_graph=args.scene_graph,
)
```
Also: if scene_graph flag set, override objects/graph/rooms to True:
```python
if args.scene_graph:
    args.objects = True
    args.graph = True
    args.rooms = True
```
(Add this block after `args = ap.parse_args()` and before cfg construction.)

- [ ] **Step 3: Verify --help shows --scene-graph**

Run: `pixi run python m3_adapter/gvd_to_vxw.py --help`
Expected: output includes `--scene-graph`

---

### Task 5: Run full test suite

- [ ] **Step 1: Run all tests**

Run: `pixi run pytest tests/ -q`
Expected: all pass (or pre-existing failures unchanged)

---

### Task 6: Commit

- [ ] **Step 1: Commit**

```bash
git add m3_adapter/gvd/scene_graph.py m3_adapter/gvd/pipeline.py m3_adapter/gvd_to_vxw.py tests/test_scene_graph.py
git commit -m "feat(gvd): hierarchical Dynamic Scene Graph (Building>Room>Place/Object) + support-parenting + CRUD/query"
```
