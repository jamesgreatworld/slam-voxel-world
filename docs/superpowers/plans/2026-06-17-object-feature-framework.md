# ObjectFeature Plugin Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor object descriptors into a modular, pluggable ObjectFeature framework so new features (color histogram, CNN embedding, ORB) can be added without touching the matching logic.

**Architecture:** A new `features.py` module defines an `ObjectFeature` ABC with `name/weight/extract/distance`; `ShapeFeature` implements PCA shape as the first plugin; `feature_cost()` composes over all registered features. `ObjectNode.shape_sig` is replaced by `features: dict`; `merge_observation` cost replaces `shape_weight*shape_dist` with `feature_cost()`.

**Tech Stack:** Python 3.11, numpy, dataclasses, abc, pixi/pytest

---

### Task 1: Create `m3_adapter/gvd/features.py`

**Files:**
- Create: `m3_adapter/gvd/features.py`

- [ ] **Step 1: Write `features.py` with ObjectFeature ABC, ShapeFeature, DEFAULT_FEATURES, extract_features, feature_cost**

See spec for exact code. Key points:
- `ObjectFeature` ABC has `name: str`, `weight: float`, abstract `extract(cells, voxel_size, ctx) -> tuple`, abstract `distance(a, b) -> float`
- `ShapeFeature.name = "shape"`, `weight = 2.0`; PCA: `cov = c.T @ c / len(c)`, `eigvalsh`, `sqrt(ev)*voxel_size` descending; returns `(0.0,0.0,0.0)` if len<3
- `ShapeFeature.distance`: pad both to max length, `linalg.norm(a-b)`
- `DEFAULT_FEATURES = [ShapeFeature()]`
- `extract_features(cells, voxel_size, ctx=None, features=None) -> dict`
- `feature_cost(feats_a, feats_b, features=None) -> float`: sum `f.weight * f.distance(a, b)` over features present in BOTH dicts
- Include ColorFeature / EmbeddingFeature comment block as shown in spec

- [ ] **Step 2: Run the new module to make sure it imports clean**

```
pixi run python -c "from m3_adapter.gvd.features import ShapeFeature, feature_cost, extract_features; import numpy as np; print('OK')"
```
Expected: `OK`

---

### Task 2: Migrate `objects.py`

**Files:**
- Modify: `m3_adapter/gvd/objects.py`

- [ ] **Step 1: Replace `shape_sig: tuple = ()` field with `features: dict = field(default_factory=dict)`**

In `ObjectNode`, change:
```python
shape_sig: tuple = ()  # principal-axis extents (std-devs) in metres, descending
```
to:
```python
features: dict = field(default_factory=dict)
```
Also add `from dataclasses import dataclass, field` (currently only `dataclass`).

- [ ] **Step 2: Replace inline PCA in `extract_objects` with `extract_features` call**

Remove the block starting `# shape signature:` through `shape_sig=sig`. Replace with:
```python
from m3_adapter.gvd.features import extract_features
feats = extract_features(cells, voxel_size, ctx={"occ": occ, "sem": sem})
```
And change `ObjectNode(... shape_sig=sig)` to `ObjectNode(... features=feats)`.

- [ ] **Step 3: Verify objects module imports and extract_objects returns features**

```
pixi run python -c "
import numpy as np
from m3_adapter.gvd.objects import extract_objects
occ = np.zeros((20,20,20), bool); sem = np.zeros((20,20,20), np.uint8)
occ[5:8,5:8,5:8]=True; sem[5:8,5:8,5:8]=9
objs = extract_objects(occ, sem, 0.1, np.zeros(3, int), label_names={9:'chair'}, min_voxels=5)
print(objs[0].features)
"
```
Expected: dict with `"shape"` key and a 3-tuple.

---

### Task 3: Migrate `scene_graph.py`

**Files:**
- Modify: `m3_adapter/gvd/scene_graph.py`

- [ ] **Step 1: Update `build_scene_graph` to store `attrs["features"]` (and mirror `attrs["shape"]`)**

In the object node creation block, change:
```python
"shape": list(getattr(o, "shape_sig", ())),
```
to:
```python
"features": dict(getattr(o, "features", {}) or {}),
"shape": list((getattr(o, "features", {}) or {}).get("shape", ())),
```
This stores canonical `attrs["features"]` and mirrors `attrs["shape"]` for back-compat.

- [ ] **Step 2: Replace `_shape_dist` + `shape_weight*sdist` in `merge_observation` with `feature_cost`**

Decision: **keep `shape_weight` param in the signature (deprecated/ignored)** to avoid breaking callers (e.g. test passes `shape_weight=3.0`). Feature weights now live in the feature objects themselves.

In `merge_observation`:
1. Remove the `_shape_dist` inner function.
2. Add import at top of function: `from m3_adapter.gvd.features import feature_cost`
3. Replace:
```python
sdist = _shape_dist(ex_info, fo)
cost[i, jj] = dist + shape_weight * sdist
```
with:
```python
fresh_feats = getattr(fo, "features", {}) or {}
ex_feats = ex_info["attrs"].get("features", {})
cost[i, jj] = dist + feature_cost(ex_feats, fresh_feats)
```

- [ ] **Step 3: Run scene_graph import check**

```
pixi run python -c "from m3_adapter.gvd.scene_graph import build_scene_graph, merge_observation; print('OK')"
```
Expected: `OK`

---

### Task 4: Migrate `tests/test_object_tracking.py`

**Files:**
- Modify: `tests/test_object_tracking.py`

- [ ] **Step 1: Update `test_shape_sig_distinguishes_tall_vs_flat` to use `o.features["shape"]`**

Change:
```python
s = o.shape_sig
```
to:
```python
s = o.features["shape"]
```

- [ ] **Step 2: Update `_obj` helper to set `o.features = {"shape": sig}` instead of `o.shape_sig = sig`**

Change:
```python
o.shape_sig = sig
```
to:
```python
o.features = {"shape": sig}
```

- [ ] **Step 3: Update `id_by_shape` in `test_shape_disambiguates_same_class_assignment` to read from `n.attrs["features"]["shape"]`**

Change:
```python
if [round(x,3) for x in n.attrs["shape"]] == [round(x,3) for x in sig]:
```
to:
```python
feat_shape = n.attrs.get("features", {}).get("shape", n.attrs.get("shape", ()))
if [round(x,3) for x in feat_shape] == [round(x,3) for x in sig]:
```
(reads `attrs["features"]["shape"]`, falls back to `attrs["shape"]` for carry-overs)

- [ ] **Step 4: Drop `shape_weight` from `merge_observation` call OR keep it (it's now ignored)**

The call `merge_observation(sg1, pg, fresh, match_radius_m=5.0, max_misses=3, shape_weight=3.0)` still works because `shape_weight` is kept in the signature (deprecated). No change needed.

- [ ] **Step 5: Add `test_feature_cost_composes_and_is_extensible` test**

Append to `tests/test_object_tracking.py`:
```python
def test_feature_cost_composes_and_is_extensible():
    from m3_adapter.gvd.features import ShapeFeature, feature_cost, extract_features
    import numpy as np
    # a line of voxels along x -> shape sig dominant axis
    cells = np.array([[i, 0, 0] for i in range(10)])
    f = extract_features(cells, 0.1, ctx={})
    assert "shape" in f and f["shape"][0] > f["shape"][1]
    # identical features -> zero cost; different -> positive
    assert feature_cost({"shape": f["shape"]}, {"shape": f["shape"]}) == 0.0
    assert feature_cost({"shape": (0.5,0.1,0.1)}, {"shape": (0.1,0.1,0.1)}) > 0
    # a descriptor missing a feature contributes nothing (graceful)
    assert feature_cost({}, {"shape": f["shape"]}) == 0.0
```

---

### Task 5: Run tests and commit

- [ ] **Step 1: Run targeted tests**

```
pixi run pytest tests/test_object_tracking.py tests/test_scene_graph.py tests/test_scene_graph_incremental.py tests/test_gvd_objects.py -q
```
Expected: all pass.

- [ ] **Step 2: Run full test suite**

```
pixi run pytest tests/ -q
```
Expected: all pass.

- [ ] **Step 3: Commit**

```
git add m3_adapter/gvd/features.py m3_adapter/gvd/objects.py m3_adapter/gvd/scene_graph.py tests/test_object_tracking.py
git commit -m "refactor(gvd): modular ObjectFeature framework — shape as first plugin; color/embedding extension points (extensible tracking)"
```
