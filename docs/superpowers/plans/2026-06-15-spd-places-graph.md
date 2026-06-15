# SP-D Places 图 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Steps use `- [ ]`.

**Goal:** 把 SP-A 的细化 GVD 骨架稀疏化成 places 拓扑图(节点+边),写 `.graph.json` + 品红节点标记叠加 `.vxw`,Godot 可视。

**Architecture:** 新纯算法模块 `m3_adapter/gvd_graph.py`(skeleton_to_graph,度数+DBSCAN 合并+链/直连边)。扩展 `gvd_to_vxw.py` 加 `--graph`:thin 后调它,写 JSON + 节点标记。复用 SP-A 的 append-material 叠加模式。

**Tech Stack:** numpy, scipy.ndimage(convolve/label/binary_dilation), sklearn DBSCAN(已装), json。

**已核实事实**:`run_gvd` 内 thin 后有 `gvd`(bool)、`dist_m`、`vmin`、`vsize` 在作用域;`vxw.Material/Chunk/Palette`、append-material 模式见 `overlay_gvd_into_world`。

---

### Task 1: gvd_graph.py — skeleton_to_graph(纯算法)

**Files:** Create `m3_adapter/gvd_graph.py`; Test `tests/test_gvd_graph.py`.

- [ ] **Step 1: 写失败测试 `tests/test_gvd_graph.py`**

```python
import numpy as np
from m3_adapter.gvd_graph import skeleton_to_graph


def _line(n=21, axis=0, fixed=10, lo=5, hi=15):
    g = np.zeros((n, n, n), dtype=bool)
    sl = [fixed, fixed, fixed]
    sl[axis] = slice(lo, hi + 1)
    g[tuple(sl)] = True
    return g


def test_line_two_endpoints_one_edge():
    g = _line()
    dist = np.ones_like(g, dtype=float)
    nodes, edges = skeleton_to_graph(g, dist, voxel_size=1.0, merge_radius_m=0.15)
    assert len(nodes) == 2
    assert all(n["type"] == "endpoint" for n in nodes)
    assert len(edges) == 1


def test_cross_one_junction_four_endpoints_four_edges():
    n = 21
    g = np.zeros((n, n, n), dtype=bool)
    g[5:16, 10, 10] = True   # horizontal arm (x)
    g[10, 5:16, 10] = True   # vertical arm (y)
    dist = np.ones_like(g, dtype=float)
    nodes, edges = skeleton_to_graph(g, dist, voxel_size=1.0, merge_radius_m=0.15)
    junctions = [x for x in nodes if x["type"] == "junction"]
    endpoints = [x for x in nodes if x["type"] == "endpoint"]
    assert len(junctions) == 1
    assert len(endpoints) == 4
    assert len(nodes) == 5
    assert len(edges) == 4


def test_hairball_merges_to_single_junction():
    n = 25
    g = np.zeros((n, n, n), dtype=bool)
    g[11:14, 11:14, 11:14] = True       # 3x3x3 hairball blob
    g[14:20, 12, 12] = True             # arm +x
    g[5:11, 12, 12] = True              # arm -x
    g[12, 14:20, 12] = True             # arm +y
    g[12, 5:11, 12] = True              # arm -y
    dist = np.ones_like(g, dtype=float)
    nodes, edges = skeleton_to_graph(g, dist, voxel_size=1.0, merge_radius_m=0.20)
    # the 27-voxel blob collapses to ONE junction node, not many
    assert len([x for x in nodes if x["type"] == "junction"]) == 1
    assert len([x for x in nodes if x["type"] == "endpoint"]) == 4
    assert len(edges) == 4
```

Run `pixi run pytest tests/test_gvd_graph.py -q` → FAIL (module missing).

- [ ] **Step 2: 实现 `m3_adapter/gvd_graph.py`**

```python
"""gvd_graph.py — sparsify a thinned GVD skeleton into a places graph (SP-D).

Pure numpy/scipy/sklearn: arrays in, graph (lists) out. No file I/O. See
docs/superpowers/specs/2026-06-15-spd-places-graph-design.md.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage
from sklearn.cluster import DBSCAN


def _node_type(degrees: np.ndarray) -> str:
    if (degrees >= 3).any():
        return "junction"
    if (degrees == 1).any():
        return "endpoint"
    if (degrees == 0).any():
        return "isolated"
    return "chain"


def skeleton_to_graph(gvd, dist_m, voxel_size, merge_radius_m=0.15):
    """Sparsify a 1-voxel-wide skeleton into a places graph.

    Args:
        gvd: bool array, the thinned skeleton.
        dist_m: float array, clearance (metres to nearest obstacle) per cell.
        voxel_size: metres per voxel.
        merge_radius_m: key voxels within this radius merge into one node.

    Returns:
        (nodes, edges):
          nodes: list of {"idx":(i,j,k), "clearance_m":float, "degree":int,
                          "type":"junction"|"endpoint"|"isolated"|"loop"}
          edges: list of (a_id, b_id, length_m), a_id<b_id, deduped.
    """
    if not gvd.any():
        return [], []
    kernel = np.ones((3, 3, 3), dtype=np.uint8)
    kernel[1, 1, 1] = 0
    nbr = ndimage.convolve(gvd.astype(np.uint8), kernel, mode="constant", cval=0)
    degree = nbr * gvd

    key = gvd & (degree != 2)
    key_coords = np.argwhere(key)
    if len(key_coords) == 0:
        any_idx = tuple(int(x) for x in np.argwhere(gvd)[0])
        return ([{"idx": any_idx, "clearance_m": float(dist_m[any_idx]),
                  "degree": 2, "type": "loop"}], [])

    eps_vox = max(merge_radius_m / voxel_size, 1e-6)
    labels = DBSCAN(eps=eps_vox, min_samples=1).fit(key_coords).labels_
    n_clusters = int(labels.max()) + 1

    key_to_cluster = {}
    for coord, lab in zip(key_coords, labels):
        key_to_cluster[(int(coord[0]), int(coord[1]), int(coord[2]))] = int(lab)

    nodes = []
    for c in range(n_clusters):
        members = key_coords[labels == c]
        mdeg = degree[members[:, 0], members[:, 1], members[:, 2]]
        centroid = members.mean(axis=0)
        snap = members[int(np.argmin(((members - centroid) ** 2).sum(axis=1)))]
        clearance = float(dist_m[members[:, 0], members[:, 1], members[:, 2]].max())
        nodes.append({
            "idx": (int(snap[0]), int(snap[1]), int(snap[2])),
            "clearance_m": clearance,
            "degree": int(mdeg.max()),
            "type": _node_type(mdeg),
        })

    edge_len = {}

    def _add_edge(a, b, length_m):
        if a == b:
            return
        e = (min(a, b), max(a, b))
        if e not in edge_len or length_m < edge_len[e]:
            edge_len[e] = length_m

    struct26 = ndimage.generate_binary_structure(3, 3)

    # 4a. chain edges
    chain = gvd & ~key
    clab, ncomp = ndimage.label(chain, structure=struct26)
    for comp in range(1, ncomp + 1):
        comp_mask = clab == comp
        size = int(comp_mask.sum())
        dil = ndimage.binary_dilation(comp_mask, structure=struct26)
        tc = np.argwhere(dil & key)
        clusters = {key_to_cluster[(int(t[0]), int(t[1]), int(t[2]))] for t in tc}
        if len(clusters) == 2:
            a, b = sorted(clusters)
            _add_edge(a, b, size * voxel_size)

    # 4b. direct key-key adjacency across clusters
    offsets = [(dx, dy, dz)
               for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               if not (dx == 0 and dy == 0 and dz == 0)]
    shape = gvd.shape
    for coord, lab in zip(key_coords, labels):
        x, y, z = int(coord[0]), int(coord[1]), int(coord[2])
        for dx, dy, dz in offsets:
            nx, ny, nz = x + dx, y + dy, z + dz
            if 0 <= nx < shape[0] and 0 <= ny < shape[1] and 0 <= nz < shape[2]:
                nlab = key_to_cluster.get((nx, ny, nz))
                if nlab is not None and nlab != int(lab):
                    _add_edge(int(lab), nlab, voxel_size)

    edges = [(a, b, edge_len[(a, b)]) for (a, b) in sorted(edge_len)]
    return nodes, edges
```

Run `pixi run pytest tests/test_gvd_graph.py -q` → expect 3 passed.

- [ ] **Step 3: Commit**
```bash
git add m3_adapter/gvd_graph.py tests/test_gvd_graph.py
git commit -m "feat(gvd): skeleton_to_graph places-graph sparsifier (SP-D task 1)"
```

---

### Task 2: gvd_to_vxw 接入 --graph(JSON + 节点标记)

**Files:** Modify `m3_adapter/gvd_to_vxw.py`; Test `tests/test_gvd.py`.

- [ ] **Step 1: 写失败测试(追加 tests/test_gvd.py)**

```python
def test_run_gvd_graph_emits_json_and_markers(tmp_path):
    import json
    from m3_adapter.gvd_to_vxw import run_gvd
    src = _make_box_vxw(tmp_path)
    out = tmp_path / "box_g.vxw"
    stats = run_gvd(str(src), str(out), seed_metres=None, d_min=0.20,
                    theta_sep=0.40, pad=1, band_max=1.0, thin=True, graph=True)
    assert stats["graph_nodes"] >= 1
    assert "graph_edges" in stats
    gjson = tmp_path / "box_g.graph.json"
    assert gjson.exists()
    g = json.loads(gjson.read_text())
    assert "nodes" in g and "edges" in g
    assert all("pos_m" in n and "clearance_m" in n for n in g["nodes"])
    # the place_node material is overlaid
    w = vxw.read_world(out)
    assert "place_node" in [m.name for m in w.palette.materials]
```

Run `pixi run pytest tests/test_gvd.py::test_run_gvd_graph_emits_json_and_markers -q` → FAIL.

- [ ] **Step 2: 实现 — 在 m3_adapter/gvd_to_vxw.py 追加 helpers + 接入 run_gvd/main**

加导入(顶部 import 区):
```python
import json
```

加常量(GVD_* 常量附近):
```python
PLACE_NODE_NAME = "place_node"
PLACE_NODE_COLOR = (255, 0, 255)  # magenta
PLACE_NODE_EMISSION = 4.0
```

加两个 helper(文件内,overlay_gvd_into_world 之后):
```python
def _write_graph_json(path: Path, nodes, edges, vmin, voxel_size) -> None:
    out_nodes = []
    for i, nd in enumerate(nodes):
        wv = np.asarray(nd["idx"], dtype=np.int64) + vmin
        pos = (wv.astype(float) * voxel_size)
        out_nodes.append({
            "id": i,
            "pos_m": [float(pos[0]), float(pos[1]), float(pos[2])],
            "clearance_m": float(nd["clearance_m"]),
            "degree": int(nd["degree"]),
            "type": nd["type"],
        })
    out_edges = [{"a": int(a), "b": int(b), "length_m": float(ln)}
                 for (a, b, ln) in edges]
    path.write_text(json.dumps({"nodes": out_nodes, "edges": out_edges}, indent=2))


def overlay_nodes_into_world(world, nodes, vmin, marker_radius: int = 1) -> None:
    """Draw each place node as a (2r+1)^3 magenta emissive cube."""
    if not nodes:
        return
    extent = world.manifest.chunk_extent
    pal = world.palette
    new_mat_id = max(m.id for m in pal.materials) + 1
    if new_mat_id > 255:
        raise ValueError("palette full; cannot add place_node material")
    new_color_idx = len(pal.color_lut)
    pal.materials.append(vxw.Material(
        id=new_mat_id, name=PLACE_NODE_NAME, color_rgb=PLACE_NODE_COLOR,
        flags=("place", "emit"), emission_rgb=PLACE_NODE_COLOR,
        emission_energy=PLACE_NODE_EMISSION,
    ))
    pal.color_lut.append(PLACE_NODE_COLOR)
    r = marker_radius
    cells = []
    for nd in nodes:
        cx, cy, cz = nd["idx"]
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    cells.append((cx + dx, cy + dy, cz + dz))
    mc = np.array(cells, dtype=np.int64) + vmin
    cc = np.floor_divide(mc, extent)
    local = (mc - cc * extent).astype(np.uint8)
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

改 `run_gvd` 签名加 `graph: bool = False, merge_radius_m: float = 0.15`(在 `thin` 之后)。
在 `run_gvd` 体内,`if thin: gvd = thin_gvd(gvd)` 之后、`overlay_gvd_into_world(...)` 之前,插入:
```python
    nodes, edges = [], []
    if graph:
        from m3_adapter.gvd_graph import skeleton_to_graph
        nodes, edges = skeleton_to_graph(gvd, dist_m, vsize, merge_radius_m=merge_radius_m)
```
在 `overlay_gvd_into_world(world, gvd, vmin)` 之后插入:
```python
    if graph:
        overlay_nodes_into_world(world, nodes, vmin)
        _write_graph_json(
            Path(output_vxw).with_suffix(".graph.json"), nodes, edges, vmin, vsize
        )
```
stats dict 加:`"graph_nodes": len(nodes), "graph_edges": len(edges)`。
在 print 区加(thin 之后):
```python
    if graph:
        print(f"[gvd] places graph: {len(nodes)} nodes, {len(edges)} edges "
              f"-> {Path(output_vxw).with_suffix('.graph.json').name}")
```

main() 加 flag:
```python
    ap.add_argument("--graph", action="store_true",
                    help="sparsify the (thinned) skeleton into a places graph "
                         "(.graph.json + magenta node markers)")
    ap.add_argument("--merge-radius", type=float, default=0.15,
                    help="merge graph nodes within this many metres")
```
传参:`graph=args.graph, merge_radius_m=args.merge_radius`。

Run `pixi run pytest tests/ -q` → expect 61 passed.

- [ ] **Step 3: Commit**
```bash
git add m3_adapter/gvd_to_vxw.py tests/test_gvd.py
git commit -m "feat(gvd): --graph emits places graph JSON + node markers (SP-D task 2)"
```

---

### Task 3: 真实数据运行 + 渲染判读(控制器自跑)

**Files:** 无代码改动。

- [ ] **Step 1: 在 apt 骨架上抽图**
```
pixi run python m3_adapter/gvd_to_vxw.py out/uhumans2_apt_full.vxw out/apt_graph.vxw --seed 3.875,1.2,5.575 --band-max 1.0 --min-component 30 --thin --graph
```
记录:nodes / edges 数量(应几百量级,非几万),graph 耗时。检查 `out/apt_graph.graph.json` 合法。

- [ ] **Step 2: 渲染**
```
F:\Godot\...console.exe --path ...\godot_viewer --position -10000,-10000 --resolution 1280x720 --quit-after 200 -- --world=F:/slam-voxel-world/out/apt_graph.vxw --snapshot=F:/slam-voxel-world/out/apt_graph.png
```

- [ ] **Step 3: 判读(控制器看图)**:品红节点是否落在路口/房间/走廊交汇处的合理位置?节点数量级合理?→ SP-D v1 达成。记录结论到设计文档 §8,清理大产物(.vxw),保留 png 本地。
