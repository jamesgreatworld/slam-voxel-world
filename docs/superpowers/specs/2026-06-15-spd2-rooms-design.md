# SP-D2 Rooms 设计+计划(places 图 → 房间)

> 日期:2026-06-15。上游 = SP-D 的 places 图(节点+边)。目标:用社区检测把 places 图分成房间(Hydra Rooms 层),按房间给节点染色可视。
> 输入图当前较密(3486 节点),房间质量部分反映该噪声;社区检测按图结构聚类,毛刺多附着在所属区域,房间或比节点数更可读。本身也是对"要不要先剪枝/上正道"的检验。

## 1. 算法
networkx Louvain(`louvain_communities`,已随 scikit-image 装,networkx 3.6.1)。
- 建图:节点 = places 节点;边带权 `w=1/max(length_m, ε)`(短边=房内强连接,长边/门口=弱)。
- `louvain_communities(G, weight="weight", resolution, seed=0)` → 社区。
- 每节点 room_id = 所属社区(按社区大小降序编号,room 0 最大)。孤立节点各自成房。

## 2. 模块
| 文件 | 职责 |
|---|---|
| `m3_adapter/gvd_rooms.py` | 纯:`partition_rooms(num_nodes, edges, resolution=1.0, seed=0) -> list[int]`(room id per node)。无 IO。 |
| `m3_adapter/gvd_to_vxw.py`(扩展) | `--rooms`(隐含 --graph):分房间 → JSON 节点加 `room` + 顶层 `num_rooms` → 节点标记**按房间染色** |

## 3. 可视化
12 色循环调色板,节点 marker 按 `room_id % 12` 染发光色(骨架仍青)。新增 12 个 `room_k` 材质。

## 4. 实现(完整代码,实现者照抄)

### 4.1 `m3_adapter/gvd_rooms.py`
```python
"""gvd_rooms.py — partition a places graph into rooms via Louvain community
detection (SP-D2). Pure: graph (counts+edges) in, room labels out. No I/O."""
from __future__ import annotations


def partition_rooms(num_nodes: int, edges, resolution: float = 1.0, seed: int = 0):
    """Return a room id per node (list length num_nodes).

    edges: iterable of (a, b, length_m). Edge weight = 1/length (short edges
    bind a room; long bottleneck edges = doorways are weak). Communities are
    numbered by descending size (room 0 is the largest). Isolated nodes each
    get their own room.
    """
    import networkx as nx
    from networkx.algorithms.community import louvain_communities

    g = nx.Graph()
    g.add_nodes_from(range(num_nodes))
    for a, b, length_m in edges:
        g.add_edge(int(a), int(b), weight=1.0 / max(float(length_m), 1e-3))
    if g.number_of_edges() == 0:
        return list(range(num_nodes))
    comms = louvain_communities(g, weight="weight", resolution=resolution, seed=seed)
    room_of = [0] * num_nodes
    for rid, comm in enumerate(sorted(comms, key=lambda c: -len(c))):
        for n in comm:
            room_of[int(n)] = rid
    return room_of
```
Tests `tests/test_gvd_rooms.py`:
```python
from m3_adapter.gvd_rooms import partition_rooms


def test_two_cliques_split_at_weak_bridge():
    edges = [(0,1,1.0),(0,2,1.0),(1,2,1.0),
             (3,4,1.0),(3,5,1.0),(4,5,1.0),
             (2,3,8.0)]  # long, weak bridge
    rooms = partition_rooms(6, edges, resolution=1.0)
    assert rooms[0] == rooms[1] == rooms[2]
    assert rooms[3] == rooms[4] == rooms[5]
    assert rooms[0] != rooms[3]
    assert len(set(rooms)) == 2


def test_isolated_nodes_each_own_room():
    rooms = partition_rooms(4, [(0,1,1.0)])  # 2,3 isolated
    assert rooms[0] == rooms[1]
    assert len(set(rooms)) == 3


def test_no_edges_all_singletons():
    assert len(set(partition_rooms(3, []))) == 3
```

### 4.2 `gvd_to_vxw.py` 扩展
常量(PLACE_NODE_* 附近):
```python
ROOM_COLORS = [
    (255, 80, 80), (80, 255, 80), (80, 80, 255), (255, 255, 80),
    (255, 80, 255), (80, 255, 255), (255, 160, 40), (160, 80, 255),
    (40, 255, 160), (255, 120, 160), (160, 255, 80), (120, 160, 255),
]
ROOM_EMISSION = 4.0
```
新 helper(overlay_nodes_into_world 之后):
```python
def overlay_room_nodes_into_world(world, nodes, room_of, vmin, marker_radius: int = 1):
    """Draw each place node as a (2r+1)^3 cube coloured by room (room_id % 12)."""
    if not nodes:
        return
    extent = world.manifest.chunk_extent
    pal = world.palette
    base_mat = max(m.id for m in pal.materials) + 1
    k = len(ROOM_COLORS)
    if base_mat + k - 1 > 255:
        raise ValueError("palette can't fit room materials")
    base_col = len(pal.color_lut)
    for j, col in enumerate(ROOM_COLORS):
        pal.materials.append(vxw.Material(
            id=base_mat + j, name=f"room_{j}", color_rgb=col,
            flags=("room", "emit"), emission_rgb=col, emission_energy=ROOM_EMISSION))
        pal.color_lut.append(col)
    r = marker_radius
    coords, mats, cols = [], [], []
    for nd, rid in zip(nodes, room_of):
        j = int(rid) % k
        cx, cy, cz = nd["idx"]
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    coords.append((cx + dx, cy + dy, cz + dz))
                    mats.append(base_mat + j)
                    cols.append(base_col + j)
    mc = np.array(coords, dtype=np.int64) + vmin
    mats = np.array(mats, dtype=np.uint8)
    cols = np.array(cols, dtype=np.uint8)
    cc = np.floor_divide(mc, extent)
    local = (mc - cc * extent).astype(np.uint8)
    for ck in np.unique(cc, axis=0):
        m = np.all(cc == ck, axis=1)
        ckey = tuple(int(x) for x in ck)
        if ckey in world.chunks:
            arr = world.chunks[ckey].voxels
        else:
            arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
            world.chunks[ckey] = vxw.Chunk(coord=ckey, voxels=arr,
                encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP)
        loc = local[m]
        arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = mats[m]
        arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 0
        arr["color_palette_idx"][loc[:, 0], loc[:, 1], loc[:, 2]] = cols[m]
    keys = np.array(list(world.chunks.keys()))
    world.manifest.bounds_chunks_min = tuple(int(x) for x in keys.min(axis=0))
    world.manifest.bounds_chunks_max = tuple(int(x) + 1 for x in keys.max(axis=0))
```
`run_gvd`:签名加 `rooms: bool = False, room_resolution: float = 1.0`。`if rooms: graph = True`(开头规整)。graph 分支里 skeleton_to_graph 后:
```python
    room_of = []
    if rooms:
        from m3_adapter.gvd_rooms import partition_rooms
        room_of = partition_rooms(len(nodes), edges, resolution=room_resolution)
```
overlay 选择(替换原 `overlay_nodes_into_world` 调用):
```python
    if graph:
        if rooms:
            overlay_room_nodes_into_world(world, nodes, room_of, vmin)
        else:
            overlay_nodes_into_world(world, nodes, vmin)
        _write_graph_json(Path(output_vxw).with_suffix(".graph.json"),
                          nodes, edges, vmin, vsize, room_of=room_of)
```
`_write_graph_json` 加可选 `room_of=None`:每节点写 `"room": int(room_of[i])`(若提供);顶层加 `"num_rooms": len(set(room_of)) if room_of else 0`。
stats 加 `"num_rooms": len(set(room_of)) if room_of else 0`。print 加 `if rooms: print(f"[gvd] rooms: {stats['num_rooms']} communities")`。
main() 加 `--rooms`(store_true)、`--room-resolution`(float, default 1.0);传 `rooms=args.rooms, room_resolution=args.room_resolution`。

## 5. 验证
- `tests/test_gvd_rooms.py` 3 个(双团/孤立/无边)。
- 集成:run_gvd(..., rooms=True) → JSON 有 room + num_rooms,palette 有 room_0。
- 真实:apt `--rooms` 跑通,num_rooms 合理(几个~几十),渲染按房间染色。
- 全套测试绿(预期 64+1 集成)。

## 6. 完成判定
按房间染色的节点叠在骨架上、Godot 可见分块 → SP-D2 v1 达成。房间数与分块合理性是对"图够不够干净"的进一步检验。
