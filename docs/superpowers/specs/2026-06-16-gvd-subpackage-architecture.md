# GVD 子系统架构重构 + 图清洗设计

> 日期:2026-06-16。动机:GVD→图→房间这条路要长期大优化/重构,当前 `gvd_to_vxw.py`(451 行,run_gvd 13 参,3 个 ~110 行重复 overlay)会腐烂。
> 目标:收进子包 `m3_adapter/gvd/`,按阶段分模块,把 **PlacesGraph 升为一等数据类 + 清洗作为可组合变换**,顺带落地本次图清洗(prune_spurs / merge_close)。CLI 调用不变。

## 1. 模块结构(子包)

```
m3_adapter/
  gvd_to_vxw.py            瘦 CLI 壳:argparse → GvdConfig → gvd.pipeline.run(cfg)
  gvd/
    __init__.py            再导出公开 API(field/graph/rooms/render/pipeline 的公共名)
    field.py              体素场(并入原 voxel_gvd + densify/seed)
    graph.py              PlacesGraph/PlaceNode + skeleton_to_graph + 清洗变换
    rooms.py              partition_rooms
    render.py             stamp_voxels 原语 + skeleton/nodes/rooms 封装 + write_graph_json
    pipeline.py           GvdConfig + run(cfg) 分阶段编排
```

**依赖方向(单向)**:pipeline → {field, graph, rooms, render};render → field(读 vmin/voxel)?不,render 只吃 world + PlacesGraph + cells,不依赖 field。graph 依赖 field?不——graph 吃 numpy 数组(gvd, dist_m)出 PlacesGraph,不 import field。各阶段模块**互不横向 import**,只被 pipeline 编排。这是"不乱"的根。

## 2. 数据契约

### 2.1 field.py(纯数组,签名与原 voxel_gvd 一致,新增 densify/seed)
```
densify_occupancy(world, pad=1) -> (occ_bool, vmin)
resolve_seed(occ, vmin, voxel_size, seed_metres=None, spawn_hint=None) -> idx
flood_free_space(occ, seed) -> free_bool
compute_esdf(occ, voxel_size) -> (dist_m, parent)
extract_gvd(free, dist_m, parent, voxel_size, d_min, theta_sep) -> gvd_bool
denoise_occupancy(occ, min_component_size, connectivity=3) -> occ_bool
thin_gvd(gvd) -> gvd_bool
```
（行为全部不变,纯搬家 + densify/seed 从 gvd_to_vxw 并入。）

### 2.2 graph.py —— PlacesGraph 一等数据类 + 变换
```python
@dataclass
class PlaceNode:
    idx: tuple[int, int, int]   # dense voxel index
    clearance_m: float
    degree: int                 # GRAPH degree (rebuilt by transforms)
    type: str                   # junction|endpoint|passthrough|isolated|loop
    room: int = -1              # -1 = unassigned (rooms.py 填)

@dataclass
class PlacesGraph:
    nodes: list[PlaceNode]
    edges: list[tuple[int, int, float]]   # (a_id, b_id, length_m), a<b, deduped
    voxel_size: float
    vmin: np.ndarray
    def positions_m(self) -> np.ndarray            # (N,3) world metres = (idx+vmin)*voxel_size
    def adjacency(self) -> dict[int, dict[int, float]]   # id -> {nbr: length}
```
构造:
```
skeleton_to_graph(gvd, dist_m, voxel_size, vmin, merge_radius_m=0.15) -> PlacesGraph
```
（原逻辑不变;现在直接产 PlacesGraph,且接收 vmin 以便 positions_m。degree=节点初始图度数。）

清洗变换(**全部 `PlacesGraph -> PlacesGraph`,纯函数,renumber 一致**):
```
prune_spurs(g, max_len_m) -> PlacesGraph
    迭代删除"图度数=1 且其唯一边 < max_len_m"的端点(骨架毛刺)。剪后可能
    暴露新短端点,循环到稳定。重建邻接、renumber、重算每节点 degree/type。
    长边连的端点(真走廊端)保留。

merge_close(g, radius_m) -> PlacesGraph
    位置 < radius_m 的节点收缩为一个(质心,clearance 取 max),重连边(自环丢弃,
    重复边取最短)。塌掉 DBSCAN 残留的近邻簇。renumber、重算 degree/type。
```
辅助:`_recompute_types(nodes, edges)` 按图度数刷新 degree/type(junction≥3 / passthrough==2 / endpoint==1 / isolated==0)。

### 2.3 rooms.py(吃 PlacesGraph,纯)
```
partition_rooms(graph, resolution=1.0, seed=0) -> list[int]   # room id per node
```
（Louvain;weight=1/length;按社区大小降序编号;无边→各自单房。返回标签列表,pipeline 写回 node.room,**rooms.py 不改 graph**。）

### 2.4 render.py(消重复)
```
stamp_voxels(world, cells_world, name, color_rgb, emission=3.0) -> int(mat_id)
    单一原语:加发光材质 + 把 Nx3 世界体素写进 chunks(建缺失 chunk)+ 重算 bounds。
stamp_skeleton(world, gvd, vmin)                  # 青 gvd_skeleton
stamp_graph_nodes(world, graph, marker_radius=1)  # 品红 place_node(无房间时)
stamp_room_nodes(world, graph, marker_radius=1)   # 12 色按 node.room 染
write_graph_json(path, graph)                     # 节点含 pos_m/clearance/degree/type/room,
                                                  #   顶层 num_rooms(room≥0 时)
```
3 个 stamp_* 都走 stamp_voxels(room 版每色一次 stamp)。原 overlay_* 的 ~110 行重复消除。

### 2.5 pipeline.py(干掉 flag-pile)
```python
@dataclass
class GvdConfig:
    input_vxw: str; output_vxw: str
    seed_metres: tuple | None = None
    # field
    pad: int = 1; band_max: float | None = 1.0; min_component: int = 0
    d_min: float = 0.20; theta_sep: float = 0.40; thin: bool = False
    # graph + 清洗
    graph: bool = False; merge_radius_m: float = 0.15
    prune_spurs_m: float = 0.0      # 0 = off
    merge_close_m: float = 0.0      # 0 = off
    # rooms
    rooms: bool = False; room_resolution: float = 1.0

def run(cfg) -> dict   # stats
```
**run 阶段顺序**(denoise 必须在 seed/flood/esdf 前,因它改障碍集):
```
load world → densify → [denoise] → seed → flood → esdf → [band clip] → extract_gvd → [thin]
→ if graph: skeleton_to_graph → [prune_spurs] → [merge_close]
→ if rooms(隐含 graph): labels=partition_rooms; 写回 node.room
→ render: stamp_skeleton; if graph: stamp_room_nodes(有房) or stamp_graph_nodes; write_graph_json
→ write_world → stats(含各阶段计数 + graph_nodes/edges/num_rooms + 清洗前后节点数)
```

## 3. 本次清洗的实际效果目标
apt(band1.0+denoise30+thin)当前 3486 节点 / 234 房。加 `--prune-spurs 0.30 --merge-close 0.20 --room-resolution 0.3`:
预期节点数量级直降(剪掉上千端点毛刺),房间数落到十几~几十。**这是对架构的验证,也是对"清洗能否救活图"的判定**。

## 4. 迁移(测试)
- `tests/test_gvd.py`(voxel/管线)→ 拆成 `test_gvd_field.py`(field 单测,原 voxel_gvd 测试搬过来改 import)+ `test_gvd_pipeline.py`(管线集成,run_gvd→pipeline.run(GvdConfig))。
- `test_gvd_graph.py` → import 改 `from m3_adapter.gvd.graph import ...`,加 prune_spurs/merge_close 测试。
- `test_gvd_rooms.py` → import 改 `from m3_adapter.gvd.rooms import ...`,partition_rooms 改吃 PlacesGraph(测试构造小 PlacesGraph)。
- 旧 `voxel_gvd.py` / `gvd_graph.py` / `gvd_rooms.py` 删除(内容已迁入子包)。
- CLI 调用与 settled 命令不变,新增 `--prune-spurs` / `--merge-close`。

## 5. 验证
- 各模块单测绿;管线集成测试绿(合成盒 → 跑通图+房间)。
- prune_spurs 测试:带短毛刺的合成图 → 毛刺端点被剪,主干保留。
- merge_close 测试:两个 < radius 的节点 → 合一,边重连。
- 真实:apt `--prune-spurs 0.3 --merge-close 0.2 --room-resolution 0.3` 跑通,节点/房数量级合理,渲染按房染色更干净。

## 6. 范围外(未来变换,架构已为其留位)
contract_degree2(链收缩)、loop 处理、语义注入节点、增量更新——都是未来的 `PlacesGraph->PlacesGraph` 变换或 pipeline 阶段,不在本次。
**最重要的留位**:**observed-free(正道 TSDF ray-carve)= field 阶段的一个新自由空间来源**,替换 `flood_free_space`,下游 esdf→gvd→graph→清洗→rooms 全不动。rosbag 已确认在 `F:\hydra_ws\datasets\uhumans2\apartment_scene\...`。

## 7. 重构 + 清洗结果(2026-06-16)

**重构完成 ✅**:`gvd_to_vxw.py` 451 行(13 参 god-function)→ 74 行 CLI 壳 + `gvd/` 子包(field 200 / graph 252 / rooms 28 / render 122 / pipeline 122)。5 个阶段模块互不横向 import,清洗=可组合 `PlacesGraph→PlacesGraph` 变换。63 测试绿。

**清洗 pass 生效(prune_spurs + merge_close + Louvain resolution):**
| 配置 | 节点 | 房间 | 视觉 |
|---|---|---|---|
| 无清洗(原) | 3486 | 234 | 密 |
| 保守 prune0.3/merge0.2/res0.3 | 2587 | 179 | 略疏 |
| 激进 prune1.0/merge0.5/res0.1 | **714** | **124** | 明显干净(out/apt_clean2.png) |

**判读:清洗有效(激进降 80% 节点),架构干净可组合 ✅。但 124 房仍过分割(应 ~10)——清洗有天花板,因骨架噪声是结构性的、不只是毛刺,再剪会吃真结构。** 第四次确认:**根治在 observed-free(正道),已被 rosbag 解锁,且架构已为其留好 field 阶段插槽。**
