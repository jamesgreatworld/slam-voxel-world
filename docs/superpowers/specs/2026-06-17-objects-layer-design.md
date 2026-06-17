# 场景图 Objects 层设计

> 日期:2026-06-17。补全 Hydra 场景图最后一层:语义体素 → 物体实例 → 链到 places 节点(包含关系)。

## 1. 实现(commit 5c4e22a)
- `m3_adapter/gvd/objects.py`:`ObjectNode`(centroid/label/bbox/voxel_count/place_id);`extract_objects(occ, sem, ...)` 对**非结构类**(默认排除 floor=3/ceiling=4/wall=19)逐类做 26 连通分量(scipy.ndimage.label),每分量 ≥min_voxels → 一个 ObjectNode;`link_to_places(objects, places_graph)` 按质心世界米距离链到最近 place 节点。
- `field.densify_semantic(world)`:返回 (occ, sem uint8, vmin),从 .vxw chunk 的 semantic_id 填语义网格。
- pipeline `--objects`(+`--object-min-voxels`):graph/rooms 后抽 objects + 链 places,写进 graph.json 的 `objects` 数组(含 place_id)+ `stamp_object_markers`(橙色 5³ 标记)。
- 89 测试绿。

## 2. 真实数据验证
apt(全语义 434K)`--objects --object-min-voxels 30`:**188 物体实例**,各链到 place(by class super_id:books40/plant25/lamp23/chair21/table16…)。场景图三层齐:355 places + 15 rooms + 188 objects(out/apt_obj.png 橙标记)。

## 3. 诚实残留
- **188 过碎**:含噪语义分割把一个真实物体拆成多个连通分量(books 40 个尤甚)。降噪需:同类邻近物体合并 / 提 min_voxels / 先对 sem 做形态学。机制对,数量待清洗。
- **label_name 是数字**:pipeline 传了空 label_names。接 hydra 标签空间(load_label_space)即得人话名,小 wiring。
- **结构 vs 物体的硬编**:structure_labels 默认 {3,4,19},可配置。

## 4. 场景图现状(Hydra 完整)
```
Rooms(Louvain 社区)
  └ Places(GVD 骨架节点,自由空间)
      └ Objects(语义聚类实例,链到最近 place)   ← 本次新增
结构层(墙/地/天花)= 度量-语义体素(不进物体)
```
graph.json 现含:nodes(places,带 room)+ edges + objects(带 place_id)+ num_rooms。

## 5. 下一步候选
物体去碎(同类合并)+ 接 label 名 | 物体语义先验(Phase ④,如"桌应落地")| Buildings 层(多房聚合)。
