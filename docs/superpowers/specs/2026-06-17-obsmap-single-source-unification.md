# 统一:ObsMap 单一真值源(退役批处理)

> 日期:2026-06-17。目标:让 ObsMap(持久增量 log-odds 地图)成为唯一真值源,游戏 .vxw 永远从它派生,退役批处理 `uhumans2_to_vxw.py`。

## 1. 动机
此前两条并存:批处理 `uhumans2_to_vxw`(全语义 .vxw + 实体 + spawn)vs ObsMap 派生 .vxw(几何/语义,无实体)。两份不统一。本次让 ObsMap 导出补齐实体+spawn,退役批处理。

## 2. 实现(commit 75f8eaf)
- `obsmap_export.obsmap_to_world(obsmap, label_names, palette, extract_entities_fn=, find_spawn_fn=)`:从 ObsMap 的 `occupancy_mask`+`semantic_grid` 建语义体素;**复用** `uhumans2_to_vxw.extract_entities`(DBSCAN→家具 OBB)和 `_find_spawn_hint`(注入避免循环 import);entity 体素从 grid 抠除(两层约定);material_id 经 palette.semantic_classes 的 default_material 映射。返回完整 World(语义 chunks + entities + spawn_hint)。
- `obsmap_to_vxw_full` + CLI `m3_adapter/obsmap_to_vxw.py <obsmap/vxwdir> <out.vxw> --hydra-cfg --scene`。
- `uhumans2_to_vxw.py` 顶部加 **[DEPRECATED for new use]** 标注,保留作参考实现 + 复用助手来源。
- 87 测试绿。

## 3. 真实数据验证
当前 obsmap(250 帧语义)→ `obsmap_to_vxw`:占据 57233 / 结构 55486 / **实体 17**(chair2/couch3/lamp8/table4)/ spawn_hint [4.325,1.25,4.475,176.3]。Godot 加载渲染:语义结构 + 17 家具实体(entity 层)+ avatar 落在 spawn 点(out/apt_unified.png)。**与批处理输出全功能对齐,全部源自 ObsMap。**

## 4. 新的单一真值流程
```
rosbag → uhumans2_stream(--semantic,可续建/增量)→ ObsMap(占据+自由+语义,唯一真值,持久)
   ├→ obsmap_to_vxw       → 完整游戏 .vxw(语义+实体+spawn),Godot 渲染
   └→ observed_free 派生   → GVD 子系统 → places/rooms 图
```
批处理退役(仅留助手 + 参考)。

## 5. 诚实的残留
- **保真度**:ObsMap 派生当前 5.5万体素(250 帧 stride4)< 批处理 43.4万(全帧)。追平需全帧细 stride 的长 carve(~小时级)。架构已统一,保真是跑量问题。
- **实体抽取的 `_OBJECT_LABELS` 硬编**(uHumans2 标签空间):注入式可换,换场景需注入对应抽取器。
- **dense 存储**:单公寓够用;城市级需换 voxel-hashing 后端(ObsMap 接口已抽象,局部可换)。
- 语义未聚成"物体层"的图节点(Objects 层仍缺,见下一步)。

## 6. 下一步候选
Objects 层(语义→图节点)| 全帧高保真 carve | voxel-hashing 后端 | Phase ④ 先验建图。
