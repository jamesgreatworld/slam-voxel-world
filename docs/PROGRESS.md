# 进展总结与未来展望(自研 Hydra + 增量语义建图)

> 日期:2026-06-17。本文是给后续会话的**交接文档**:当前进展 + 架构 + 未完成展望。
> 详细设计散在 `docs/superpowers/specs/2026-06-*.md`,本文是索引 + 总览。

## 0. 一句话现状
"自研 Hydra 式增量语义建图"主线**从设计到实时、到单一真值源、到完整场景图(含物体跟踪/去碎/模型替换)走完了**,远超最初 SP-A→SP-D 设想。最大未启动前沿 = **Phase ④ 先验引导建图**(docs/vision.md)。

## 1. 端到端数据流(现状)
```
RGB-D + 位姿 + 语义(uHumans2 rosbag,可换任何源)
   │  uhumans2_stream.py(--live / --incremental / --semantic,可存档/恢复/续建)
   ▼
ObsMap(m3_adapter/obsmap.py):持久 log-odds 观测地图
   通道:logodds(占据/自由/未知,一份网格三态)+ sem_label/sem_count(语义,Boyer-Moore)
   能力:save/load、增量增删改(log-odds 翻转=删除)、脏盒追踪
   │
   ├─ obsmap_to_vxw.py ──→ 完整游戏 .vxw(语义体素 + 实体[DBSCAN OBB] + spawn + mc_item 预制模型)
   │                         → Godot 渲染(greedy mesh);--watch 文件热重载(§12 P1/P2)
   └─ observed_free 派生 ──→ GVD 子系统(m3_adapter/gvd/)
          field(densify/esdf/gvd/thin/denoise/局部增量)→ graph(PlacesGraph+清洗)
          → rooms(Louvain)→ objects(语义聚类+链places)→ render/scene_graph
          → places/rooms/objects 拓扑图 + DSG
```

## 2. 已完成(模块 → 文件 → commit 锚点)
| 能力 | 文件 | 关键 commit |
|---|---|---|
| main.gd 拆 6 controller | godot_viewer/* | df86eef |
| 批式 GVD(ESDF→GVD→骨架→图→房间) | m3_adapter/gvd/* | 子包重构 342701e |
| **observed-free 射线碰撞(正道)根治泄漏** | uhumans2_carve.py + field | 8ee6ff5 (99.1%→5.9%) |
| **ObsMap 持久 log-odds(占据+自由+语义)** | obsmap.py | be92a94 / 2efc1a2 |
| 续建驱动(存档/恢复/增量) | uhumans2_stream.py | 0653a36 |
| v1 流式(--live + Godot --watch 热重载) | stream + world_session | 55a16b9 / 92e0140 |
| v2 增量 ESDF(局部重算 **52×**) | field.extract_gvd_local | d0a2d78 / f39b5b5 |
| v3 语义通道(Boyer-Moore) | obsmap.py | 2efc1a2 / c232dbb |
| **统一 ObsMap 单一真值源**(批处理退役) | obsmap_export.obsmap_to_world | 75f8eaf |
| Objects 层(语义聚类→链 places) | gvd/objects.py | 5c4e22a |
| **DSG**(Building>Room>Place/Object,CRUD+查询,持久化+增量合并) | gvd/scene_graph.py | 01d5fe8 / 43b5df0 |
| 物体跟踪(类内匈牙利 + 形状签名) | scene_graph.merge_observation | 6775a83 |
| 模块化特征框架(ObjectFeature 插件) | gvd/features.py | 2a3bd46 |
| 去碎(DBSCAN 容差 + bbox 邻近合并) | gvd/objects.py | 61900c0 |
| **物体类→mc_item 预制模型替换** | obsmap_export.SUPER_ID_TO_MC_ITEM | 9299115 |
| 时序物体合并(持久共现碎片→合一) | scene_graph.consolidate_fragments | 5092e0f |

settled GVD 命令:`--observed-free <npz> --band-max 0 --min-component 30 --thin --rooms --prune-spurs 0.3 --merge-close 0.2 --drop-small 5 --room-resolution 0.3 --objects [--scene-graph]`。
settled 建图:`uhumans2_stream <bag> <vxwdir> --semantic --hydra-cfg F:/hydra_ws`(rosbag 在 F:/hydra_ws/datasets/...)。

## 3. 未完成(展望,按价值排)

### 3.1 Phase ④ 先验引导建图(最大前沿,未启动)—— docs/vision.md
- ④a 平面约束(墙平整/垂直)| ④b 模板/尺寸先验(家具拟合标准模型,**mc_item 替换是入口,已做**)| ④c **物理验证回路**("桌应落地"重力/碰撞)| ④d 学习补全。

### 3.2 当前工作的直接延续
- **颜色/ORB/CNN 特征**:`gvd/features.py` 框架已支持,只实现 shape;颜色需给 ObsMap 加 RGB 通道(rosbag 有 RGB 主题)。
- **Hydra 膨胀式房间法**(剪 clearance 低的门口节点 → 连通分量)—— 可能比 Louvain 干净,place 已带 clearance_m。
- **真 Voxblox 波前 ESDF**(v2 是局部重算,非完整 raise/lower 队列)= v2.1。
- **全帧高保真 carve**(追平批处理 43.4 万体素;现 stride4/部分帧约 5.7 万)。
- **mc_item 缩放保真**:entity_renderer 可能按 OBB 缩放预制模型,DBSCAN 膨胀 OBB 会致变形 —— 建议改为"OBB 定位/朝向,模型用原生比例"。
- 更多 mc_item preset(couch/plant/books 现无模型 → 保留 OBB)。

### 3.3 架构储备 / 远期
- §12 **P3 WebSocket 流式协议**(现用文件热重载 = P1/P2)。
- **voxel-hashing 后端**(城市级;ObsMap 接口已抽象,局部可换)。
- Buildings 层聚合(多建筑)。

### 3.4 遗留小债
- `_editor_selftest` 既有失败(task,重构前就失败,Godot/驱动 readback 环境问题,待查)。
- 回 Godot 玩法层(从最早搁置)。

## 4. 关键技术决策(供新会话沿用)
- ObsMap = OctoMap log-odds 模型(非原创),**dense numpy 3D 数组**(单公寓够用,城市级换 hash);占据/自由是同一 log-odds 数的两阈值视图(构造上一致)。
- 双层数据:`.vxw`(游戏渲染,稀疏占据)+ ObsMap/sidecar(建图工作态)= "一份逻辑图、两通道"(vision.md §4)。
- 物体破碎根因 = 语义噪声 + 几何空洞(非 mesh/voxel 之别);解法 = 去碎①② + 时序合并 + **预制模型替换(治本于显示层)**。
- 追踪 = SORT→DeepSORT 思路;我们的物体静止为主,**不需 Kalman 位姿预测**,靠类内匈牙利 + 外观特征。
