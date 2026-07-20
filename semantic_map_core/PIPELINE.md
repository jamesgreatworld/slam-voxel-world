# 自研 Hydra 增量构建流程(semantic_map_core)

从 RGB-D 输入到增量语义场景图,全 C++(`smc_live_node`),与 Python 参考实现逐模块零误差。
实测(House 完整数据集, VM 2核):积分 8ms/帧、场景图 cycle 181ms、峰值 RSS 111MB、原速零丢帧。

## 一、数据流总览

```
传感器/bag                     lightning SLAM
 /tg/depth  32FC1 ─┐            (雷达+IMU)
 /tg/semantic mono8 ├─ 时间同步     │
 /tg/rgb    rgb8   ─┘   │       TF: odom→base(100Hz IMU predict)
 /tg/camera_info ────── 内参        + base→cam_optical(静态外参)
                        ▼            │
              ┌─ 每帧(积分线程, ~8ms) ◄─ TF 查询(odom←cam_optical, 5s 重试队列)
              │  unproject(stride6, 0.2-15m) → 光学系点云
              │  → T_odom_cam 变换 → ROS z-up → vxw y-up
              │  → ObsMap.integrate_frame(射线 free 雕刻 + 命中 log-odds + 语义 Boyer-Moore 投票)
              │        【地图层增量: 每帧只改射线扫过的格】
              │
              └─ 周期(场景图线程, 每 10s, ~181ms)
                 occupancy/free/sem 快照 → 紧致裁剪(occ bbox pad1)
                 → denoise(26连通≥30) → ESDF(Felzenszwalb) → GVD(父体素间距) → thin(Lee 细化)
                 → skeleton_to_graph(骨架→places 图)
                 → prune_spurs(0.3m) → merge_close(0.2m) → drop_small_components(5)
                 → partition_rooms_clearance(门口 0.85m 切房) → merge_nested_rooms
                 → extract_objects(逐类 DBSCAN + bbox 合并 + shape 特征) → link_to_places
                 → surface_tiles(2D 可通行层)
                 → 【场景图层增量: merge_observation 与上一周期融合, 物体身份保持】
                 → 发布 /dsg MarkerArray + dump json/txt
```

## 二、三层空间结构(DSG)

```
building:0                     ← 根(1 个)
 └─ room:{r}                   ← 房间: clearance 切门 + 嵌套合并, 位置=成员 places 质心
     ├─ place:{i}              ← 骨架节点(junction/endpoint), 带 clearance/degree
     └─ object:{id}            ← 物体: 支撑父子优先(cup-on-table), 否则挂最近 place 的房间
         └─ object:{id2}       ← 被支撑物体(bbox y-up 几何判定)
```
另有与三层并行的 **surface places(2D 可通行层)**,供导航代价图。

## 三、两级增量语义

| 层 | 机制 | 粒度 |
|---|---|---|
| 地图层(ObsMap) | 每帧射线积分, log-odds 命中/穿透, 语义逐格投票 | 帧级增量(~8ms) |
| 场景图层(DSG) | 周期全图重算(170-250ms)+ `merge_observation` 跨周期融合 | 周期级增量, 物体身份跨周期稳定 |

室内规模下"周期全图重算"已实时(cycle 181ms ≪ 10s 间隔可压到 1s 内);
真 dirty-box 窗口化重算留给大场景(见 OPTIMIZATION.md)。

## 四、物体增删改查(CRUD)语义与验证

`merge_observation`(类内匈牙利匹配, 代价=质心距离+2×shape 特征距离, 门限 0.5m):

- **增(add)**:新物体匹配不上任何已有物体 → 分配新稳定 id 入图。
- **删(delete)**:已有物体连续 `max_misses=3` 个周期未被观测 → 从图中移除。
- **改(update)**:匹配成功 → **沿用原 id**,位置/bbox/voxel_count 更新为新观测,seen_count+1,misses 清零;
  未观测但未超限 → carry-over 保留(misses+1, 挂回原房间)。`consolidate_fragments` 另可合并同物碎片。
- **查(query)**:DSG 支持按 id / 按层(nodes_by_layer)/ 按父子(children/ancestors)查询;
  每周期 dump json(`scene_graph.json` / `cpp_dsg.txt`)。

**验证结果**:
1. **离线确定性验证**(bench_online, 与 Python 逐节点零误差):House 41 物体,构造"删 2 个+移 1 个+预置 misses=3"
   场景 → matched=38(改)/ added=1(增)/ removed=1(删)/ carried=2(保留),975 节点 0 字段差异。
2. **在线实测**(smc_live_node, 完整 house_sim_bag):5 个周期累计 matched=118 / added=44 / removed=1 /
   carried=16 —— 四种操作全部在真实在线流中发生;最终 DSG 9 房/280 places/49 物体。

## 五、与原版 MIT Hydra 的实测对比(House 356帧/35.6s, VM 2核)

| | MIT Hydra | 自研 Python | **自研 C++(实测)** |
|---|---|---|---|
| 跑完全程 | 281s, 丢约半数关键帧 | 40s 跟播, 积分 100/356 | **41s(=数据原速), 积分 372, 丢 3** |
| 每帧成本 | frontend 637ms/kf | ~0.4s/帧 | **8.0ms/帧(max 31ms)** |
| 场景图更新 | 增量 386-637ms/kf | cycle 0.6-0.9s | **cycle 181ms(max 253ms)** |
| 峰值 RSS | 2979 MB | 488 MB | **111 MB** |
| DSG | rooms 未成形 | 9房/255pl/36obj | **9房/280pl/49obj** |
