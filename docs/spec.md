# SLAM → 体素可交互世界：模块化 Spec

> **状态**：架构 Spec / 可行性分析（非 TDD 实现计划）。SLAM 模块由你自己处理，本文档锁定 **下游 5 个模块的契约**，特别是 **地图数据接口**，让你可以独立替换、并行扩展任何一个模块。
> **日期**：2026-05-22
> **目标读者**：你（项目作者）+ 后续接手的工程师

---

## 0. 目标与非目标

**目标**
- 把 SLAM 实时（或离线）产出的稠密占据/语义信息，转换成 **可被游戏引擎加载、玩家可交互、可破坏** 的体素世界。
- 6 个模块松耦合，**用稳定的数据接口（不是函数调用）** 解耦，便于你独立替换任何一个模块（例如把 ORB-SLAM3 换成 nvblox，把 Godot 换成 Unreal）。
- 单人/小团队 12 个月可以达到 "扫描一个房间 → 在引擎里走动并破坏" 的 MVP。

**非目标**
- 不复刻 GTA5 级别的玩法（NPC、车辆物理、任务系统、网络）。这些是 MVP 跑通后才考虑的扩展。
- 不自研渲染引擎。直接选 Godot / Unreal / Bevy。
- 不实现"实时 SLAM + 实时世界编辑"闭环（这是高级 stretch goal）。MVP 是 **离线生成 → 在线加载**。

---

## 1. 可行性证据（每个模块都有开源前例）

下表回答"这事能不能做"。结论：**每一层都有现成开源项目可以站着抄**。

| 阶段 | 关键开源项目 | 关键证据 |
|---|---|---|
| 点云/扫描 → 体素 | **Cloud2Craft** (AntoineMiras) | 把 LAS/LAZ 点云直接灌进 Minecraft。证明像素粒度的"现实→体素"管线在 2022 年就跑通 |
| 真实地理 → 体素世界 | **VoxelEarth** (ryanhlewis) | Google 3D Tiles → Minecraft chunks，含玩家位置→世界 chunk 流式加载 |
| 室内扫描 → 语义体素 | **World2Minecraft** (arXiv 2604.27578, 2026) | 3D 语义占据预测 → Minecraft 指令；带 VLN 任务，是最接近你目标的工作 |
| 城市级 LiDAR → 体素 | **CityMinecraft** (andreiBe) | Turku 市 LiDAR + OSM + 航拍 → Minecraft 世界，多源融合的工程范本 |
| 网格 → 体素 | **Scan2Craft** (y3jian) | GLB/OBJ/PLY → .litematic，含 CIE-LAB 颜色到方块的映射 |
| GPU TSDF 重建 | **nvblox** (NVIDIA-Isaac) | TSDF/ESDF/Color/Mesh 多层共形栅格的接口设计，是工业级参考 |
| CPU TSDF 重建 | **Voxblox** (ETH-ASL) | TSDF + Protobuf 序列化的层级数据格式 |
| 稀疏体素存储 | **OpenVDB** / **NanoVDB** | 工业标准（VFX 奥斯卡技术奖），稀疏体素之王 |
| Rust SVO-DAG | **voxelis** (WildPixelGames) | 4cm 体素 + DAG 压缩 99.999%，含 Bevy/Godot/C++ 绑定，最现代的核心库 |
| Godot 体素地形 | **Zylann/godot_voxel** | Godot 4 的 C++ 模块，含 Transvoxel LOD、blocky/smooth 双模式、流式 chunk |
| Unreal OpenVDB | **eidosmontreal/unreal-vdb** | UE5 直接读 OpenVDB/NanoVDB，转 Texture3D |
| Unreal SVO 寻路 | **darbycostello/Nav3D** v2.0 | SVO 3D 寻路（Daniel Brewer 算法），AI 可在体素空间飞行 |
| 体素光追渲染 | **Teardown** 引擎技术拆解 | 单 GPU 卡可实时光追 + 全可破坏，确立"千倍 Minecraft 密度"的上限 |
| 高保真重建 | **Scaniverse / Polycam + 3DGS** | 手机扫描→ Unity/Unreal 工作流，含坐标系/比例修正细节 |
| 数据格式 | **MagicaVoxel .vox**（ephtracy） | 工业事实标准，Chunked 二进制，已被 Teardown 使用 |

**结论**：技术风险低。最大的风险不是"能不能做"，而是"能不能粘合"。所以本 Spec 把绝大多数篇幅花在**模块边界（数据接口）**上。

---

## 2. 模块拆分总览

```
┌──────────┐   PointCloud/   ┌──────────┐    VoxelGrid    ┌──────────┐
│ M1: 传感 │──Depth+Pose────►│ M2: SLAM │────(+Color/────►│ M3: 地图 │
│ 采集层   │  RGB/IMU/LiDAR  │ & 重建   │     Semantics)  │ 构造器   │
└──────────┘                 │ (你处理) │                 │          │
                             └──────────┘                 └────┬─────┘
                                                               │
                                                               │ Voxel World
                                                               │ Container
                                                               │ (本 Spec 核心)
                                                               ▼
┌──────────┐   GameEvent     ┌──────────┐    ChunkMesh    ┌──────────┐
│ M6: 玩法 │◄──(damage,─────┤ M5: 引擎 │◄────────(or─────┤ M4: 渲染 │
│ /交互层  │   pickup, etc)  │ 物理/AI  │     SVO ray-    │ 适配器   │
└──────────┘                 └──────────┘     march)      └──────────┘
```

**6 个模块的职责一行版**：

- **M1 传感采集**：把现实世界的图像/深度/IMU 喂给 SLAM。可被替换为：iPhone ARKit / RealSense / 已有的数据集回放。
- **M2 SLAM & 重建**：你自己处理。**只需吐出 §4 定义的 `VoxelWorldContainer`** 即可，本 Spec 不管内部用什么算法。
- **M3 地图构造器**：把 SLAM 原始体素流转成"游戏世界"的体素 — 命名空间化、chunk 切片、语义→材质映射、坐标系/比例修正。
- **M4 渲染适配器**：把体素世界喂给具体引擎。两种实现路线（greedy mesh / SVO raymarch）。
- **M5 引擎集成层**：物理 / 角色控制 / 碰撞 / AI 寻路。包一层"引擎无关 API"。
- **M6 玩法 / 交互层**：玩家输入、破坏事件、可拾取物等游戏逻辑。完全在引擎脚本层。

**为什么这样分？**
- M2 和 M3 之间的边界（地图数据接口）是 **整个项目唯一不能改的接口**。其他所有模块都可以独立替换。
- M4 与 M5 分离：因为"如何把体素变成可见的像素"和"如何让玩家撞到墙不能穿过去"是不同的难题，往往用不同的数据结构。

---

## 3. 顶层数据流（4 阶段）

```
[Stage A: 扫描期]        [Stage B: 构建期]       [Stage C: 烘焙期]       [Stage D: 运行期]
                                                                          
M1 → M2                  M2 → M3                 M3 → 落盘                M4/M5/M6 加载
                                                                          
传感器数据流             实时或离线重建          一次性持久化             流式 chunk 加载
↓                        ↓                       ↓                        ↓
*.rosbag / *.mp4         VoxelWorldContainer     world.vxw 文件目录       玩家在游戏里走动
                         (内存中)                (本 Spec §4 定义)
```

- **Stage A → C 可以离线，完全脱机生成**，这是 MVP 路线
- **Stage A → D 在线** 是高级路线（"SLAM 实时改世界"），不在 MVP 范围

---

## 4. ★ 地图数据接口 `VoxelWorldContainer`（最重要的章节）

这是你 SLAM 模块（M2）唯一需要产出的东西。后面所有模块都只依赖这个接口，**不依赖你 SLAM 的实现**。

### 4.1 整体文件布局（落盘形式）

一个 "world" 是一个目录，不是单个文件。这是 nvblox/Voxblox/OpenVDB 通用做法：

```
world.vxw/
├── manifest.json          # 世界元数据（必须）
├── palette.json           # 材质 / 语义类别 / 颜色调色板（必须）
├── chunks/                # 体素 chunk 数据（必须）
│   ├── 0_0_0.chunk
│   ├── 0_0_1.chunk
│   └── ...
├── chunks.idx             # chunk 索引（chunk 坐标 → 文件偏移）
├── trajectory.json        # SLAM 轨迹（可选，调试用）
└── semantics/             # 语义辅助数据（可选）
    └── instances.json     # 语义实例列表（每个"门/桌子"是一个实例）
```

### 4.2 manifest.json — 世界根元数据

```json
{
  "format_version": "1.0",
  "world_id": "uuid-v4-string",
  "created_at": "2026-05-22T10:00:00Z",
  "source": {
    "slam_system": "ORB-SLAM3" ,
    "sensor": "RealSense D455",
    "raw_data_hash": "sha256:..."
  },
  "coord_system": {
    "convention": "right_handed_y_up",
    "world_origin_in_slam_frame": [0.0, 0.0, 0.0],
    "world_up_axis": [0, 1, 0],
    "scene_scale_meters_per_unit": 1.0
  },
  "voxel": {
    "size_meters": 0.05,
    "chunk_extent_voxels": [32, 32, 32]
  },
  "bounds_chunks": {
    "min": [-10, -2, -10],
    "max": [ 10,  4,  10]
  },
  "lod_levels": 3
}
```

**关键字段约束**（M3 必须填，M4/M5 只读）：
- `voxel.size_meters`：单个体素的物理边长（米）。**必须填实际值**，不能写 1.0 当默认。MVP 推荐 `0.05`（5cm，对人尺度刚好）。
- `voxel.chunk_extent_voxels`：单个 chunk 包含多少体素，**三个值必须相等且是 2 的幂**（便于八叉树）。推荐 `[32, 32, 32]`。
- `coord_system.convention`：**强制 right-handed Y-up**（Godot / Blender / OpenGL 默认）。从 SLAM 的 Z-up（ROS 默认）转换的工作在 M3 完成，落盘后游戏端不需要再转。
- `bounds_chunks`：世界范围用 chunk 坐标表示，不是体素坐标，不是米。

### 4.3 palette.json — 材质与语义调色板

```json
{
  "materials": [
    { "id": 0, "name": "air",      "color_rgb": [0,0,0],       "flags": ["empty"] },
    { "id": 1, "name": "concrete", "color_rgb": [180,180,180], "flags": ["solid","destructible"], "density": 2400, "hardness": 30 },
    { "id": 2, "name": "wood",     "color_rgb": [139,90,43],   "flags": ["solid","destructible","flammable"], "density": 700, "hardness": 5 },
    { "id": 3, "name": "glass",    "color_rgb": [200,230,255], "flags": ["solid","destructible","transparent"], "density": 2500, "hardness": 2 }
  ],
  "semantic_classes": [
    { "id": 0, "name": "unknown", "default_material": 1 },
    { "id": 1, "name": "wall",    "default_material": 1 },
    { "id": 2, "name": "floor",   "default_material": 1 },
    { "id": 3, "name": "door",    "default_material": 2 },
    { "id": 4, "name": "window",  "default_material": 3 }
  ],
  "color_lut": [
    [0, 0, 0],
    [180, 178, 175],
    [139, 90, 43]
  ]
}
```

**约束**：
- `material.id` 是 `uint8`（0-255），所以一个世界最多 256 种材质。MVP 推荐 ≤ 32 种。
- `id=0` 永远保留给 "air"。
- 颜色用 `sRGB` 8-bit，**不要预乘 alpha**。

### 4.4 chunks/{x}_{y}_{z}.chunk — 单个 chunk 的二进制格式

**这是文件级的"地图数据"具体定义**。借鉴 MagicaVoxel chunked 设计，但更简单（无嵌套）。

每个 `.chunk` 文件是 **小端字节流**：

```
偏移   类型         字段                       说明
0      char[4]      magic = "CHNK"             文件魔数
4      uint16       format_version             目前固定 = 0x0100
6      uint16       compression                0=raw, 1=lz4, 2=zstd
8      int32        chunk_x                    chunk 坐标
12     int32        chunk_y
16     int32        chunk_z
20     uint8        encoding                   0=dense, 1=rle, 2=svo
21     uint8[3]     reserved (must be 0)
24     uint32       payload_bytes              下方 payload 长度
28     uint8[N]     payload                    见下方
28+N   uint32       crc32                      payload 的 CRC32
```

**payload 的三种 encoding**：

**encoding=0 (dense)** — 适合 >50% 占据率的 chunk
```
voxels: VoxelCell[32*32*32]   // 行主序，z 最慢，y 次之，x 最快
```

**encoding=1 (rle)** — 适合稀疏 chunk（大部分项目走这条）
```
run_count: uint32
runs: RleRun[run_count]
```

**encoding=2 (svo)** — 适合极稀疏 + 需要快速空间查询
```
node_count: uint32
nodes: SvoNode[node_count]    // 引用 voxelis 的 BlockId 编码
```

**VoxelCell（每体素 4 字节，固定大小）**：

```
偏移   类型     字段              说明
0      uint8    material_id       0=air，引用 palette.materials[id]
1      uint8    semantic_id       引用 palette.semantic_classes[id]
2      uint8    state             0=完整, 1=破损, 2=燃烧, 3=积水, ...（具体含义由 M6 定义）
3      uint8    color_palette_idx 0=用 material 的 default color_rgb，1-255 = 引用 palette.color_lut[idx]
```

为了支持 per-voxel 颜色而不撑爆每体素字节数，`palette.json` 中再加一个 **`color_lut`** 数组（256 个 RGB 项）：M3 在扫描期对每个 chunk 出现过的体素颜色做 k-means 量化到 ≤255 色，落进 LUT。这是 MagicaVoxel 的成熟做法。

> **设计取舍说明（这是必读的"为什么"）**：4 字节/voxel 是 dense chunk 大小与表达力的平衡点。32³ dense chunk = 128 KB，落盘走 zstd 后通常 5-20 KB。LUT 量化把"任意颜色"压成 8-bit 索引，世界总大小可控；如果后期发现 256 色不够，可以为每 chunk 单独存一个 LUT（在 chunk header 里加一段），仍兼容本格式。

**RleRun（5 字节/run）**：
```
偏移   类型     字段
0      VoxelCell cell  (4 字节)
4      uint8    length    run 长度（1-255，0 保留）
```
扫描顺序与 dense 相同。

### 4.5 chunks.idx — chunk 索引

```
偏移   类型             字段
0      char[4]          magic = "CIDX"
4      uint32           chunk_count
8      ChunkEntry[N]    entries（按 morton 码排序）

ChunkEntry (20 字节):
0      int32   chunk_x
4      int32   chunk_y
8      int32   chunk_z
12     uint64  file_offset_in_megachunk    // 0 表示独立 .chunk 文件
20     uint32  size_bytes
```

> **为什么需要 idx**：M4 流式加载时，根据玩家位置算出 16 个邻居 chunk 坐标，用 idx 二分查找文件位置，避免列目录。

### 4.6 semantics/instances.json — 语义实例（可选但强烈推荐）

```json
{
  "instances": [
    {
      "instance_id": 1,
      "class_id": 3,
      "label": "door_kitchen",
      "bbox_voxel_min": [120, 0, 80],
      "bbox_voxel_max": [124, 40, 82],
      "voxel_mask_chunk_refs": [[3,0,2], [3,0,3]],
      "attributes": { "openable": true, "initial_state": "closed" }
    }
  ]
}
```

这就是"GTA5 风格交互"的入口：M6 看到玩家走到 `instance_id=1` 的 bbox 内并按 E，就能触发 `door.open()`。没有这一层，体素只是几何，**有这一层，体素才成为可交互世界**。

### 4.7 接口契约（M2 必须满足的、M3+ 可以依赖的）

| # | 契约 | 强制？ |
|---|---|---|
| C1 | `voxel.size_meters` 在整个世界生命周期内不变 | 是 |
| C2 | chunk 文件名 `x_y_z.chunk` 中的坐标 = chunk 内部存的坐标 | 是 |
| C3 | 任意两个 chunk 的内部体素在世界坐标系下不重叠 | 是 |
| C4 | `material_id=0` 表示空气，碰撞/渲染都跳过 | 是 |
| C5 | semantic_id 的范围必须在 palette.json 里有定义 | 是 |
| C6 | 同一世界内所有 chunk 必须用同一份 palette.json | 是 |
| C7 | 单 chunk 写出后必须包含 CRC32，加载方必须校验 | 是 |
| C8 | 加载方对未知 encoding 必须报错退出，不能猜 | 是 |
| C9 | 缺失的 chunk = 该区域全部空气（不是"未知"） | 是 |

---

## 5. 各模块技术选型（含开源依据）

### M1 传感采集

| 选项 | 优点 | 缺点 | 推荐场景 |
|---|---|---|---|
| **iPhone Pro + ARKit** | 自带深度+IMU 融合，免费，便携 | 仅 iOS，原始数据要 Record3D 类 App 导出 | MVP 入门 |
| **Intel RealSense D455** | ROS 一线公民，开源驱动，~$300 | 户外阳光下深度噪声大 | 桌面/室内开发首选 |
| **Livox Mid-360** | LiDAR，户外 100m+，结构化 | ~$1000，需自己接电源 | 户外/大场景 |
| **公开数据集（TUM RGB-D / KITTI）** | 零硬件成本，可重复实验 | 不是你自己的世界 | 跑通管线时用 |

**MVP 建议**：先用 TUM RGB-D `fr1_room` 序列把整个管线打通，再换 RealSense。

### M2 SLAM & 重建（你处理，本 Spec 给参考清单）

| 选项 | 输出契合度 | 备注 |
|---|---|---|
| **nvblox** | ★★★★★ | GPU TSDF，速度比 voxblox 快 31×；接口已经是 layered voxel grid，最接近 §4 的契约 |
| **Voxblox** | ★★★★ | CPU，工程稳定，Protobuf 序列化好抄 |
| **VDBblox** | ★★★★ | OpenVDB 后端，存储天然稀疏 |
| **RTAB-Map** | ★★★ | 开箱即用，输出 .ply/.pcd，需要写个体素化后处理 |
| **ORB-SLAM3 + Open3D TSDF** | ★★★ | 经典组合，论文最多 |
| **3DGS / Nerfstudio** | ★★ | 视觉最佳但**点不是体素**，体素化是另一个研究问题；不推荐 MVP |

**输出适配**：无论选什么，最后一步都是写一个"adapter"，把 SLAM 的内部表示转成 §4 的 .vxw 目录。这个 adapter 是 M3 的入口。

### M3 地图构造器（本项目你需要写的核心组件之一）

主要工作：
1. **坐标系/比例修正**：ROS 的 Z-up + 米单位 → §4 的 Y-up + voxel_size。具体见 [Gaussian Splatting 文章对 Unity/Unreal 的坐标修正][unity-3dgs] 的踩坑列表（轴翻转 + 比例因子）。
2. **chunk 切片**：把 SLAM 输出的一个大稠密 volume 按 32³ 切片。
3. **encoding 选择策略**：每 chunk 算占据率，> 50% 用 dense，5–50% 用 rle，< 5% 用 svo。
4. **语义→材质映射**：用 palette.json 的 `default_material` 字段做查表。
5. **语义实例提取**：连通域分析（每个连通的"door"语义体素 → 一个 instance）。CityMinecraft 的 `--decorate` 步骤是好范本。

**实现栈推荐**：Python + Open3D + NumPy + zstandard（mvp 阶段写得快）。性能瓶颈出现后可以把热点用 Rust/C++ 重写。

### M4 渲染适配器

两条路，**选一条，不要骑墙**：

**A. Mesh 路线（推荐 MVP）**：
- 每 chunk 用 **Greedy Meshing** 或 **Surface Nets / Dual Contouring** 转网格
- 用引擎自带渲染管线
- 引用：voxelis 的 SIMD greedy mesher（WIP）、Zylann/godot_voxel 的 Transvoxel LOD
- 优点：兼容性最好，光照/阴影白嫖引擎
- 缺点：体素改变要重网格化

**B. SVO Raymarch 路线（Teardown 路线）**：
- 不生成三角形，shader 里光线步进体素网格
- 引用：Amanatides & Woo Fast Voxel Traversal、Teardown 技术拆解、Lundqvist et al. 2023 (LTH) 实现报告
- 优点：millimeter 级密度也能跑实时；天然支持精确光追
- 缺点：自己写 fragment shader，移植性差，引擎光照系统接不进来

**MVP 选 A**。等 MVP 跑通、性能不够时再考虑 B。

### M5 引擎集成层

**主候选**：

| 引擎 | 体素现成方案 | 物理 | 推荐度 |
|---|---|---|---|
| **Godot 4** | Zylann/godot_voxel（C++ 模块）/ godot_openvdb_texture3d | 引擎自带 / Jolt | ★★★★ MVP 首选 |
| **Unreal 5** | eidosmontreal/unreal-vdb + darbycostello/Nav3D | Chaos / Jolt | ★★★ 美术质量上限高，但 C++ 编译重 |
| **Bevy + voxelis** | voxelis 原生 Rust | bevy_rapier | ★★★ 全 Rust 路线漂亮，但生态浅 |

**MVP 建议**：Godot 4 + Zylann/godot_voxel。理由：
1. C++ 模块自带 chunk 流式加载、LOD、blocky/smooth 双模
2. GDScript 写交互/UI 比 C++ 蓝图快 5×
3. 编辑器轻量，VMware Ubuntu 起得来（Unreal 在 VM 里很痛苦）

**物理**：碰撞用 Minecraft-like 的快速 AABB（Zylann 自带），不用 PhysX。

**寻路**：参考 Nav3D / AeonixNavigation / godot_flight_navigation_3d 的 SVO 实现。但 MVP 不做 NPC，先跳过。

### M6 玩法 / 交互层

完全在引擎脚本层。关键事件：

```python
# 伪代码（GDScript 风格）
on_player_action(action):
    if action.type == "destroy":
        chunk = world.chunk_at(action.target_voxel)
        chunk.set_voxel(action.target_voxel, AIR)
        renderer.invalidate_chunk(chunk)
        physics.invalidate_chunk(chunk)
        emit_signal("voxel_destroyed", action.target_voxel)
    elif action.type == "interact":
        instance = world.instance_at(action.target_voxel)
        if instance and instance.attributes.openable:
            instance.toggle_state()
```

---

## 6. MVP 里程碑（建议节奏，非强制）

| 阶段 | 周数 | 输出验收 |
|---|---|---|
| **P0 接口固化** | 1–2 | §4 的 .vxw 目录格式有 Python 读写库 + 单元测试 + 一个"全空 + 一面墙"的样例文件，能在 16 进制查看器里逐字节解释 |
| **P1 假数据 → 引擎** | 3–4 | M4+M5：Godot 加载 P0 的样例 .vxw，玩家可走、可碰撞 |
| **P2 真实扫描** | 5–8 | M1+M2+M3：用 TUM `fr1_room` 跑通，生成 .vxw，能在 Godot 里走一圈 |
| **P3 第一种交互** | 9–10 | M6：能用工具破坏一个体素，对应 chunk 重网格化 |
| **P4 语义层** | 11–12 | semantic 实例提取 + "开门" 交互 |

P0 是死命令：**先有数据接口、再有任何代码**。这条线被反转过的项目 90% 死在集成阶段。

---

## 7. 风险与对策

| 风险 | 概率 | 影响 | 对策 |
|---|---|---|---|
| Mesh 重网格化在改一个体素时太慢 | 高 | MVP 卡死 | chunk 内本地重网格化（只重做受影响 chunk），参考 godot_voxel 的脏标记机制 |
| SLAM 漂移导致 chunk 边界对不上 | 高 | 世界扭曲 | 离线后处理用回环检测重整体姿，再做体素化（不要边 SLAM 边 .vxw） |
| 体素颜色和真实纹理差距大 | 中 | 视觉劝退 | 加 per-voxel 颜色（§4.4 已留位），后期可加 texture splatting |
| Godot 在 VMware 里跑不动 | 中 | 开发受阻 | 宿主机直接装 Godot，VM 只跑 SLAM（参考 [[project_setup]] 的 F:\ 用法） |
| 数据量爆炸（1 公里² 房间） | 低（MVP 阶段） | 加载慢 | encoding=svo + zstd；引入 LOD（lod_levels 字段已留） |
| palette.json 在世界生命周期里变更 | 低 | 数据损坏 | 已通过契约 C6 禁止；如必须改，要做 chunk 全量迁移 |

---

## 8. 给后续工程师的扩展点说明

按 "替换难度从易到难" 列出：

1. **加新材质**：只改 palette.json，无需重新生成 chunks。
2. **加新交互类型**：只改 M6，无需碰其他模块。
3. **换 SLAM 后端**：只改 M2 + M3 入口 adapter，下游不受影响。
4. **换渲染引擎**：M4+M5 全换，但 §4 数据契约不变，离线 .vxw 数据继续可用。
5. **加 LOD**：M3 生成时多产 `lod_1/`、`lod_2/` 子目录，每层是上层体素 2×2×2 合并。manifest 的 `lod_levels` 字段已经预留。
6. **加多人**：M5 之上加网络层；体素世界天然 chunk 化，同步友好。
7. **加实时 SLAM 闭环**：M2 流式产 .vxw 增量，M4/M5 增量 reload。MVP 不做。

---

## 9. 参考文献与开源链接

- Cloud2Craft — https://github.com/AntoineMiras/Cloud2Craft
- VoxelEarth — https://github.com/ryanhlewis/VoxelEarth
- World2Minecraft — https://world2minecraft.github.io/
- CityMinecraft — https://github.com/andreiBe/CityMinecraft
- Scan2Craft — https://github.com/y3jian/MinecraftModel
- nvblox — https://github.com/nvidia-isaac/nvblox
- Voxblox — https://github.com/ethz-asl/voxblox
- VDBblox — https://github.com/yinloonga/vdbblox
- OpenVDB — https://github.com/AcademySoftwareFoundation/openvdb
- voxelis (Rust SVO-DAG) — https://github.com/WildPixelGames/voxelis
- Zylann/godot_voxel — https://github.com/Zylann/godot_voxel
- eidosmontreal/unreal-vdb — https://github.com/eidosmontreal/unreal-vdb
- darbycostello/Nav3D — https://github.com/darbycostello/Nav3D
- MagicaVoxel .vox spec — https://github.com/ephtracy/voxel-model/blob/master/MagicaVoxel-file-format-vox-extension.txt
- Teardown 技术访谈 — https://www.gamedeveloper.com/design/how-beautiful-voxels-laid-the-way-for-i-teardown-s-i-heist-y-framework
- Amanatides & Woo, "A Fast Voxel Traversal Algorithm for Ray Tracing", 1987
- Daniel Brewer, "3D Flight Navigation Using Sparse Voxel Octrees", Game AI Pro 3

[unity-3dgs]: https://polyvia3d.com/guides/gaussian-splatting-unity-unreal
