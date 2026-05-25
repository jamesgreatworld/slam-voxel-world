# 架构设计（slam-voxel-world）

> 本文档管"**怎么组织**"。`spec.md` 管"**.vxw 文件长什么样**"。两者互补，spec 是契约，本文是模块边界与依赖方向。
> 日期：2026-05-25。任何模块重构前必读，重构后回填。

---

## 0. 设计原则

| 原则 | 含义 | 反例（不要做） |
|---|---|---|
| **关注点分离 (SoC)** | 一个模块只做一类事 | `main.gd` 同时管渲染、输入、UI、快照、日志 |
| **数据契约优先** | 模块之间只通过 `.vxw` 文件 或者 well-typed dataclass 通信 | 让 SLAM 模块直接 import Godot 类 |
| **可替换性** | 同层的模块可以整体换掉而不动其他层 | 渲染器写死在 `vxw_format.py` 里 |
| **渐进增强** | 新功能优先以"增加新文件"实现，不修改老文件 | 改 Material dataclass 加 5 个字段，引发 21 个测试 fail |
| **单向依赖** | 上层依赖下层，下层不知道上层 | Python `vxw_format` import Godot 模块 |

---

## 1. 系统鸟瞰

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  REAL WORLD  (the scanned environment / point cloud source)                  │
└──────────────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  M1 传感采集     [外部]                                                       │
│  RealSense / iPhone ARKit / Livox LiDAR / rosbag                             │
└──────────────────────────────────────────────────────────────────────────────┘
                                  │  raw frames + IMU
                                  ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  M2 SLAM 与重建  [用户处理 — lightning_lm_foxy / Hydra]                       │
│  → 写出 .pcd / .bin (PCL 点云) 到 E:\aros_slam_ws\...\data\<run>\global.pcd  │
└──────────────────────────────────────────────────────────────────────────────┘
                                  │  PCL .pcd 文件
                                  ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  M3 适配器层  [Python; 可扩展多种 import]                                     │
│   • pcd_to_vxw.py           ← 已实现                                          │
│   • schematic_to_vxw.py     ← 未来：Minecraft .schem/.litematic              │
│   • vox_to_vxw.py           ← 未来：MagicaVoxel .vox                          │
│   • bin_to_vxw.py           ← 未来：lightning 的 per-tile bin                 │
└──────────────────────────────────────────────────────────────────────────────┘
                                  │  .vxw 目录（文件契约 = spec.md §4）
                                  ▼
        ╔═══════════════════════════════════════════════════════════════╗
        ║   .vxw 格式 = 整个系统的"接口" (Layer 0 数据契约)              ║
        ║   manifest.json + palette.json + chunks/*.chunk + chunks.idx  ║
        ║   spec.md 是唯一权威，vxw_format.py / vxw_loader.gd 是镜像     ║
        ╚═══════════════════════════════════════════════════════════════╝
                                  │
                ┌─────────────────┼─────────────────┐
                ▼                 ▼                 ▼
┌─────────────────────┐  ┌────────────────┐  ┌────────────────────────┐
│ M4 渲染器           │  │ M5 引擎集成     │  │ M6 玩法 / 交互          │
│ (Godot/Unreal/Bevy) │  │ (相机/碰撞/AI) │  │ (编辑/破坏/拾取/UI)    │
└─────────────────────┘  └────────────────┘  └────────────────────────┘
```

**关键观察**：`.vxw` 是整个系统的中央接口。SLAM 端只要写出符合 spec 的目录，游戏端只要读取就行，**两侧可以独立替换、独立演进**。

---

## 2. 分层架构（按依赖方向，下层先于上层）

| 层 | 责任 | Python | Godot | 状态 |
|---|---|---|---|---|
| **L0 契约** | `.vxw` 文件格式定义 | `docs/spec.md` | 同 | ✅ |
| **L1 格式实现** | 读写 `.vxw` 的 IO 库 | `vxw_format.py` | `vxw_loader.gd` + `vxw_writer.gd` | ✅ |
| **L2 数据领域** | 内存中的 World/Chunk/Voxel 对象 | `vxw.World` 类 | `VxwLoader.VxwWorld` 类 | ✅ |
| **L3 持久化适配** | 外部数据 ↔ .vxw 转换 | `m3_adapter/*` | — | 部分（pcd） |
| **L4 渲染层** | 把 World 变成可见像素 | — | `voxel_renderer.gd`（待建） | **god class 状** |
| **L5 仿真/控制层** | 相机、玩家、物理、AI | — | `camera_controller.gd`、`voxel_editor.gd`、`stereo_rig.gd`（待拆） | **god class 状** |
| **L6 UI 层** | HUD、菜单、面板 | — | `ui/*.gd`（待建） | **god class 状** |
| **L7 编排层** | 装配 + 路由事件 | `pcd_to_vxw.py main()` | `main.gd`（瘦壳，待重构） | **god class** |

**违规现状**：`main.gd` 目前混合了 L4 (`_build_multimesh`) + L5 (`_apply_keyboard`, orbit cam) + L6 (HUD labels) + L7 (init wiring)，共 350 行。这是改了几次后自然膨胀的，必须拆。

---

## 3. 模块详解

### 3.1 L1 格式（vxw_format / vxw_loader / vxw_writer）

**契约**：byte-level 等价。三者实现同一份 spec §4。

**Python 公开 API**（`vxw_format.py`）：
```python
# 数据类
World(manifest, palette, chunks: dict[tuple[int,int,int], Chunk])
Chunk(coord, voxels: np.ndarray[E,E,E,VOXEL_DTYPE], encoding, compression)
Manifest, Palette, Material, SemanticClass

# IO 函数
read_world(path) -> World
write_world(path, world)             # 全量写，会清空旧 chunks/
save_dirty(path, world) -> set       # 增量写，只刷脏 chunk

# 单文件 IO
read_chunk(path) -> Chunk
write_chunk(path, chunk)
read_manifest / write_manifest / read_palette / write_palette
read_chunks_index / write_chunks_index

# 修改
World.set_voxel(world_voxel, cell)
World.mark_dirty(chunk_coord)
World.get_chunk_or_air(coord) -> np.ndarray
```

**GDScript 镜像**（`vxw_loader.gd` / `vxw_writer.gd`）：相同概念，能跨进程跨语言读写。

**禁忌**：
- 任何不在 spec §4 的 byte-level 改动都要先改 spec、再改三处实现、再加测试。
- L1 不能引用 Godot 类或 numpy 以上的 Python 依赖。

### 3.2 L3 持久化适配（m3_adapter/）

**契约**：每个 adapter 都是一个独立的 Python 脚本，把某种外部格式转成 .vxw。

**当前**：
```python
# m3_adapter/pcd_to_vxw.py
def load_pcd_xyz(path) -> np.ndarray  # Nx3 float64
def ros_zup_to_vxw_yup(xyz) -> np.ndarray
def voxelize_and_group(xyz, voxel_size, chunk_extent, compression) -> (chunks, bmin, bmax)
def build_palette() -> Palette
def main() -> CLI 入口
```

**扩展模板** —— 加新输入格式时复制这个结构：
```python
# m3_adapter/<name>_to_vxw.py
def load_<format>(path) -> SomeInternalRepresentation
def to_voxels(...) -> (chunks_dict, bounds_min, bounds_max)
def build_palette() -> Palette        # 该格式特有的材质映射
def main() -> CLI 入口
```

**未来加哪些**：
- `schematic_to_vxw.py` — Minecraft .schem / .litematic
- `vox_to_vxw.py` — MagicaVoxel .vox（用 [magicavoxel](https://github.com/ephtracy/voxel-model) spec）
- `nbt_to_vxw.py` — Minecraft 结构方块导出
- `live_slam_to_vxw.py` — 在线 SLAM ROS2 节点直接推 .chunk 文件

### 3.3.1 SLAM → 体素的多条重建路线（M3 内并列）

SLAM 的最基础输出是**点云**，但从点云到游戏可用的体素世界，有多条质量逐渐提升的路线。它们都是 M3 adapter，**并列存在、用户可选**，不互相替代：

| Adapter | 输入 | 中间形态 | 体素来源 | 视觉效果 | 实现状态 |
|---|---|---|---|---|---|
| `pcd_to_vxw.py` | 点云 | 无 | 每个点 → 最近 voxel cell | **碎屑感**，有洞 | ✅ 已实现 |
| `tsdf_to_vxw.py` | 点云 | Open3D `ScalableTSDFVolume` 融合 | TSDF 零等值面附近的 voxel 标为占据 | **表面更连续**，洞少 | 未实现 |
| `mesh_to_vxw.py` | 点云或现成 mesh | Marching Cubes / Poisson 重建 mesh | mesh 体素化（每三角形覆盖到的 voxel） | **更像建筑**，但丢失结构外细节 | 未实现 |
| `nerf_to_vxw.py` / `gs_to_vxw.py` | RGB 序列 | NeRF / 3DGS 重建 | 从神经场密度阈值采样到 voxel | **最高保真**但显存重 | 未实现 |
| `semantic_to_vxw.py` | 点云 + 2D 语义分割 | TSDF + semantic fusion（Hydra/Kimera 风格） | 每 voxel 同时知道几何 + 语义类 | 可做"门是门、墙是墙"的交互 | 未实现 |

**关键洞察**：你的实际工程目标是探索"哪种重建路线最适合 Minecraft/游戏风格"。架构允许你**对同一份 SLAM 数据跑多个 adapter，输出到不同 .vxw**，然后在 Godot 端切换加载对比效果。这本身就是 R5 之后值得做的研究工作。

当前 `pcd_to_vxw.py` 已经验证管线打通；下一步若想质量提升，最有性价比的是接 Open3D 的 TSDF（`tsdf_to_vxw.py`），表面会瞬间变连续。

### 3.3 L4 渲染层（**待重构**：拆出 voxel_renderer.gd）

**目标 API**：
```gdscript
# voxel_renderer.gd  (extends Node3D)
func build(world, palette: Palette) -> void   # 初始化 MMIs，分组到 per-material
func remove_voxel(world_voxel_idx: Vector3i) -> bool   # 视觉移除（hide instance）
func add_voxel(world_voxel_idx, material_id) -> bool   # 视觉添加（未来）
func get_voxel_at_ray(origin, dir, max_dist) -> Variant   # 拾取（返回 Vector3i 或 null）
```

**重要：per-material 渲染** —— 按 material_id 把体素分桶，每桶一个 MultiMeshInstance3D，每桶有自己的 StandardMaterial3D：

```
mmi_by_material: dict[int, MultiMeshInstance3D]
```

- 不透明材质 → 默认 StandardMaterial3D
- 玻璃 → albedo alpha < 1, transparency = TRANSPARENCY_ALPHA
- 发光 → emission_enabled = true
- 金属 → metallic 高，roughness 低

`Material` dataclass 也要扩字段（见 L2 扩展点）。

**替换性**：把 `voxel_renderer.gd` 整个换成自定义着色器/SVO raymarch/greedy mesh，**其他层不受影响**。

### 3.4 L5 仿真/控制层（**待重构**）

拆成几个独立 controller：

```
camera_controller.gd     1P/3P 切换 + 鼠标 orbit + 键盘飞行（当前在 main.gd 里）
stereo_rig_controller.gd RIG 6DOF + L/R PiP 同步（当前在 main.gd）
voxel_editor.gd          ✅ 已有：click → 拾取 → 销毁 + 发信号
physics_controller.gd    未来：1P 重力、墙体碰撞
ai_controller.gd         未来：NPC 寻路
```

每个 controller 是 Node3D 或 Node，自包含 `_input` / `_process`，对外只发信号或暴露 setter。

### 3.5 L6 UI 层（**新建**）

```
ui/hud.gd               常驻 HUD（状态行、pose、模式标签）
ui/pause_menu.gd        Esc 打开的菜单（Save/Load/Quit/View/...）
ui/action_bar.gd        底部操作提示
ui/material_palette.gd  未来：选当前要放置的材质
ui/console.gd           未来：开发者命令行
```

每个 UI 文件 = 一个 PackedScene + 对应 gd 脚本。`main.tscn` 实例化它们。

### 3.6 L7 编排层（**目标：瘦壳**）

重构后的 `main.gd` 应该 < 80 行，只做：
```gdscript
extends Node3D

@onready var renderer = $VoxelRenderer
@onready var cam_ctl  = $CameraController
@onready var rig_ctl  = $StereoRigController
@onready var editor   = $VoxelEditor
@onready var hud      = $UI/HUD
@onready var menu     = $UI/PauseMenu

func _ready():
    var world = VxwLoader.load_world(_arg("world"))
    renderer.build(world, world.palette)
    cam_ctl.attach(rig_ctl.get_rig_node())
    editor.init_editor(world, renderer, $Camera3D)
    editor.voxel_destroyed.connect(_on_voxel_destroyed)
    menu.save_requested.connect(_on_save)
    menu.load_requested.connect(_on_load)
    hud.bind(world, rig_ctl, cam_ctl)
```

---

## 4. 可替换性矩阵

| 当前模块 | 可替换为 | 触发条件 | 工作量 |
|---|---|---|---|
| **Godot 渲染器** | Unreal Engine 5 + OpenVDB plugin | 高保真要求 | ★★★★ (M4+M5+M6 全换) |
| **Godot 渲染器** | Bevy + voxelis | Rust 全栈 | ★★★★ |
| **pcd_to_vxw** | bin_to_vxw（流式 per-tile） | 100M+ 点云 | ★★ |
| **pcd_to_vxw** | schematic_to_vxw | 导入玩家建筑 | ★★ |
| **Hydra/lightning SLAM** | ORB-SLAM3 / nvblox | 性能或场景需求 | 用户自处理，M3 加新 adapter |
| **DENSE/RLE 编码** | SVO 编码 | 超稀疏场景 | ★★（在 vxw_format 加 encoding=2） |
| **GZIP 压缩** | ZSTD（Godot 不行）/ LZ4 | 性能调优 | ★ |
| **flycam（旁观）** | 第三人称跟随 / 第一人称步行 | 玩法变化 | ★★（改 camera_controller） |

**关键点**：只要 .vxw 格式不变，**任何一格的换都不会塌陷其他格子**。

---

## 5. 扩展点（"想加 X 时改哪里"清单）

| 想加什么 | 改哪里 | 不要改哪里 |
|---|---|---|
| 新材质（玻璃/发光/金属） | `Material` dataclass + `voxel_renderer.gd` 的 per-material 分桶 | `Chunk`、`write_chunk` |
| 新数据源（.schem / .vox） | `m3_adapter/` 新文件 | 现有 adapter |
| 新交互（放置 / 油漆刷） | `voxel_editor.gd` 加方法 + UI 加按钮 | renderer 内部 |
| 新 UI 面板 | `ui/<name>.gd` + `ui/<name>.tscn` + main.tscn 实例化 | main.gd 主逻辑 |
| 新 SLAM 后端 | M3 加新 adapter；用户在 M2 跑那个后端 | M4+ 任何模块 |
| 新游戏机制（重力、NPC） | `L5 controller` 新文件 | renderer、format |
| 新世界 LOD 层级 | `Manifest.lod_levels` 已留位；M3 生成 `lod_1/` 子目录；renderer 按距离切换 | spec §4 主结构 |

---

## 6. 当前实现的违规清单（改之前别加新东西）

| # | 违规 | 模块 | 影响 |
|---|---|---|---|
| 1 | `main.gd` 同时承担 L4-L7 四层职责 | main.gd | 任何渲染/输入/UI 改动都要碰这一个文件，diff 难审 |
| 2 | `_build_multimesh` 写死单 MMI + 单 material | main.gd | 玻璃、发光、金属砖渲染不出来 |
| 3 | `Material` 数据类只有 color/flags，无 transparency / emission / metallic / roughness | vxw_format.py | 即使渲染器升级也没数据可读 |
| 4 | 调色板是世界级（global），不是 chunk 级 | vxw_format.py | 大场景一旦超 256 材质就破，Minecraft 是 per-section 调色板 |
| 5 | `voxel_editor` 直接知道 MMI 内部（spatial hash 用 instance index） | voxel_editor.gd | 改渲染器要同步改编辑器 |
| 6 | `flycam.gd` 是死代码（main.tscn 没引） | flycam.gd | 维护负担 |
| 7 | 没有 UI 层，所有控制靠快捷键，新手记不住 | — | 用户体验 |

---

## 7. 重构路线（按依赖顺序）

**阶段 R1：拆 main.gd（不加新功能，纯结构）**
1. 新建 `voxel_renderer.gd`，把 `_build_multimesh` + per-material 框架（先放一个材质桶占位）搬过去
2. 新建 `camera_controller.gd`，把 1P/3P + orbit + 键盘飞行搬过去
3. 新建 `stereo_rig_controller.gd`，把 RIG 控制 + L/R PiP 同步搬过去
4. 新建 `ui/hud.gd`，把状态/pose/模式 label 搬过去
5. `main.gd` 收缩到 < 80 行的瘦壳
- **验证**：所有现有快照、selftest 仍通过

**阶段 R2：UI 菜单（用户痛点）**
1. `ui/pause_menu.gd` + `.tscn`：Esc 弹出 Panel，按钮 Save / Load / View / Reset / Quit
2. `ui/action_bar.gd`：底部常驻提示
3. 删除 flycam.gd 死代码
- **验证**：快照能看到菜单 + 操作栏；点击按钮触发对应 action

**阶段 R3：编辑器接入（3 agent 整合）**
1. main.gd `_ready` 调 `editor.init_editor(world, renderer, $Camera3D)`
2. 连 `voxel_destroyed` → `_on_voxel_destroyed` → `vxw_writer.patch_voxel`
3. 菜单按钮 Save → 写入快照备份
- **验证**：点体素消失 + 重启加载存档能恢复

**阶段 R4：材质属性 + per-material 渲染**
1. Python `Material` 加字段：`transparent`、`emission_rgb`、`metallic`、`roughness`
2. 修改 spec.md §4.3 的 palette.json 结构（向后兼容：旧字段缺省填默认）
3. `voxel_renderer.gd`：按 material_id 分桶 → 多 MMI → 每 MMI 用对应 StandardMaterial3D
4. 生成一个 demo 世界：concrete + glass + lamp 三种材质
- **验证**：玻璃透明、灯发光，截图证明

**阶段 R5：Minecraft 资源对接**
1. `m3_adapter/schematic_to_vxw.py`：读 .schem，把 Minecraft block id 映射到我们的 material id
2. `palette_minecraft.json`：标准 MC 16 个常见方块的 Material 定义
3. （可选）材质包导入：把 16x16 PNG 贴到对应 material 的 MMI

**阶段 R6：物理碰撞（让"走路"成为可能）**
1. `physics_controller.gd`：从渲染器拿当前所有体素位置，构建 StaticBody3D + BoxShape3D（或 ConcavePolygonShape3D 由 greedy mesh）
2. 1P 模式给 rig 加重力 + 碰撞
- **验证**：1P 模式不再"穿墙"

**阶段 R7+ ：玩法层**
- NPC、破坏 + 重力坍塌、放置体素、库存系统、保存槽

---

## 8. 操作守则（写代码时回查）

1. 新加一个文件前，问"这放第几层？依赖了哪些下层？被哪些上层用？"
2. 改 spec.md 前，**先写迁移说明**（新字段缺省值、旧文件如何加载）
3. 改 `vxw_format.py` API 前，跑 `pixi run pytest tests/ -q`，红了任何一个测试就停手
4. 改 Godot 文件前，跑 headless：
   ```powershell
   F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer --position -10000,-10000 --resolution 1280x720 --quit-after 60 -- --world=F:/slam-voxel-world/out/baseline_20cm.vxw --snapshot=F:/slam-voxel-world/out/check.png
   ```
   PNG 没生成或有 push_error 就回滚

5. **不允许的依赖方向**：
   - Python 任何模块 → Godot
   - 同层模块互相 import（要走信号或事件）
   - `vxw_format.py` → 任何不在 numpy/zstandard/gzip 之外的库

---

## 9. 重构优先级建议

按"价值/代价"排：

| 阶段 | 价值 | 代价 | 推荐顺序 |
|---|---|---|---|
| R1（拆 main.gd） | 后续所有改动都受益 | 1 天 | **1** |
| R2（UI 菜单） | 直接解决"快捷键记不住" | 半天 | **2** |
| R3（编辑器接入） | 给玩家真正的 "modify" 能力 | 半天 | **3** |
| R4（per-material） | 解锁玻璃/发光/金属 | 1 天 | **4** |
| R5（MC 对接） | 海量社区资源 | 2-3 天 | 5 |
| R6（物理） | "1P 真走路" | 1-2 天 | 6 |
| R7（玩法） | 真"游戏" | 多周 | 后续 |

R1 + R2 + R3 是这一批的目标。R4 单独一批。R5+ 看实际需要。
