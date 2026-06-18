# CharacterBody3D 玩家物理升级(StereoRig)

> 日期:2026-06-18。设计文档(spec)。
> 解决的问题:1P(第一人称)行走用的是**单点射线桩**(`_slide_axis`/`_is_grounded`),会穿薄墙、从地板洞掉出去、不与家具碰撞。升级为真正的 `CharacterBody3D` 胶囊 + `move_and_slide` 扫掠碰撞,并加完整手感(斜坡/台阶/蹲下)。3P 自由飞观察相机保持不变。

## 1. 架构

`godot_viewer/stereo_rig_controller.gd` 的 `StereoRig` 由 `extends Node3D` 改为 **`extends CharacterBody3D`**,新增一个 `CollisionShape3D` 子节点(`CapsuleShape3D`)。CharacterBody3D 是 Node3D 子类,现有 transform / 子节点(双目相机、humanoid、walk 动画、sit)逻辑全部照旧。

**三模式分流**(`_process`/`_physics_process` 内按 `_physics_mode`、`is_seated()` 走):

| 模式 | 行为 |
|---|---|
| **1P 物理(改写)** | WASD→水平 `velocity`(yaw 平面);重力累加 `velocity.y`;`Space` 在 `is_on_floor()` 时跳;`move_and_slide()` 扫掠胶囊碰撞(体素 StaticBody + 家具 RigidBody)。`is_on_floor()` 取代 `_is_grounded`。 |
| **3P 自由飞(不变)** | 直接 `translate`/写 `global_position`,不调 `move_and_slide`、无重力、无碰撞。 |
| **sit(不变)** | 停在实体上,跳过移动。 |

**关键帧:** 物理移动放到 `_physics_process(delta)`(move_and_slide 必须在物理帧);3P 自由飞 + 视觉同步可留在 `_process`。`_sync_stereo` / `_animate_walk` 每帧都跑。

**删除**射线桩:`_slide_axis`、`_is_grounded` 及其 `@export collision_clearance_m`/`ground_probe_m`。

**碰撞层**:胶囊 `collision_mask` 覆盖体素碰撞体所在层(`voxel_renderer` 的 `VoxelCollisionBody` StaticBody3D)+ 家具 RigidBody 层。当前两者都在默认层 1;胶囊 mask 含层 1 即可。`collision_layer` 设独立位避免与可拾取实体的射线选取冲突(见 §5 风险)。

## 2. 组件与参数

**节点结构:**
```
StereoRig (CharacterBody3D)          ← rig 原点 = 眼睛/相机高度
├ BodyShape (CollisionShape3D, CapsuleShape3D)  ← 下移使胶囊底=脚
├ Humanoid / LeftEyeMarker / RightEyeMarker
└ (双目 Camera3D 由 controller 同步到 rig 原点 ± baseline)
```
rig 原点 = 眼睛高度。胶囊中心下移使**胶囊底对齐脚底**:`BodyShape.position.y = -(eye_height_m − capsule_h/2)`。

**@export 参数:**
| 参数 | 默认 | 用途 |
|---|---|---|
| `eye_height_m` | 1.6 | rig 原点(相机)距脚高度 |
| `capsule_radius_m` | 0.3 | 胶囊半径 |
| `capsule_stand_h_m` | 1.7 | 站立胶囊全高 |
| `capsule_crouch_h_m` | 0.9 | 蹲下胶囊全高 |
| `max_step_m` | 0.35 | 可攀台阶高 |
| `floor_max_angle_deg` | 46 | 斜坡上限(写入 `self.floor_max_angle`) |
| 保留 | — | `gravity_mps2=9.81`/`jump_velocity_mps=4.5`/`rig_speed`/`fast_multiplier` |
| 删除 | — | `collision_clearance_m`/`ground_probe_m` |
| 改语义 | — | `spawn_y_m`:rig 原点落生高度 = 地面 + eye_height(默认仍 1.0 起步可调) |

**1P 物理段逻辑(`_physics_process`):**
1. 水平输入:`fwd = -basis.z`、`right = basis.x`,各置 `y=0` 归一;WASD 累加 → `horiz`;`velocity.x/z = horiz.normalized()*spd`(`spd = rig_speed*(Shift?fast:1)`)。
2. 重力/跳:`is_on_floor()` 为真且未跳 → `velocity.y = 0`,按 Space → `velocity.y = jump_velocity_mps`;否则 `velocity.y -= gravity_mps2*delta`。
3. `move_and_slide()`。
4. **台阶**:若 `get_slide_collision_count()>0` 且本帧水平位移被显著阻挡、且 `test_move(从当前抬高 max_step_m 处沿水平位移)` 不碰撞 → `global_position.y += 命中台阶高(≤max_step_m)`。配 `floor_snap_length = max_step_m` 让下台阶贴地。
5. **蹲下**:按住 Ctrl → `BodyShape` 高度切 `capsule_crouch_h_m`、相机 eye 降 `(stand−crouch)/2`;松开时 `test_move` 头顶净空(抬到 stand 高不碰撞)才恢复;卡住则维持蹲。

**斜坡**:`self.floor_max_angle = deg_to_rad(floor_max_angle_deg)`、`floor_stop_on_slope = true`。

**输入映射(1P)**:WASD 水平 · Space 跳 · Ctrl 蹲(held)· Shift 冲刺。3P 自由飞不变(Space/Ctrl=升/降)。

## 3. 测试策略

GDScript 集成逻辑(move_and_slide vs 体素)无法纯单测;分两层:
- **纯逻辑单测(可选,轻)**:把"yaw 平面水平向量"算法抽成无副作用静态函数,Python 侧无关;GDScript 侧若有 `_editor_selftest` 钩子可加断言。
- **headless Godot 冒烟(主)**:扩 `boot_config` + `test_hooks_controller`,加两个 CLI test flag,跑完打日志,pytest 解析日志断言:
  - `--test-physics-drop`:强制 1P、把 rig 置于已知地面上方 0.5m、跑 ~60 物理帧 → 日志 `physics_drop {settled_y, on_floor:true, fell_through:false}`;断言落在地面附近且 `on_floor`。
  - `--test-physics-wall`:1P 朝最近墙体推进 ~60 帧 → 日志 `physics_wall {blocked:true, dx_into_wall<ε}`;断言被挡未穿。
- **人工快照**:补全公寓里走一圈(已有 --snapshot)。

## 4. 模块边界
- 仅改 `godot_viewer/stereo_rig_controller.gd`(主)+ `main.tscn`(给 StereoRig 加 CapsuleShape 子节点;或在 `init_controller` 代码内建 CollisionShape 以免改场景文件——优先代码内建,减少 .tscn 改动)。
- `boot_config.gd` + `test_hooks_controller.gd`:加两个测试 flag(测试用)。
- 不动 `voxel_renderer`(碰撞体已就绪)、不动 vlayer。

## 5. 风险 / 待定
- **拾取选取冲突**:实体选取用射线(`entity_selector`)。给玩家胶囊单独 `collision_layer` 位,确保选取射线 mask 不含玩家层,避免点到自己。
- **台阶判定**:`test_move` 的 step-up 在 ConcavePolygonShape 上需调参;max_step_m 过大可能"爬墙"。先保守 0.35m + 仅在 `is_on_floor` 时启用。
- **眼/脚偏移与 sit/reset_pose**:`reset_pose`、`set_pose`、sit 的 lift 偏移都按"原点=眼"语义,改胶囊偏移后需一并核对(脚不穿地、坐姿不变)。
- **代码内建 CollisionShape 时机**:在 `init_controller`(或 `_ready`)创建,早于第一帧物理。
- crouch 切换胶囊时机:改 shape 高度同时改 `position.y` 保持脚底不动。
