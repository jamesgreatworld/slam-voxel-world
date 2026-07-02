# CharacterBody3D 玩家物理升级 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `StereoRig` 的 1P 行走从单点射线桩升级为真正的 `CharacterBody3D` 胶囊 + `move_and_slide`(重力/跳/扫掠碰撞,撞体素结构+家具),并加斜坡限角、台阶攀爬、蹲下;3P 自由飞不变。

**Architecture:** `godot_viewer/stereo_rig_controller.gd` 的 `StereoRig` 改为 `extends CharacterBody3D`,代码内建 `CapsuleShape3D` 子节点(脚底对齐)。1P 走 `_physics_process` 用 `velocity`+`move_and_slide()`;3P/sit 不变。删除 `_slide_axis`/`_is_grounded`。验证用 headless Godot 冒烟(test flag 打结构化日志 + 快照)。

**Tech Stack:** Godot 4.6.3(GDScript)、`F:/Godot/Godot_v4.6.3-stable_win64_console.exe`、现有 `voxel_renderer` 的 VoxelCollisionBody(StaticBody3D)。无 Python 改动。

**Setup:** 直接在 `main` 上做(用户偏好单分支)。每任务一个 commit,结尾加 `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`。

**冒烟基准世界:** `out/apt_completed.vxw`(已补全地板的公寓,确保 1P 有连续地面)。Godot 启动模板:
```
"/f/Godot/Godot_v4.6.3-stable_win64_console.exe" --path "F:/slam-voxel-world/godot_viewer" --position -10000,-10000 --resolution 1280x720 --quit-after 240 -- --world=F:/slam-voxel-world/out/apt_completed.vxw <FLAGS>
```

---

### Task 1: StereoRig → CharacterBody3D + 胶囊 + 参数(无 1P 行为变化)

**Files:**
- Modify: `godot_viewer/stereo_rig_controller.gd`(class 声明、@export 块、`init_controller`)

- [ ] **Step 1: 改 class 声明与 @export**

`extends Node3D` → `extends CharacterBody3D`。
在 @export 块:删除 `collision_clearance_m`、`ground_probe_m`;新增:
```gdscript
@export var eye_height_m: float = 1.6        # rig 原点(相机)距脚
@export var capsule_radius_m: float = 0.3
@export var capsule_stand_h_m: float = 1.7
@export var capsule_crouch_h_m: float = 0.9
@export var max_step_m: float = 0.35
@export var floor_max_angle_deg: float = 46.0
```
保留 `rig_speed`/`fast_multiplier`/`keyboard_look_speed`/`keyboard_roll_speed`/`stereo_baseline`/`gravity_mps2`/`jump_velocity_mps`/`spawn_y_m`。

- [ ] **Step 2: 在 `init_controller` 末尾代码内建胶囊碰撞体 + 设角色参数**

在 `init_controller(...)` 函数体最后追加:
```gdscript
    # Body capsule for 1P physics. Built in code (no .tscn change). Capsule is
    # offset down so its BOTTOM aligns with the feet, while the rig origin stays
    # at eye height (where the cameras live).
    var _body_shape := CollisionShape3D.new()
    _body_shape.name = "BodyShape"
    var cap := CapsuleShape3D.new()
    cap.radius = capsule_radius_m
    cap.height = capsule_stand_h_m
    _body_shape.shape = cap
    _body_shape.position = Vector3(0.0, -(eye_height_m - capsule_stand_h_m * 0.5), 0.0)
    add_child(_body_shape)
    self.floor_max_angle = deg_to_rad(floor_max_angle_deg)
    self.floor_snap_length = max_step_m
```
并在文件变量区加缓存引用(供后续任务用):`var _body_shape: CollisionShape3D = null`,并在上面把 `var _body_shape :=` 改为赋值 `_body_shape = ...`(去掉局部 `var`),即:`_body_shape = CollisionShape3D.new()`。

- [ ] **Step 3: 冒烟——确认解析正常、能加载、3P 渲染不变**

Run:
```
"/f/Godot/Godot_v4.6.3-stable_win64_console.exe" --path "F:/slam-voxel-world/godot_viewer" --position -10000,-10000 --resolution 1280x720 --quit-after 200 -- --world=F:/slam-voxel-world/out/apt_completed.vxw --snapshot=F:/slam-voxel-world/out/_t1.png 2>&1 | grep -iE "error|script|world_loaded|snapshot"
```
Expected: 有 `world_loaded` 与 `snapshot ok`;**无 SCRIPT ERROR / parse 行**(混缩进或 API 误用会在此暴露)。看 `_t1.png` 确认场景正常。完后 `rm out/_t1.png`。

- [ ] **Step 4: Commit**
```bash
git add godot_viewer/stereo_rig_controller.gd
git commit -m "feat(rig): StereoRig -> CharacterBody3D + foot-aligned capsule + params"
```

---

### Task 2: 重写 1P 行走为 move_and_slide + 删除射线桩 + 落地/撞墙冒烟 flag

**Files:**
- Modify: `godot_viewer/stereo_rig_controller.gd`(`_apply_keyboard_physics`、删 `_slide_axis`/`_is_grounded`、`_process`→物理移动入 `_physics_process`)
- Modify: `godot_viewer/boot_config.gd`(加 `--test-physics-drop` / `--test-physics-wall`)
- Modify: `godot_viewer/test_hooks_controller.gd`(消费 flag,打结构化日志)

- [ ] **Step 1: 重写 1P 物理段**

把现有 `_apply_keyboard_physics(delta)` 整体替换为(用 CharacterBody3D 速度模型):
```gdscript
# Physics mode (1P): horizontal WASD on yaw plane + gravity/jump via move_and_slide.
func _apply_keyboard_physics(delta: float) -> void:
    var fwd: Vector3 = -transform.basis.z
    var right: Vector3 = transform.basis.x
    fwd.y = 0.0
    right.y = 0.0
    if fwd.length_squared() > 0.0001: fwd = fwd.normalized()
    if right.length_squared() > 0.0001: right = right.normalized()
    var horiz := Vector3.ZERO
    if Input.is_key_pressed(KEY_W): horiz += fwd
    if Input.is_key_pressed(KEY_S): horiz -= fwd
    if Input.is_key_pressed(KEY_A): horiz -= right
    if Input.is_key_pressed(KEY_D): horiz += right
    var spd := rig_speed
    if Input.is_key_pressed(KEY_SHIFT): spd *= fast_multiplier
    if horiz.length_squared() > 0.0001:
        horiz = horiz.normalized() * spd
    velocity.x = horiz.x
    velocity.z = horiz.z
    if is_on_floor():
        if velocity.y < 0.0:
            velocity.y = 0.0
        if Input.is_key_pressed(KEY_SPACE):
            velocity.y = jump_velocity_mps
    else:
        velocity.y -= gravity_mps2 * delta
    move_and_slide()
    # Look rotation: yaw + pitch only (no roll in 1P)
    var dyaw := 0.0
    var dpitch := 0.0
    if Input.is_key_pressed(KEY_LEFT):  dyaw += 1.0
    if Input.is_key_pressed(KEY_RIGHT): dyaw -= 1.0
    if Input.is_key_pressed(KEY_UP):    dpitch += 1.0
    if Input.is_key_pressed(KEY_DOWN):  dpitch -= 1.0
    if dpitch != 0.0:
        rotate_object_local(Vector3.RIGHT, dpitch * keyboard_look_speed * delta)
    if dyaw != 0.0:
        rotate(Vector3.UP, dyaw * keyboard_look_speed * delta)
```

- [ ] **Step 2: 物理移动移到 `_physics_process`**

`move_and_slide` 必须在物理帧。把 `_process(delta)` 改为:物理模式不在 `_process` 里走,改在 `_physics_process`:
```gdscript
func _process(delta: float) -> void:
    if is_seated():
        _sync_stereo()
        return
    if not _physics_mode:
        _apply_keyboard_freefly(delta)
    _sync_stereo()
    _animate_walk(delta)

func _physics_process(delta: float) -> void:
    if _physics_mode and not is_seated():
        _apply_keyboard_physics(delta)
        _sync_stereo()
```

- [ ] **Step 3: 删除射线桩**

删除整个 `func _slide_axis(...)` 与 `func _is_grounded() -> bool:`(已无引用)。

- [ ] **Step 4: 加冒烟 flag**

`boot_config.gd` 变量区加:`var test_physics_drop: bool = false` 和 `var test_physics_wall: bool = false`;在 `parse_args` 的 elif 链加:
```gdscript
        elif arg == "--test-physics-drop":
            test_physics_drop = true
        elif arg == "--test-physics-wall":
            test_physics_wall = true
```

`test_hooks_controller.gd`:仿照现有 `test_*` 钩子,加一个延迟 ~90 物理帧后采样并打日志的逻辑(需要持有 rig 与 logger 引用——复用该控制器既有注入)。新增方法(在该控制器 `_physics_process` 或一个计数器里):
```gdscript
# physics smoke: force 1P, then after warmup log settled state / blockage.
var _phys_frames_left: int = -1
var _phys_start: Vector3 = Vector3.ZERO
var _phys_mode: String = ""

func _maybe_start_physics_smoke() -> void:
    if _cfg.test_physics_drop:
        _phys_mode = "drop"
    elif _cfg.test_physics_wall:
        _phys_mode = "wall"
    else:
        return
    _rig.set_physics_mode(true)
    _phys_frames_left = 90
    _phys_start = _rig.global_position

func _physics_process(_delta: float) -> void:
    if _phys_frames_left < 0:
        return
    if _phys_mode == "wall":
        # drive forward by simulating W via direct velocity nudge is hard; instead
        # push the rig forward each frame and let move_and_slide block it.
        _rig.global_position += -_rig.transform.basis.z * 0.05
        _rig.global_position.y = _phys_start.y      # keep level for the wall test
    _phys_frames_left -= 1
    if _phys_frames_left == 0:
        var p: Vector3 = _rig.global_position
        if _phys_mode == "drop":
            _logger.info("physics_drop", {
                "settled_y": p.y, "on_floor": _rig.is_on_floor(),
                "fell_through": p.y < _phys_start.y - 5.0,
            })
        else:
            var dz: float = (p - _phys_start).length()
            _logger.info("physics_wall", {"travelled_m": dz, "on_floor": _rig.is_on_floor()})
        get_tree().quit()
```
并在该控制器初始化处(它已注入 `_cfg`/`_rig`/`_logger`)调用 `_maybe_start_physics_smoke()`。**实现者注意**:读 `test_hooks_controller.gd` 现有结构,按其既有注入字段名(`_cfg`/`_rig`/`_logger` 等)对齐;若字段名不同则适配。

- [ ] **Step 5: 冒烟——落地不穿、撞墙被挡**

落地:
```
... -- --world=...apt_completed.vxw --view=1p --rig-pose=0,5,0,0,0,0 --test-physics-drop 2>&1 | grep physics_drop
```
Expected: `physics_drop {settled_y:~脚在地面, on_floor:true, fell_through:false}`。

撞墙:把 rig 放在一面墙前朝墙(具体 pose 由实现者用快照定位一面墙;若不便,可退化为"朝任意方向推进 90 帧后 travelled_m 明显小于 90*0.05=4.5m 即说明被挡"):
```
... -- --world=...apt_completed.vxw --view=1p --rig-pose=<near-wall> --test-physics-wall 2>&1 | grep physics_wall
```
Expected: `travelled_m` 远小于 4.5(被墙挡住)。

- [ ] **Step 6: Commit**
```bash
git add godot_viewer/stereo_rig_controller.gd godot_viewer/boot_config.gd godot_viewer/test_hooks_controller.gd
git commit -m "feat(rig): 1P walk via move_and_slide (gravity/jump/swept collision) + physics smoke flags"
```

---

### Task 3: 台阶攀爬(max_step_m)

**Files:**
- Modify: `godot_viewer/stereo_rig_controller.gd`(`_apply_keyboard_physics` 末尾加 step-up 辅助)

- [ ] **Step 1: 加 step-up 辅助**

在 `_apply_keyboard_physics` 的 `move_and_slide()` 之后、look-rotation 之前插入:
```gdscript
    # Step-up assist: if a near-vertical obstacle blocked horizontal motion while
    # grounded, and the same horizontal move is clear when raised by max_step_m,
    # lift the body up onto the step (floor_snap_length pulls us back down on descents).
    if is_on_floor() and horiz.length_squared() > 0.0001:
        var blocked := false
        for i in get_slide_collision_count():
            var n := get_slide_collision(i).get_normal()
            if abs(n.y) < 0.3:          # near-vertical wall/step face
                blocked = true
        if blocked:
            var hstep := Vector3(velocity.x, 0.0, velocity.z) * delta
            var up := Vector3(0.0, max_step_m, 0.0)
            # clear above and clear forward-at-raised-height?
            if not test_move(global_transform, up) \
               and not test_move(Transform3D(global_transform.basis, global_transform.origin + up), hstep):
                global_position += up
```

- [ ] **Step 2: 冒烟——能跨小台阶**

在补全公寓里若难找标准台阶,做最小验证:构造一个带 1 格(0.05m)台阶的小测试世界或就用门槛;退化验证 = 朝有矮坎方向走,`physics_wall` 的 `travelled_m` 比无 step-up 时更大(越过坎)。至少确认无回归(落地/撞墙冒烟仍 OK):
```
... --view=1p --rig-pose=0,5,0,0,0,0 --test-physics-drop 2>&1 | grep physics_drop
```
Expected: 仍 `on_floor:true, fell_through:false`。

- [ ] **Step 3: Commit**
```bash
git add godot_viewer/stereo_rig_controller.gd
git commit -m "feat(rig): step-up assist for climbing small ledges (max_step_m)"
```

---

### Task 4: 蹲下(Ctrl 缩胶囊 + 降眼)

**Files:**
- Modify: `godot_viewer/stereo_rig_controller.gd`(`_apply_keyboard_physics` 内加 crouch;用 `_body_shape` 引用)

- [ ] **Step 1: 加 crouch 状态与切换**

变量区加 `var _crouched: bool = false`。在 `_apply_keyboard_physics` 顶部(算 horiz 之前)插入:
```gdscript
    var want_crouch := Input.is_key_pressed(KEY_CTRL)
    if want_crouch and not _crouched:
        _set_capsule_height(capsule_crouch_h_m)
        _crouched = true
    elif not want_crouch and _crouched:
        # only stand if there is headroom (raising back to stand height is clear)
        var dh := capsule_stand_h_m - capsule_crouch_h_m
        if not test_move(global_transform, Vector3(0.0, dh, 0.0)):
            _set_capsule_height(capsule_stand_h_m)
            _crouched = false
```
加辅助方法:
```gdscript
# Resize the body capsule keeping the FEET fixed; lower/raise the eye (rig origin)
# accordingly so the camera tracks the head.
func _set_capsule_height(h: float) -> void:
    var cap := _body_shape.shape as CapsuleShape3D
    var old_h: float = cap.height
    cap.height = h
    _body_shape.position.y = -(eye_height_m - h * 0.5)
    # drop/raise the rig origin (eye) by the height change so feet stay put
    global_position.y += (h - old_h) * 0.5
```
（说明:脚底固定 = 胶囊底不动;胶囊高变化 Δh,则中心与原点上移 Δh/2,故脚不动。）

- [ ] **Step 2: 冒烟——蹲下不崩、仍在地面**

```
... --view=1p --rig-pose=0,5,0,0,0,0 --test-physics-drop 2>&1 | grep -E "physics_drop|error|script"
```
Expected: 无 SCRIPT ERROR;`physics_drop on_floor:true`。(crouch 的交互手感由人工快照确认。)

- [ ] **Step 3: Commit**
```bash
git add godot_viewer/stereo_rig_controller.gd
git commit -m "feat(rig): crouch (Ctrl) — shrink capsule, lower eye, stand only with headroom"
```

---

### Task 5: 核对 reset_pose / sit / spawn 偏移 + 公寓走查 + 回归

**Files:**
- Modify: `godot_viewer/stereo_rig_controller.gd`(如需:`reset_pose`/`set_pose`/sit 偏移按"原点=眼、脚=原点−eye_height"语义核对)

- [ ] **Step 1: 核对落生/坐姿不穿地**

读 `reset_pose`、`set_pose`、sit 相关(`_SIT_LIFT_ABOVE_ENTITY_CENTRE` 等),确认改胶囊偏移后:`reset_pose` 把原点(眼)置于 `spawn_y_m`,脚在 `spawn_y_m - eye_height_m`;若默认会把脚塞进地面,调 `spawn_y_m` 默认或 reset 逻辑使脚略高于地面(让重力落地)。sit 模式不调 move_and_slide(已 `is_seated()` 短路),确认坐姿视觉不变。

- [ ] **Step 2: 公寓 1P 走查快照(人工)**

```
"/f/Godot/Godot_v4.6.3-stable_win64.exe" --path "F:/slam-voxel-world/godot_viewer" -- --world=F:/slam-voxel-world/out/apt_completed.vxw --view=1p
```
人工:WASD 走动应被墙挡、踩在地板上不下坠、Space 跳、Ctrl 蹲、Shift 冲刺;Tab 切 3P 仍自由飞。

- [ ] **Step 3: Python 回归(确认无牵连)**

Run: `pixi run python -m pytest tests/test_vlayer_overlay.py tests/test_vlayer_compose.py tests/test_vlayer_pipeline.py tests/test_floor_fill.py tests/test_vlayer_export.py tests/test_obsmap.py -q`
Expected: 全 PASS(本特性纯 GDScript,Python 不受影响——回归守护)。

- [ ] **Step 4: Commit**
```bash
git add godot_viewer/stereo_rig_controller.gd
git commit -m "fix(rig): reconcile reset_pose/sit/spawn with foot/eye capsule offset"
```

---

## 后续(本计划之外)
- 把玩家胶囊放到独立 collision_layer,核对 `entity_selector` 选取射线 mask 不含玩家层(spec §5 风险)。
- 真台阶世界做 step-up 定量验证;floor_snap_length / max_step_m 调参。
- 头部碰撞细节、贴墙滑动手感、跳跃高度调参。
