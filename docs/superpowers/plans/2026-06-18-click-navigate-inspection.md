# 双击导航巡检相机 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 3P 巡检相机加"双击表面 → 环绕中心平滑飞到该点",随后复用现成右键环视 + 滚轮缩放就近巡检(查漏洞/看模型)。

**Architecture:** 改 `godot_viewer/camera_controller.gd`:加公开方法 `navigate_to(world_point)`(设目标 + 平滑 lerp rig 原点=环绕中心)+ `_process` 内插值 + LMB 双击 `_input` 处理(光标射线命中→调 navigate_to)。`boot_config`/`test_hooks_controller` 加 `--test-nav=x,y,z` 做 headless 冒烟。仅 3P 生效。

**Tech Stack:** Godot 4.6.3 GDScript;`F:/Godot/Godot_v4.6.3-stable_win64_console.exe`;冒烟世界 `out/apt_completed.vxw`。无 Python 改动。

**Setup:** 直接在 `main` 上做。每任务一 commit,结尾 `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`。

启动模板:
```
"/f/Godot/Godot_v4.6.3-stable_win64_console.exe" --path "F:/slam-voxel-world/godot_viewer" --position -10000,-10000 --resolution 1280x720 --quit-after 200 -- --world=F:/slam-voxel-world/out/apt_completed.vxw <FLAGS>
```

---

### Task 1: `navigate_to` + 平滑飞行 + `--test-nav` headless 冒烟

**Files:**
- Modify: `godot_viewer/camera_controller.gd`(export + nav 变量 + `navigate_to` + `_process` lerp)
- Modify: `godot_viewer/boot_config.gd`(`--test-nav=x,y,z`)
- Modify: `godot_viewer/test_hooks_controller.gd`(消费 flag → 调 navigate_to → 打日志)

- [ ] **Step 1: camera_controller 加 export + 变量 + `navigate_to`**

在 `@export var zoom_step ...` 之后加:
```gdscript
@export var nav_fly_time: float = 0.3   # seconds to fly the orbit centre to a clicked point
```
在 `var _orbit_enabled ...` 附近的变量区加:
```gdscript
var _nav_active: bool = false
var _nav_from: Vector3 = Vector3.ZERO
var _nav_to: Vector3 = Vector3.ZERO
var _nav_t: float = 0.0
```
加公开方法(放在 `_process` 之前):
```gdscript
# Fly the orbit centre (the rig origin) smoothly to a world point. Used by
# double-click navigation and by the --test-nav smoke hook. 3P only.
func navigate_to(world_point: Vector3) -> void:
    if view_mode != ViewMode.THIRD_PERSON or _rig_ctl == null:
        return
    _nav_from = _rig_ctl.global_position
    _nav_to = world_point
    _nav_t = 0.0
    _nav_active = true
```

- [ ] **Step 2: `_process` 内插值(3P 分支,算 offset 之前)**

在 `_process` 的 `else:`(3P)分支最前面、计算 `offset` 之前插入:
```gdscript
        if _nav_active:
            _nav_t = min(1.0, _nav_t + _delta / max(0.0001, nav_fly_time))
            var e: float = _nav_t * _nav_t * (3.0 - 2.0 * _nav_t)   # smoothstep
            _rig_ctl.global_position = _nav_from.lerp(_nav_to, e)
            if _nav_t >= 1.0:
                _nav_active = false
```
(注:`_process(_delta)` 形参当前名为 `_delta`,直接用。)

- [ ] **Step 3: boot_config 加 `--test-nav`**

变量区加:
```gdscript
var test_nav: Vector3 = Vector3.ZERO
var test_nav_set: bool = false
```
`parse_args` elif 链加:
```gdscript
        elif arg.begins_with("--test-nav="):
            var np := arg.substr("--test-nav=".length()).split(",")
            if np.size() == 3:
                test_nav = Vector3(float(np[0]), float(np[1]), float(np[2]))
                test_nav_set = true
```

- [ ] **Step 4: test_hooks_controller 消费 flag**

先用 Read 看 `test_hooks_controller.gd` 现有结构(它已注入 `_cfg` 及各控制器;确认相机控制器的注入字段名,如 `_cam_ctl`/`_camera`——按实际命名)。仿现有 `test_*` 钩子,加:计数器在 N 帧后采样并打日志。示例(字段名按实际适配):
```gdscript
var _nav_frames_left: int = -1

# call this from the controller's existing init/_ready hook block, after deps injected
func _maybe_start_nav_smoke() -> void:
    if _cfg.test_nav_set and _cam_ctl != null:
        _cam_ctl.set_view_mode_str("3p")
        _cam_ctl.navigate_to(_cfg.test_nav)
        _nav_frames_left = 30

func _process(_delta: float) -> void:
    if _nav_frames_left < 0:
        return
    _nav_frames_left -= 1
    if _nav_frames_left == 0:
        var p: Vector3 = _rig_ctl.global_position   # rig = orbit centre
        var t: Vector3 = _cfg.test_nav
        _logger.info("nav", {
            "reached": p.distance_to(t) < 0.05,
            "pos": [p.x, p.y, p.z], "target": [t.x, t.y, t.z],
        })
        get_tree().quit()
```
**实现者注意**:`test_hooks_controller.gd` 可能已有 `_process` 或集中调用钩子的地方 —— 合并进去,别重复定义 `_process`;`_rig_ctl`/`_cam_ctl`/`_logger`/`_cfg` 用该文件实际的字段名。若该控制器没持有 rig 引用,改用 `_cam_ctl` 暴露的 orbit 中心或新增注入。

- [ ] **Step 5: 冒烟——navigate_to 抵达目标**

Run:
```
"/f/Godot/Godot_v4.6.3-stable_win64_console.exe" --path "F:/slam-voxel-world/godot_viewer" --position -10000,-10000 --resolution 1280x720 --quit-after 200 -- --world=F:/slam-voxel-world/out/apt_completed.vxw --test-nav=1.5,0.5,1.5 2>&1 | grep -iE "nav|error|script"
```
Expected: `nav {reached:true, pos:[~1.5,0.5,1.5], target:[1.5,0.5,1.5]}`;无 SCRIPT ERROR。

- [ ] **Step 6: Commit**
```bash
git add godot_viewer/camera_controller.gd godot_viewer/boot_config.gd godot_viewer/test_hooks_controller.gd
git commit -m "feat(camera): navigate_to smooth orbit-centre fly + --test-nav smoke hook"
```

---

### Task 2: LMB 双击 → 射线命中 → 导航

**Files:**
- Modify: `godot_viewer/camera_controller.gd`(`_input` 加双击处理)

- [ ] **Step 1: `_input` 加双击导航(3P)**

在 `_input` 的 `if event is InputEventMouseButton:` 块内,处理右键/滚轮之前,加 LMB 双击分支:
```gdscript
        if event.button_index == MOUSE_BUTTON_LEFT and event.pressed and event.double_click \
           and view_mode == ViewMode.THIRD_PERSON and _orbit_enabled:
            var mp := get_viewport().get_mouse_position()
            var from := _cam.project_ray_origin(mp)
            var dir := _cam.project_ray_normal(mp)
            var q := PhysicsRayQueryParameters3D.create(from, from + dir * 1000.0)
            var hit := _cam.get_world_3d().direct_space_state.intersect_ray(q)
            if not hit.is_empty():
                navigate_to(hit.position)
                get_viewport().set_input_as_handled()   # don't let entity_selector treat it as a pick
            return
```
（放在 `if event.button_index == MOUSE_BUTTON_RIGHT:` 判断之前;保持其余 RMB/滚轮逻辑不变。`_orbit_enabled` 为假(编辑模式)时不导航。)

- [ ] **Step 2: 冒烟——无回归(解析/加载/选取)**

Run（确认无 SCRIPT ERROR、世界仍加载;Task 1 的 nav 冒烟仍通过):
```
"/f/Godot/Godot_v4.6.3-stable_win64_console.exe" --path "F:/slam-voxel-world/godot_viewer" --position -10000,-10000 --resolution 1280x720 --quit-after 200 -- --world=F:/slam-voxel-world/out/apt_completed.vxw --test-nav=2,0.5,2 2>&1 | grep -iE "nav|error|script"
```
Expected: `nav reached:true`;无报错。

- [ ] **Step 3: 人工双击验证(交互)**

Run:
```
"/f/Godot/Godot_v4.6.3-stable_win64.exe" --path "F:/slam-voxel-world/godot_viewer" -- --world=F:/slam-voxel-world/out/apt_completed.vxw
```
3P 下双击墙/地/家具 → 环绕中心平滑飞到该点;右键拖拽可环视、滚轮缩放;单击仍能选实体。

- [ ] **Step 4: Python 回归(确认无牵连)**
Run: `pixi run python -m pytest tests/test_vlayer_overlay.py tests/test_obsmap.py -q`
Expected: PASS(纯 GDScript 改动,Python 不受影响)。

- [ ] **Step 5: Commit**
```bash
git add godot_viewer/camera_controller.gd
git commit -m "feat(camera): double-click surface to navigate orbit centre there (3P inspection)"
```

---

## 后续(本计划之外)
- 双击窗口内抑制 entity_selector 单击(避免第一击误选)。
- 导航中按 WASD 即取消导航(手感)。
- 可选:双击实体 → 飞到并自动设合适环绕距离(按实体 OBB 取景)。
