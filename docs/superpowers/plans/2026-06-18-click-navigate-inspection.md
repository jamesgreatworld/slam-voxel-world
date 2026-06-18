# 双击导航巡检相机 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 3P 巡检相机加"双击表面 → 环绕中心平滑飞到该点",随后复用现成右键环视 + 滚轮缩放就近巡检(查漏洞/看模型)。

**Architecture:** 仅改 `godot_viewer/camera_controller.gd`:加公开方法 `navigate_to(world_point)`(设目标 + `_process` 内平滑 lerp rig 原点=环绕中心)+ LMB 双击 `_input` 处理(光标射线命中→调 navigate_to)。仅 3P 生效。**不动** boot_config/test_hooks/main.gd(测试钩子注入成本过高,见验证小节)。

**Tech Stack:** Godot 4.6.3 GDScript;`F:/Godot/Godot_v4.6.3-stable_win64_console.exe`;冒烟世界 `out/apt_completed.vxw`。无 Python 改动。

**Setup:** 直接在 `main` 上做。commit 结尾 `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`。

**验证说明:** 这是交互式巡检相机,headless 无法模拟真实双击命中。自动验证 = headless 启动**无 SCRIPT ERROR + world_loaded**(解析/启动 sanity);功能验收 = **人工双击**(符合项目既有"manual snapshot"验证风格 + 轻量验证偏好)。

---

### Task 1: `navigate_to` + 平滑飞行 + 双击导航(camera_controller 单文件)

**Files:**
- Modify: `godot_viewer/camera_controller.gd`

- [ ] **Step 1: 加 export + nav 变量 + `navigate_to`**

在 `@export var zoom_step: float = 1.0` 之后加:
```gdscript
@export var nav_fly_time: float = 0.3   # seconds to fly the orbit centre to a clicked point
```
在变量区(`var _orbit_enabled ...` 附近)加:
```gdscript
var _nav_active: bool = false
var _nav_from: Vector3 = Vector3.ZERO
var _nav_to: Vector3 = Vector3.ZERO
var _nav_t: float = 0.0
```
在 `func _process` 之前加公开方法:
```gdscript
# Fly the orbit centre (the rig origin) smoothly to a world point. 3P only.
# Called by double-click navigation. The camera keeps orbiting the (moving)
# centre, so the view reframes onto the clicked point.
func navigate_to(world_point: Vector3) -> void:
    if view_mode != ViewMode.THIRD_PERSON or _rig_ctl == null:
        return
    _nav_from = _rig_ctl.global_position
    _nav_to = world_point
    _nav_t = 0.0
    _nav_active = true
```

- [ ] **Step 2: `_process` 3P 分支内插值(算 offset 之前)**

当前 `_process(_delta)` 的 `else:`(3P)分支最前面、计算 `var offset` 之前,插入:
```gdscript
        if _nav_active:
            _nav_t = min(1.0, _nav_t + _delta / max(0.0001, nav_fly_time))
            var ease: float = _nav_t * _nav_t * (3.0 - 2.0 * _nav_t)   # smoothstep
            _rig_ctl.global_position = _nav_from.lerp(_nav_to, ease)
            if _nav_t >= 1.0:
                _nav_active = false
```

- [ ] **Step 3: `_input` 加 LMB 双击导航(3P)**

在 `_input` 内 `if event is InputEventMouseButton:` 块的最前面(处理 RIGHT/滚轮之前)插入:
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
                get_viewport().set_input_as_handled()   # don't let entity_selector pick on the double-click
            return
```
（放在 `if event.button_index == MOUSE_BUTTON_RIGHT:` 之前;RMB/滚轮逻辑不变。编辑模式 `_orbit_enabled=false` 时不导航。）

- [ ] **Step 4: 冒烟——解析/启动 sanity**

Run:
```
"/f/Godot/Godot_v4.6.3-stable_win64_console.exe" --path "F:/slam-voxel-world/godot_viewer" --position -10000,-10000 --resolution 1280x720 --quit-after 200 -- --world=F:/slam-voxel-world/out/apt_completed.vxw --snapshot=F:/slam-voxel-world/out/_nav_t1.png 2>&1 | grep -iE "error|script|world_loaded|snapshot"
```
Expected: `world_loaded` + `snapshot ok`;**无 SCRIPT ERROR / parse 行**。完后 `rm out/_nav_t1.png`。

- [ ] **Step 5: Commit**
```bash
git add godot_viewer/camera_controller.gd
git commit -m "feat(camera): double-click surface to fly orbit centre there (3P inspection navigation)"
```

---

### Task 2: 人工验收 + Python 回归

**Files:** 无(验证)

- [ ] **Step 1: 人工双击验证(交互)**
```
"/f/Godot/Godot_v4.6.3-stable_win64.exe" --path "F:/slam-voxel-world/godot_viewer" -- --world=F:/slam-voxel-world/out/apt_completed.vxw
```
3P 下:双击墙/地/家具 → 环绕中心平滑飞到该点;右键拖拽环视、滚轮缩放;单击仍能选实体。

- [ ] **Step 2: Python 回归(确认无牵连)**
Run: `pixi run python -m pytest tests/test_vlayer_overlay.py tests/test_obsmap.py -q`
Expected: PASS(纯 GDScript 改动,Python 不受影响)。

---

## 后续(本计划之外)
- 双击窗口内抑制 entity_selector 单击(避免第一击误选)。
- 导航中按 WASD 即取消导航(手感)。
- 双击实体 → 飞到并按实体 OBB 自动取景。
- 若需自动化:给 test_hooks 注入相机 + 加 `--test-nav` 帧计数冒烟。
