# 双击导航巡检相机(3P)

> 日期:2026-06-18。设计文档(spec)。
> 解决的问题:巡检场景(找几何漏洞、看物体模型是否正确)时,靠操控人物走动很不方便。增加"**双击表面 → 环绕中心平滑飞到该点**",随后复用现成的右键环视 + 滚轮缩放就近巡检。

## 1. 架构

主要改 `godot_viewer/camera_controller.gd`。现状:3P(THIRD_PERSON)相机绕 **rig 原点**(`_rig_ctl.global_transform.origin`)环绕,右键拖拽转 orbit、滚轮缩放(`_process` 末尾的 orbit 计算)。本功能把"双击点"设为新的环绕中心 —— 即把 rig 原点平滑移到该点(相机自然跟着重新取景)。

- 仅 **3P** 生效(1P 光标锁定、第一人称,不参与)。
- 复用既有 orbit/zoom,无需新相机逻辑。
- LMB 单击仍归 `entity_selector`(选/取消实体);本功能用 **LMB 双击**,且把双击事件标记为已处理,避免误触发选取。

## 2. 行为与组件

**输入(`camera_controller._input`,仅 3P):**
- `InputEventMouseButton`、`button_index == MOUSE_BUTTON_LEFT`、`event.double_click == true`:
  1. `var mp := get_viewport().get_mouse_position()`
  2. `var from := _cam.project_ray_origin(mp)`;`var dir := _cam.project_ray_normal(mp)`
  3. `var q := PhysicsRayQueryParameters3D.create(from, from + dir * 1000.0)`;`var hit := _cam.get_world_3d().direct_space_state.intersect_ray(q)`
  4. 命中(`hit` 非空)→ `_nav_from = _rig_ctl.global_position`;`_nav_to = hit.position`;`_nav_t = 0.0`;`_nav_active = true`;`get_viewport().set_input_as_handled()`(阻断 entity_selector 对双击的处理)。
  5. 未命中 → 忽略。

**运动(`camera_controller._process(delta)`,3P 分支内,在算 orbit 之前):**
```gdscript
if _nav_active:
    _nav_t = min(1.0, _nav_t + delta / max(0.0001, nav_fly_time))
    var e := _nav_t * _nav_t * (3.0 - 2.0 * _nav_t)   # smoothstep ease
    _rig_ctl.global_position = _nav_from.lerp(_nav_to, e)
    if _nav_t >= 1.0:
        _nav_active = false
```
其余 orbit 计算照旧(相机绕 `_rig_ctl.global_transform.origin`),所以中心移动 = 取景平滑跟随。`nav_fly_time = 0` 时第一帧即到位(瞬移)。

**新增 @export / 变量:**
- `@export var nav_fly_time: float = 0.3`
- `var _nav_active := false`、`var _nav_from := Vector3.ZERO`、`var _nav_to := Vector3.ZERO`、`var _nav_t := 0.0`

## 3. 测试策略
GDScript 集成 → headless 冒烟为主:
- **`--test-nav=x,y,z` flag**(`boot_config` + `test_hooks_controller`):强制 3P、把鼠标/射线步骤旁路,直接调一个新公开方法 `camera_controller.navigate_to(Vector3)`(双击命中后也调它),跑 ~30 帧后打日志 `nav {reached:bool, pos:[...], target:[...]}`;pytest/grep 断言 `reached` 且 `pos≈target`。
- 把"设目标 + lerp"抽到 `navigate_to(world_point)` 公开方法,双击命中后调用它 —— 既便于测试,又把射线/输入与运动解耦(单一职责)。
- 人工:补全公寓里双击墙/家具,确认飞到位、能右键环视。

## 4. 模块边界
- `godot_viewer/camera_controller.gd`:双击 `_input` 处理 + `navigate_to(Vector3)` 公开方法 + `_process` 内 lerp + export。
- `godot_viewer/boot_config.gd`:加 `--test-nav=x,y,z` 解析(`var test_nav: Vector3`、`var test_nav_set: bool`)。
- `godot_viewer/test_hooks_controller.gd`:消费 flag → 调 `navigate_to` + 延迟打日志。
- 不动 entity_selector / rig / vlayer。

## 5. 风险 / 待定
- **射线命中层**:体素 `VoxelCollisionBody`(StaticBody3D)与实体代理(RigidBody3D)都在默认层 1,默认 intersect_ray mask 命中两者 → OK。
- **双击的第一击**:仍会触发一次 entity_selector 单击(选/取消)。可接受;若干扰巡检,后续加"双击窗口内抑制单击"。
- **导航中按 WASD**:3P 下 rig 仍可被 WASD 移动,会与 lerp 叠加;0.3s 内影响极小,不处理(或后续:有输入即取消导航)。
- **edit mode**:编辑模式下 RMB 归编辑器(已有 `_orbit_enabled`);双击导航与编辑放置是否冲突由实现者确认(编辑模式下可禁用双击导航)。
