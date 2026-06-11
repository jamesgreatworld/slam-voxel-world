# main.gd 拆分 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 822 行的 `godot_viewer/main.gd` 拆成 6 个单一职责 controller + ~170 行编排瘦壳,零行为变化(spec: `docs/superpowers/specs/2026-06-11-main-gd-split-design.md`)。

**Architecture:** 纯搬家重构。每个 task 抽出一类职责到新文件,main.gd 同步收缩;controllers 之间不互相 import,跨模块协作经信号(`world_session.world_loaded`、`entity_subsystem.menu_requested`)或由 main 注入引用。新 controller 全部由 main 在 `_ready` 动态创建(沿用 entity 节点的 set_script 模式,不动 main.tscn)。

**Tech Stack:** Godot 4.6 GDScript。验证从简(用户确认):每个 task 只做 10 秒启动检查(抓 SCRIPT ERROR),最后做一次总体冒烟。

**关键约定(执行前必读):**
- 本计划中"搬入"= 从 main.gd 剪切函数体,粘贴到新文件,只做计划中列出的改名/引用替换,**不改逻辑**。
- main.gd 当前内容以 commit `efffd3f` 时点为准(822 行)。各 task 给出的行号引用该版本;如有漂移以函数名定位。
- 每个 task 结束跑一次启动检查:
  ```powershell
  F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer --position -10000,-10000 --resolution 1280x720 --quit-after 10 -- --world=F:/slam-voxel-world/out/baseline_20cm.vxw
  ```
  控制台无 `SCRIPT ERROR` / `Parse Error` 即过;有就修完再 commit。

---

### Task 1: boot_config.gd — CLI 解析出壳

**Files:**
- Create: `godot_viewer/boot_config.gd`
- Modify: `godot_viewer/main.gd`(`_ready` 的 arg 循环 83-129 行、`_test_*` 变量 60-74 行、`_parse_rig_pose` 366-376 行)

- [ ] **Step 1: 创建 boot_config.gd(完整内容如下)**

```gdscript
# boot_config.gd — parses CLI user args into a plain config object.
# Pure data + parsing: no side effects, no scene access. main.gd builds one
# at boot; test_hooks_controller.gd consumes the test_* fields.
extends RefCounted

var raw_args: PackedStringArray = PackedStringArray()

var world_path: String = "../out/baseline.vxw"
var view_override: String = ""        # "" | "1p" | "3p"
var rig_pose_spec: String = ""        # "x,y,z,yaw,pitch,roll"
var open_item_picker: bool = false
var spawn_items: Array = []

var test_delete_first: bool = false
var test_rotate_first_deg: float = 0.0
var test_grab_first_to: Vector3 = Vector3.ZERO
var test_grab_first_set: bool = false
var test_undo_times: int = 0
var test_duplicate_first: bool = false
var test_snapshot_world: bool = false
var test_toggle_behavior_on_first: bool = false
var test_hide_voxel_set: bool = false
var test_hide_voxel_vi: Vector3i = Vector3i.ZERO
var test_toggle_day_night: bool = false


func parse_args(args: PackedStringArray) -> void:
    raw_args = args
    for arg in args:
        if arg.begins_with("--world="):
            world_path = arg.substr("--world=".length())
        elif arg == "--view=1p":
            view_override = "1p"
        elif arg == "--view=3p":
            view_override = "3p"
        elif arg.begins_with("--rig-pose="):
            rig_pose_spec = arg.substr("--rig-pose=".length())
        elif arg == "--open-item-picker":
            open_item_picker = true
        elif arg.begins_with("--spawn-items="):
            spawn_items = arg.substr("--spawn-items=".length()).split(",")
        elif arg == "--test-delete-first":
            test_delete_first = true
        elif arg.begins_with("--test-rotate-first="):
            test_rotate_first_deg = float(arg.substr("--test-rotate-first=".length()))
        elif arg.begins_with("--test-grab-first-to="):
            var parts := arg.substr("--test-grab-first-to=".length()).split(",")
            if parts.size() == 3:
                test_grab_first_to = Vector3(
                    float(parts[0]), float(parts[1]), float(parts[2])
                )
                test_grab_first_set = true
        elif arg.begins_with("--test-undo-times="):
            test_undo_times = int(arg.substr("--test-undo-times=".length()))
        elif arg == "--test-duplicate-first":
            test_duplicate_first = true
        elif arg == "--test-snapshot-world":
            test_snapshot_world = true
        elif arg == "--test-toggle-behavior-on-first":
            test_toggle_behavior_on_first = true
        elif arg.begins_with("--test-hide-voxel="):
            var hv_parts := arg.substr("--test-hide-voxel=".length()).split(",")
            if hv_parts.size() == 3:
                test_hide_voxel_vi = Vector3i(
                    int(hv_parts[0]), int(hv_parts[1]), int(hv_parts[2])
                )
                test_hide_voxel_set = true
        elif arg == "--test-toggle-day-night":
            test_toggle_day_night = true


static func parse_rig_pose(spec: String) -> Transform3D:
    var parts := spec.split(",")
    if parts.size() != 6:
        push_error("[boot_config] --rig-pose needs 6 comma-separated numbers, got %d" % parts.size())
        return Transform3D.IDENTITY
    var p := Vector3(float(parts[0]), float(parts[1]), float(parts[2]))
    var basis := Basis()
    basis = basis.rotated(Vector3.UP, deg_to_rad(float(parts[3])))     # yaw
    basis = basis.rotated(basis.x, deg_to_rad(float(parts[4])))        # pitch (local X)
    basis = basis.rotated(basis.z, deg_to_rad(float(parts[5])))        # roll (local Z)
    return Transform3D(basis, p)
```

- [ ] **Step 2: main.gd 接入 cfg**

1. 顶部 const 区加:`const BootConfig = preload("res://boot_config.gd")`;删除 `const DEFAULT_WORLD_PATH := "../out/baseline.vxw"`(缺省值已进 boot_config)。
2. 删除成员变量 `_test_delete_first` 到 `_test_toggle_day_night`(60-74 行那一组)。加 `var cfg  # BootConfig`。
3. `_ready` 开头的局部变量声明 + arg 循环(83-129 行)整体替换为:
   ```gdscript
   cfg = BootConfig.new()
   cfg.parse_args(OS.get_cmdline_user_args())
   ```
4. main.gd 全文替换引用(用编辑器全局替换,逐一确认):
   - `world_path`(_ready 局部)→ `cfg.world_path`;`view_override` → `cfg.view_override`;`rig_pose_spec` → `cfg.rig_pose_spec`;`open_item_picker` → `cfg.open_item_picker`;`spawn_items` → `cfg.spawn_items`
   - `_test_xxx` → `cfg.test_xxx`(所有 11 个字段)
   - `snap_ctl.configure_from_cli(args)` → `snap_ctl.configure_from_cli(cfg.raw_args)`
   - `_parse_rig_pose(` → `BootConfig.parse_rig_pose(`;删除 main.gd 的 `_parse_rig_pose` 函数。

- [ ] **Step 3: 启动检查(见顶部命令),无 SCRIPT ERROR**

- [ ] **Step 4: Commit**

```bash
git add godot_viewer/boot_config.gd godot_viewer/main.gd
git commit -m "refactor: extract boot_config.gd from main.gd (CLI parsing)"
```

---

### Task 2: environment_controller.gd — Day/Night 出壳

**Files:**
- Create: `godot_viewer/environment_controller.gd`
- Modify: `godot_viewer/main.gd`(514-609 行的 Day/Night 区块)

- [ ] **Step 1: 创建 environment_controller.gd(完整内容如下)**

```gdscript
# environment_controller.gd — Day/Night visual state.
# Tweaks DirectionalLight3D + WorldEnvironment.environment (+ procedural sky
# material) between two hardcoded palettes. The Day values mirror what
# main.tscn bakes in so a fresh launch is visually identical. Lamp / computer
# emission_energy_multiplier on entity meshes keeps them readable in Night.
extends Node

const _DAY_LIGHT_ENERGY: float = 1.6
const _DAY_LIGHT_COLOR: Color = Color(1, 1, 1)
const _DAY_AMBIENT_COLOR: Color = Color(0.4, 0.45, 0.55, 1)
const _DAY_AMBIENT_ENERGY: float = 0.25
const _DAY_SKY_TOP: Color = Color(0.4, 0.5, 0.65, 1)
const _DAY_SKY_HORIZON: Color = Color(0.65, 0.7, 0.75, 1)
const _DAY_GROUND_BOTTOM: Color = Color(0.1, 0.1, 0.1, 1)
const _DAY_GROUND_HORIZON: Color = Color(0.45, 0.4, 0.35, 1)

const _NIGHT_LIGHT_ENERGY: float = 0.55
const _NIGHT_LIGHT_COLOR: Color = Color(0.82, 0.88, 1.0)
const _NIGHT_AMBIENT_COLOR: Color = Color(0.20, 0.24, 0.34, 1)
const _NIGHT_AMBIENT_ENERGY: float = 0.12
const _NIGHT_SKY_TOP: Color = Color(0.05, 0.07, 0.16, 1)
const _NIGHT_SKY_HORIZON: Color = Color(0.12, 0.16, 0.26, 1)
const _NIGHT_GROUND_BOTTOM: Color = Color(0.02, 0.02, 0.05, 1)
const _NIGHT_GROUND_HORIZON: Color = Color(0.08, 0.10, 0.16, 1)
# Sharper shadows in moonlight — real moonlight casts harder edges than the
# soft default. Day mode keeps the default blur.
const _DAY_SHADOW_BLUR: float = 1.0
const _NIGHT_SHADOW_BLUR: float = 0.5

var _light: DirectionalLight3D
var _world_env: WorldEnvironment
var _pause_menu: CanvasLayer
var _logger
# Day/Night state. Default is Day to match the values baked into main.tscn.
var _night_mode: bool = false


func init_controller(light: DirectionalLight3D, world_env: WorldEnvironment,
        pause_menu: CanvasLayer, logger) -> void:
    _light = light
    _world_env = world_env
    _pause_menu = pause_menu
    _logger = logger


func is_night() -> bool:
    return _night_mode


func toggle_day_night() -> void:
    _night_mode = not _night_mode
    if _night_mode:
        _apply_night_mode()
    else:
        _apply_day_mode()
    if _pause_menu != null:
        _pause_menu.set_day_night_label(_night_mode)
    if _logger != null:
        var le := 0.0
        if _light != null:
            le = _light.light_energy
        _logger.info("day_night_toggle", {
            "night_mode": _night_mode,
            "directional_light_energy": le,
        })


func _get_sky_material() -> ProceduralSkyMaterial:
    if _world_env == null or _world_env.environment == null:
        return null
    var sky: Sky = _world_env.environment.sky
    if sky == null:
        return null
    var mat = sky.sky_material
    if mat is ProceduralSkyMaterial:
        return mat
    return null


func _apply_day_mode() -> void:
    if _light != null:
        _light.light_energy = _DAY_LIGHT_ENERGY
        _light.light_color = _DAY_LIGHT_COLOR
        _light.shadow_blur = _DAY_SHADOW_BLUR
    if _world_env != null and _world_env.environment != null:
        _world_env.environment.ambient_light_color = _DAY_AMBIENT_COLOR
        _world_env.environment.ambient_light_energy = _DAY_AMBIENT_ENERGY
    var sky_mat: ProceduralSkyMaterial = _get_sky_material()
    if sky_mat != null:
        sky_mat.sky_top_color = _DAY_SKY_TOP
        sky_mat.sky_horizon_color = _DAY_SKY_HORIZON
        sky_mat.ground_bottom_color = _DAY_GROUND_BOTTOM
        sky_mat.ground_horizon_color = _DAY_GROUND_HORIZON


func _apply_night_mode() -> void:
    if _light != null:
        _light.light_energy = _NIGHT_LIGHT_ENERGY
        _light.light_color = _NIGHT_LIGHT_COLOR
        _light.shadow_blur = _NIGHT_SHADOW_BLUR
    if _world_env != null and _world_env.environment != null:
        _world_env.environment.ambient_light_color = _NIGHT_AMBIENT_COLOR
        _world_env.environment.ambient_light_energy = _NIGHT_AMBIENT_ENERGY
    var sky_mat: ProceduralSkyMaterial = _get_sky_material()
    if sky_mat != null:
        sky_mat.sky_top_color = _NIGHT_SKY_TOP
        sky_mat.sky_horizon_color = _NIGHT_SKY_HORIZON
        sky_mat.ground_bottom_color = _NIGHT_GROUND_BOTTOM
        sky_mat.ground_horizon_color = _NIGHT_GROUND_HORIZON
```

- [ ] **Step 2: main.gd 移除 Day/Night、接入 env_ctl**

1. 删除 main.gd 514-609 行整个 Day/Night 区块(`_DAY_*`/`_NIGHT_*` 常量、`_night_mode`、`_get_sky_material`、`_apply_day_mode`、`_apply_night_mode`、`_toggle_day_night`、`_on_toggle_day_night`)。`var _night_mode`(77 行)也删。
2. 顶部加 `const EnvironmentControllerScript = preload("res://environment_controller.gd")` 和 `var env_ctl: Node`。
3. `_ready` 中 `logger = VxwLogger.new()` 之后插入:
   ```gdscript
   env_ctl = Node.new()
   env_ctl.set_script(EnvironmentControllerScript)
   env_ctl.name = "EnvironmentController"
   add_child(env_ctl)
   env_ctl.init_controller(directional_light, world_env, pause_menu, logger)
   ```
4. `_wire_pause_menu` 中 `pause_menu.toggle_day_night_requested.connect(_on_toggle_day_night)` 改为:
   ```gdscript
   pause_menu.toggle_day_night_requested.connect(func():
       env_ctl.toggle_day_night()
       _close_pause()
   )
   ```
5. `_ready` 末尾测试钩子 `if cfg.test_toggle_day_night:` 块中 `_toggle_day_night()` → `env_ctl.toggle_day_night()`。

- [ ] **Step 3: 启动检查,无 SCRIPT ERROR**

- [ ] **Step 4: Commit**

```bash
git add godot_viewer/environment_controller.gd godot_viewer/main.gd
git commit -m "refactor: extract environment_controller.gd (day/night) from main.gd"
```

---

### Task 3: world_session.gd — 世界生命周期出壳

**Files:**
- Create: `godot_viewer/world_session.gd`
- Modify: `godot_viewer/main.gd`(`_resolve`、`_save_world_snapshot`、`_on_save_world_backup`、`_on_reload_world`、`_on_load_world_requested`、`_load_world_in_place`、`_copy_dir_recursive`、`_on_import_litematic_requested`、`_world`/`_world_path_absolute` 全部引用)

- [ ] **Step 1: 创建 world_session.gd(完整内容如下;函数体均从 main.gd 原样搬入,只改本计划列出的引用名)**

```gdscript
# world_session.gd — owns the loaded world + its path. Initial load, in-place
# swap, reload, backup, world snapshot and litematic import all live here.
# After every in-place swap it emits world_loaded so other controllers rebind.
# (This signal is the seam hot-reload [architecture.md §12 P1] will reuse.)
extends Node

signal world_loaded(world, path: String)

const VxwLoader = preload("res://vxw_loader.gd")

var world  # VxwLoader.VxwWorld
var world_path: String = ""

var _renderer: Node3D
var _logger


func init_session(renderer: Node3D, logger) -> void:
    _renderer = renderer
    _logger = logger


func resolve(p: String) -> String:
    if p.is_absolute_path():
        return p
    var base := ProjectSettings.globalize_path("res://")
    return base.path_join(p)


# Initial boot load. Returns false when the world is empty/missing; the caller
# decides what to show. Does NOT emit world_loaded — boot wiring in main._ready
# runs in explicit order because the other controllers don't exist yet.
func load_initial(world_path_raw: String) -> bool:
    world_path = resolve(world_path_raw)
    world = VxwLoader.load_world(world_path)
    if world.voxel_count() == 0:
        _logger.error("world", {"reason": "empty load", "path": world_path_raw})
        return false
    _log_loaded()
    _renderer.build(world)
    return true


# Menu "Load world" entry — validate, then swap in place.
func request_load(new_path: String) -> void:
    _logger.info("world_load_requested", {"new_path": new_path, "old_path": world_path})
    if not DirAccess.dir_exists_absolute(new_path):
        _logger.error("world_load_requested", {"reason": "dir missing", "path": new_path})
        return
    if not FileAccess.file_exists(new_path + "/manifest.json"):
        _logger.error("world_load_requested", {"reason": "manifest.json missing in selected dir", "path": new_path})
        return
    load_in_place(new_path)


func reload() -> void:
    _logger.info("world_reload", {"path": world_path})
    load_in_place(world_path)


# In-place world swap: tear down renderer children + rebuild. Listener rebind
# (voxel editor / entities / HUD / rig) happens via the world_loaded signal.
func load_in_place(path: String) -> void:
    var new_world = VxwLoader.load_world(path)
    if new_world.voxel_count() == 0:
        _logger.error("world_load_in_place", {"reason": "empty world", "path": path})
        return
    world = new_world
    world_path = path
    for child in _renderer.get_children():
        child.queue_free()
    _renderer.build(world)
    _log_loaded()
    world_loaded.emit(world, world_path)


func save_backup() -> void:
    var ts := Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
    var src := world_path
    var dir := src.get_base_dir()
    var base := src.get_file()
    if base.ends_with(".vxw"):
        base = base.substr(0, base.length() - 4)
    var dst := "%s/%s_backup_%s.vxw" % [dir, base, ts]
    var ok := _copy_dir_recursive(src, dst)
    _logger.info("world_backup", {"src": src, "dst": dst, "ok": ok})


func save_snapshot() -> void:
    # Snapshot the current world dir into out/snapshots/<base>_<ts>/
    var src := world_path
    if src == "" or not DirAccess.dir_exists_absolute(src):
        if _logger != null:
            _logger.error("world_snapshot", {"reason": "src missing", "src": src})
        return
    var ts := Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
    var base := src.get_file()
    if base.ends_with(".vxw"):
        base = base.substr(0, base.length() - 4)
    var proj_root := ProjectSettings.globalize_path("res://..")
    var dst_root := proj_root + "/out/snapshots"
    DirAccess.make_dir_recursive_absolute(dst_root)
    var dst := "%s/%s_%s" % [dst_root, base, ts]
    var ok := _copy_dir_recursive(src, dst)
    if _logger != null:
        _logger.info("world_snapshot", {"src": src, "dst": dst, "ok": ok})


func import_litematic(path: String) -> void:
    _logger.info("import_litematic", {"src": path})
    var basename: String = path.get_file().get_basename()
    var out_vxw: String = ProjectSettings.globalize_path("res://../out") + "/" + basename + ".vxw"
    var proj_root: String = ProjectSettings.globalize_path("res://..")
    var pixi_cmd := "pixi"
    var args := [
        "run", "python",
        proj_root + "/m3_adapter/litematic_to_vxw.py",
        path, out_vxw,
        "--voxel-size", "1.0",
        "--compression", "gzip",
    ]
    var output: Array = []
    var exit_code: int = OS.execute(pixi_cmd, args, output, true, true)
    if exit_code != 0:
        _logger.error("import_litematic", {"exit_code": exit_code, "output": output})
        return
    _logger.info("import_litematic_ok", {"out": out_vxw})
    load_in_place(out_vxw)


func _log_loaded() -> void:
    _logger.info("world_loaded", {
        "voxels": world.voxel_count(),
        "voxel_size_m": world.voxel_size_meters,
        "chunk_extent": world.chunk_extent,
        "path": world_path,
    })


func _copy_dir_recursive(src: String, dst: String) -> bool:
    if not DirAccess.dir_exists_absolute(src):
        push_error("[world_session] copy_dir source missing: " + src)
        return false
    DirAccess.make_dir_recursive_absolute(dst)
    var d := DirAccess.open(src)
    if d == null:
        return false
    d.list_dir_begin()
    var name := d.get_next()
    while name != "":
        if name == "." or name == "..":
            name = d.get_next()
            continue
        var sp := src + "/" + name
        var dp := dst + "/" + name
        if d.current_is_dir():
            _copy_dir_recursive(sp, dp)
        else:
            DirAccess.copy_absolute(sp, dp)
        name = d.get_next()
    return true
```

- [ ] **Step 2: main.gd 接入 ws**

1. 顶部:加 `const WorldSessionScript = preload("res://world_session.gd")`、`var ws: Node`;删 `var _world`、`var _world_path_absolute`、`const VxwLoader = preload(...)`(main 不再直接读世界;`VxwWriter` 暂留,Task 4 删)。
2. `_ready` 中世界加载段(原 137-149 行)替换为:
   ```gdscript
   ws = Node.new()
   ws.set_script(WorldSessionScript)
   ws.name = "WorldSession"
   add_child(ws)
   ws.init_session(renderer, logger)
   if not ws.load_initial(cfg.world_path):
       status_label.text = "No voxels loaded. Path tried: " + cfg.world_path
       return
   ```
   注意:原 `renderer.build(_world)`(152 行)删除——`load_initial` 已做。
3. `_ready` 控制器初始化段加一行(`_wire_pause_menu()` 之前):`ws.world_loaded.connect(_on_world_loaded)`,并新增 handler:
   ```gdscript
   func _on_world_loaded(world, _path: String) -> void:
       # In-place swap rebind: re-init editor with the new world, reset rig,
       # refresh HUD voxel count / size.
       voxel_editor.init_editor(renderer, main_cam)
       stereo_rig.reset_pose()
       hud_ctl.init_controller(status_label, mode_label, pose_label, world, stereo_rig, cam_ctl, logger)
   ```
4. main.gd 全文替换:`_world_path_absolute` → `ws.world_path`,`_world` → `ws.world`(注意 `hud_ctl.init_controller(..., ws.world, ...)`)。
5. 删除 main.gd 函数:`_resolve`、`_save_world_snapshot`、`_on_save_world_backup`、`_on_reload_world`、`_on_load_world_requested`、`_load_world_in_place`、`_copy_dir_recursive`、`_on_import_litematic_requested`。
6. `_wire_pause_menu` 对应改为:
   ```gdscript
   pause_menu.save_world_requested.connect(ws.save_backup)
   pause_menu.reload_world_requested.connect(func():
       _close_pause()
       ws.reload()
   )
   pause_menu.load_world_requested.connect(func(p: String):
       _close_pause()
       ws.request_load(p)
   )
   pause_menu.import_litematic_requested.connect(func(p: String):
       _close_pause()
       ws.import_litematic(p)
   )
   ```
7. 其余调用点:toolbar `snapshot_pressed` → `ws.save_snapshot`;`_input` 的 KEY_F5 → `ws.save_snapshot()`;测试钩子 `if cfg.test_snapshot_world: _save_world_snapshot()` → `ws.save_snapshot()`。

- [ ] **Step 3: 启动检查,无 SCRIPT ERROR**

- [ ] **Step 4: Commit**

```bash
git add godot_viewer/world_session.gd godot_viewer/main.gd
git commit -m "refactor: extract world_session.gd from main.gd (load/swap/backup/snapshot)"
```

---

### Task 4: edit_session.gd — 撤销栈 + voxel 持久化出壳

**Files:**
- Create: `godot_viewer/edit_session.gd`
- Modify: `godot_viewer/main.gd`(`_undo_stack`/`_UNDO_CAP`、`_push_undo`、`_refresh_undo_label`、`undo_last_edit`、`_on_voxel_destroyed`、`_on_voxel_placed`、`_patch_voxel_on_disk`、`_on_toggle_edit_mode`、`_on_material_picker_requested`、`_on_material_selected`)

- [ ] **Step 1: 创建 edit_session.gd(完整内容如下)**

```gdscript
# edit_session.gd — voxel edit persistence + the unified undo stack.
# Voxel entries ("destroy"/"place") are handled here; "entity_*" entries
# route to entity_edit_controller (injected via set_entity_edit).
extends Node

const VxwWriter = preload("res://vxw_writer.gd")

# Undo stack: each entry = {op: "destroy"|"place", vi: Vector3i, material_id: int}
# - "destroy" entry: the voxel was destroyed; undo re-places it with material_id
# - "place" entry: a new voxel was placed; undo destroys it
const _UNDO_CAP: int = 50
var _undo_stack: Array = []

var _ws  # world_session
var _renderer: Node3D
var _voxel_editor: Node3D
var _pause_menu: CanvasLayer
var _material_picker: CanvasLayer
var _cam_ctl: Node
var _edit_warning: Label
var _entity_edit = null
var _logger


func init_session(ws, renderer: Node3D, voxel_editor: Node3D,
        pause_menu: CanvasLayer, material_picker: CanvasLayer,
        cam_ctl: Node, edit_warning: Label, logger) -> void:
    _ws = ws
    _renderer = renderer
    _voxel_editor = voxel_editor
    _pause_menu = pause_menu
    _material_picker = material_picker
    _cam_ctl = cam_ctl
    _edit_warning = edit_warning
    _logger = logger
    _voxel_editor.voxel_destroyed.connect(_on_voxel_destroyed)
    _voxel_editor.voxel_placed.connect(_on_voxel_placed)
    _material_picker.material_selected.connect(_on_material_selected)


func set_entity_edit(entity_edit) -> void:
    _entity_edit = entity_edit


func toggle_edit_mode() -> void:
    var new_state: bool = not bool(_voxel_editor.is_edit_enabled())
    _voxel_editor.set_edit_enabled(new_state)
    _cam_ctl.set_orbit_enabled(not new_state)
    _pause_menu.set_edit_mode_label(new_state)
    _edit_warning.visible = new_state
    _logger.info("edit_mode", {"enabled": new_state})


func push_undo(entry: Dictionary) -> void:
    _undo_stack.append(entry)
    if _undo_stack.size() > _UNDO_CAP:
        _undo_stack.pop_front()
    _pause_menu.set_undo_count(_undo_stack.size())


func undo_last_edit() -> bool:
    if _undo_stack.is_empty():
        _logger.info("undo", {"status": "stack empty"})
        return false
    var entry: Dictionary = _undo_stack.pop_back()
    var op: String = entry["op"]
    if op == "destroy":
        var vi: Vector3i = entry["vi"]
        var mid: int = int(entry["material_id"])
        if _renderer.add_voxel(vi, mid):
            _voxel_editor._occupied[vi] = true
            patch_voxel_on_disk(vi, mid, 0)
    elif op == "place":
        var vi2: Vector3i = entry["vi"]
        if _renderer.hide_voxel(vi2):
            _voxel_editor._occupied.erase(vi2)
            patch_voxel_on_disk(vi2, 0, 0)
    elif op.begins_with("entity_"):
        _entity_edit.apply_undo(entry)
    _pause_menu.set_undo_count(_undo_stack.size())
    _logger.info("undo", {"op": op, "stack_left": _undo_stack.size()})
    return true


func open_material_picker() -> void:
    if _ws.world == null:
        return
    _material_picker.set_current(_voxel_editor.get_current_material())
    _material_picker.open(_ws.world)


func _on_material_selected(mid: int) -> void:
    _voxel_editor.set_current_material(mid)
    _logger.info("material_selected", {"material_id": mid})


func _on_voxel_destroyed(world_voxel_index: Vector3i, _world_position_m: Vector3) -> void:
    # We don't know the material_id of the destroyed voxel from the signal,
    # so for undo we restore as material=1 (stone) — pragmatic fallback.
    push_undo({"op": "destroy", "vi": world_voxel_index, "material_id": 1})
    patch_voxel_on_disk(world_voxel_index, 0, 0)
    _logger.info("voxel_destroyed", {
        "world_voxel": [world_voxel_index.x, world_voxel_index.y, world_voxel_index.z],
    })


func _on_voxel_placed(world_voxel_index: Vector3i, _world_position_m: Vector3, material_id: int) -> void:
    push_undo({"op": "place", "vi": world_voxel_index, "material_id": material_id})
    patch_voxel_on_disk(world_voxel_index, material_id, 0)
    _logger.info("voxel_placed", {
        "world_voxel": [world_voxel_index.x, world_voxel_index.y, world_voxel_index.z],
        "material_id": material_id,
    })


# Map a world voxel index to its (chunk_coord, local_voxel) and patch the chunk
# file with a single voxel cell. material_id=0 means clear to air.
func patch_voxel_on_disk(world_voxel_index: Vector3i, material_id: int, semantic_id: int) -> void:
    var ce: int = _ws.world.chunk_extent
    var fdiv := Vector3(world_voxel_index) / float(ce)
    var chunk_coord := Vector3i(int(floor(fdiv.x)), int(floor(fdiv.y)), int(floor(fdiv.z)))
    var local := world_voxel_index - chunk_coord * ce
    var chunk_path: String = _ws.world_path + "/chunks/%d_%d_%d.chunk" % [
        chunk_coord.x, chunk_coord.y, chunk_coord.z,
    ]
    var cell := PackedByteArray([material_id, semantic_id, 0, 0])
    VxwWriter.patch_voxel(
        chunk_path,
        chunk_coord,
        local,
        cell,
        ce,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.GZIP,
    )
```

- [ ] **Step 2: main.gd 接入 es**

1. 顶部:加 `const EditSessionScript = preload("res://edit_session.gd")`、`var es: Node`;删 `const VxwWriter = preload(...)`、`const _UNDO_CAP`、`var _undo_stack`。
2. 删除 main.gd 函数:`_push_undo`、`_refresh_undo_label`(无人调用,直接删)、`undo_last_edit`、`_on_voxel_destroyed`、`_on_voxel_placed`、`_patch_voxel_on_disk`、`_on_toggle_edit_mode`、`_on_material_picker_requested`、`_on_material_selected`。
3. `_ready` 中,`ws.load_initial` 成功之后插入:
   ```gdscript
   es = Node.new()
   es.set_script(EditSessionScript)
   es.name = "EditSession"
   add_child(es)
   es.init_session(ws, renderer, voxel_editor, pause_menu, material_picker, cam_ctl, edit_warning, logger)
   ```
   同时删除 `_ready` 里原来的两行 connect:`voxel_editor.voxel_destroyed.connect(...)`、`voxel_editor.voxel_placed.connect(...)`(已进 es.init_session);`material_picker.material_selected.connect(_on_material_selected)`(在 `_wire_pause_menu` 里)同样删除。
   注意 `@onready var edit_warning: Label = $HUD/EditWarning` 保留(传给 es 用)。
4. 引用替换:`entity_edit.entity_undo_push.connect(_push_undo)` → `entity_edit.entity_undo_push.connect(es.push_undo)`;`_input` 的 Ctrl+Z → `es.undo_last_edit()`(加 `and es != null` 守卫);测试钩子 `undo_last_edit()` → `es.undo_last_edit()`。
5. `_wire_pause_menu`:`toggle_edit_mode_requested` → `es.toggle_edit_mode`;`undo_requested` → `es.undo_last_edit`;`material_picker_requested` → `es.open_material_picker`。

- [ ] **Step 3: 启动检查,无 SCRIPT ERROR**

- [ ] **Step 4: Commit**

```bash
git add godot_viewer/edit_session.gd godot_viewer/main.gd
git commit -m "refactor: extract edit_session.gd from main.gd (undo stack + voxel persistence)"
```

---

### Task 5: entity_subsystem.gd — 实体子系统装配出壳

**Files:**
- Create: `godot_viewer/entity_subsystem.gd`
- Modify: `godot_viewer/main.gd`(实体装配块 152-223 行、`_open_inspector_for_selection`、`_on_entity_inspector_committed`、`_on_context_rotate`、`_on_context_delete`、8 个 preload + 8 个 var)

- [ ] **Step 1: 创建 entity_subsystem.gd(完整内容如下)**

```gdscript
# entity_subsystem.gd — assembles the entity-layer nodes and their wiring:
# renderer, item picker, placer, selector, edit controller, inspector,
# top toolbar and context bar. Rebinds itself on world_session.world_loaded.
extends Node

signal menu_requested

const EntityRendererScript = preload("res://entity_renderer.gd")
const ItemPickerScript = preload("res://item_picker.gd")
const EntityPlacerScript = preload("res://entity_placer.gd")
const EntitySelectorScript = preload("res://entity_selector.gd")
const EntityEditControllerScript = preload("res://entity_edit_controller.gd")
const EntityInspectorScript = preload("res://entity_inspector.gd")
const TopToolbarScript = preload("res://top_toolbar.gd")
const EntityContextBarScript = preload("res://entity_context_bar.gd")

var ent_renderer: Node3D = null
var picker: CanvasLayer = null
var placer: Node3D = null
var selector: Node3D = null
var edit: Node = null
var inspector: CanvasLayer = null
var toolbar: CanvasLayer = null
var context_bar: CanvasLayer = null

var _ws  # world_session
var _logger


func build(main_cam: Camera3D, voxel_editor: Node3D, stereo_rig: Node3D,
        ws, edit_session, logger) -> void:
    _ws = ws
    _logger = logger

    ent_renderer = Node3D.new()
    ent_renderer.set_script(EntityRendererScript)
    ent_renderer.name = "EntityRenderer"
    add_child(ent_renderer)
    ent_renderer.init_renderer(logger)
    ent_renderer.load_entities(ws.world_path, ws.world.palette_rgb)

    picker = CanvasLayer.new()
    picker.set_script(ItemPickerScript)
    picker.name = "ItemPicker"
    add_child(picker)
    picker.init_picker(logger)
    picker.set_presets(ent_renderer.get_item_presets())

    placer = Node3D.new()
    placer.set_script(EntityPlacerScript)
    placer.name = "EntityPlacer"
    add_child(placer)
    placer.init_placer(main_cam, voxel_editor, logger)

    selector = Node3D.new()
    selector.set_script(EntitySelectorScript)
    selector.name = "EntitySelector"
    add_child(selector)
    selector.init_selector(
        main_cam, ws.world_path, ent_renderer,
        placer, voxel_editor, logger
    )

    edit = Node.new()
    edit.set_script(EntityEditControllerScript)
    edit.name = "EntityEditController"
    add_child(edit)
    edit.init_controller(
        ws.world_path, ws.world,
        ent_renderer, picker, placer, selector,
        stereo_rig, logger
    )
    edit.entity_undo_push.connect(edit_session.push_undo)
    edit_session.set_entity_edit(edit)

    inspector = CanvasLayer.new()
    inspector.set_script(EntityInspectorScript)
    inspector.name = "EntityInspector"
    add_child(inspector)
    inspector.init_inspector(ws.world_path, logger)
    inspector.entity_committed.connect(_on_inspector_committed)

    toolbar = CanvasLayer.new()
    toolbar.set_script(TopToolbarScript)
    toolbar.name = "TopToolbar"
    add_child(toolbar)
    toolbar.items_pressed.connect(func(): picker.toggle())
    toolbar.snapshot_pressed.connect(ws.save_snapshot)
    toolbar.menu_pressed.connect(func(): menu_requested.emit())

    context_bar = CanvasLayer.new()
    context_bar.set_script(EntityContextBarScript)
    context_bar.name = "EntityContextBar"
    add_child(context_bar)
    context_bar.inspector_pressed.connect(open_inspector_for_selection)
    context_bar.duplicate_pressed.connect(func(): edit.duplicate_selected())
    context_bar.physics_toggle_pressed.connect(func(): selector.toggle_physics_on_selected())
    context_bar.rotate_pressed.connect(_on_context_rotate)
    context_bar.delete_pressed.connect(_on_context_delete)
    context_bar.use_pressed.connect(func(): edit.use_selected())
    selector.entity_selected.connect(func(id: String):
        context_bar.on_entity_selected(id, selector.get_selected_label_name())
    )
    selector.selection_cleared.connect(func():
        context_bar.on_selection_cleared()
    )

    ws.world_loaded.connect(_on_world_loaded)


func _on_world_loaded(world, path: String) -> void:
    ent_renderer.load_entities(path, world.palette_rgb)
    if selector != null:
        selector.set_world_path(path)
    if edit != null:
        edit.set_world(world, path)
    if inspector != null:
        inspector.set_world_path(path)


func open_inspector_for_selection() -> void:
    if inspector == null or selector == null:
        return
    var sel_id: String = String(selector._selected_id)
    if sel_id == "":
        if _logger != null:
            _logger.info("entity_inspector_open", {"status": "no selection"})
        return
    var path := _ws.world_path + "/entities.json"
    if not FileAccess.file_exists(path):
        return
    var txt := FileAccess.get_file_as_string(path)
    if txt.is_empty():
        return
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return
    for e in d.entities:
        if String(e.get("id", "")) == sel_id:
            inspector.set_world_path(_ws.world_path)
            inspector.open_for(e)
            return


func _on_inspector_committed(_updated: Dictionary) -> void:
    if ent_renderer != null and _ws.world != null:
        ent_renderer.load_entities(_ws.world_path, _ws.world.palette_rgb)


func _on_context_rotate(yaw_delta_rad: float) -> void:
    var sid: String = selector.get_selected_id()
    if sid != "":
        selector.rotate_by_id(sid, yaw_delta_rad)


func _on_context_delete() -> void:
    var sid: String = selector.get_selected_id()
    if sid != "":
        selector.delete_by_id(sid)
```

- [ ] **Step 2: main.gd 接入 ents**

1. 顶部:删 8 个 preload(`EntityRendererScript` … `EntityContextBarScript`)与 8 个 var(`entity_renderer` … `entity_context_bar`);加 `const EntitySubsystemScript = preload("res://entity_subsystem.gd")`、`var ents: Node`。
2. `_ready` 中实体装配块(原 153-223 行,从 `entity_renderer = Node3D.new()` 到 selector 两个 connect 结束)整体替换为:
   ```gdscript
   ents = Node.new()
   ents.set_script(EntitySubsystemScript)
   ents.name = "EntitySubsystem"
   add_child(ents)
   ents.build(main_cam, voxel_editor, stereo_rig, ws, es, logger)
   ents.menu_requested.connect(_toggle_pause)
   ```
   (此块须放在 es 创建之后。)
3. 删除 main.gd 函数:`_open_inspector_for_selection`、`_on_entity_inspector_committed`、`_on_context_rotate`、`_on_context_delete`。
4. 引用替换:
   - `_input`:KEY_I 条件 `item_picker != null` → `ents != null and ents.picker != null`,调用 → `ents.picker.toggle()`;Ctrl+D → `ents.edit.duplicate_selected()`(守卫 `ents != null and ents.edit != null`);F2 → `ents.open_inspector_for_selection()`(守卫 `ents != null`);KEY_U → `ents.edit.use_selected()`(守卫同 Ctrl+D)。
   - CLI overrides:`if cfg.open_item_picker and item_picker != null: item_picker.open()` → `if cfg.open_item_picker and ents.picker != null: ents.picker.open()`;`entity_edit.spawn_in_front_of_rig(...)` → `ents.edit.spawn_in_front_of_rig(...)`。
   - 测试钩子块:`entity_selector` → `ents.selector`、`entity_edit` → `ents.edit`、`entity_renderer.get_item_presets()` → `ents.ent_renderer.get_item_presets()`(Task 6 会把这些块整体搬走,此处先保持可运行)。

- [ ] **Step 3: 启动检查,无 SCRIPT ERROR**

- [ ] **Step 4: Commit**

```bash
git add godot_viewer/entity_subsystem.gd godot_viewer/main.gd
git commit -m "refactor: extract entity_subsystem.gd from main.gd (entity node assembly + wiring)"
```

---

### Task 6: test_hooks_controller.gd + main.gd 最终瘦身

**Files:**
- Create: `godot_viewer/test_hooks_controller.gd`
- Rewrite: `godot_viewer/main.gd`(最终形态,全文如 Step 2)

- [ ] **Step 1: 创建 test_hooks_controller.gd(完整内容如下)**

```gdscript
# test_hooks_controller.gd — executes the --test-* CLI hooks after the world
# and all controllers are ready. Keeps the test-only paths out of main.gd.
# Hook semantics are unchanged from the pre-split main.gd; see each block.
extends Node

var _cfg          # boot_config
var _ws           # world_session
var _es           # edit_session
var _ents         # entity_subsystem
var _renderer: Node3D
var _env          # environment_controller
var _logger


func init_hooks(cfg, ws, es, ents, renderer: Node3D, env_ctl, logger) -> void:
    _cfg = cfg
    _ws = ws
    _es = es
    _ents = ents
    _renderer = renderer
    _env = env_ctl
    _logger = logger


func _read_entity_records() -> Array:
    var ent_path := _ws.world_path + "/entities.json"
    if not FileAccess.file_exists(ent_path):
        return []
    var txt := FileAccess.get_file_as_string(ent_path)
    if txt.is_empty():
        return []
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return []
    return d.entities


func run() -> void:
    if _cfg.test_delete_first or _cfg.test_rotate_first_deg != 0.0 or _cfg.test_grab_first_set:
        var rec_list := _read_entity_records()
        if rec_list.size() > 0:
            var first_id := String(rec_list[0].get("id", ""))
            if _cfg.test_grab_first_set:
                _ents.selector.grab_to(first_id, _cfg.test_grab_first_to)
            if _cfg.test_rotate_first_deg != 0.0:
                _ents.selector.rotate_by_id(first_id, deg_to_rad(_cfg.test_rotate_first_deg))
            if _cfg.test_delete_first:
                _ents.selector.delete_by_id(first_id)

    for _i in _cfg.test_undo_times:
        _es.undo_last_edit()

    if _cfg.test_duplicate_first:
        var recs := _read_entity_records()
        if recs.size() > 0:
            _ents.selector._selected_id = String(recs[0].get("id", ""))
            _ents.edit.duplicate_selected()

    if _cfg.test_snapshot_world:
        _ws.save_snapshot()

    if _cfg.test_toggle_behavior_on_first:
        # Find the first entity whose preset declares "switchable" and toggle
        # it. Logs the resulting state so the caller can assert
        # entities.json[<idx>].custom_meta.state == "on".
        var recs3 := _read_entity_records()
        var presets3: Dictionary = _ents.ent_renderer.get_item_presets()
        var picked_id := ""
        var picked_idx := -1
        for i in recs3.size():
            var rec: Dictionary = recs3[i]
            var item_id: String = String(rec.get("custom_meta", {}).get("mc_item", ""))
            if item_id == "" or not presets3.has(item_id):
                continue
            var behs: Array = presets3[item_id].get("behaviors", [])
            if behs.has("switchable"):
                picked_id = String(rec.get("id", ""))
                picked_idx = i
                break
        if picked_id != "":
            _ents.selector._selected_id = picked_id
            var ok3: bool = bool(_ents.edit.use_selected())
            _logger.info("test_toggle_behavior_on_first",
                        {"id": picked_id, "index": picked_idx, "applied": ok3})
        else:
            _logger.info("test_toggle_behavior_on_first",
                        {"status": "no switchable entity"})

    if _cfg.test_hide_voxel_set:
        # Phase-2 dirty-rebuild verification hook. Hide a single voxel and
        # log whether the renderer accepted the call. The snapshot frame
        # (20 frames later by default) gives _process plenty of time to
        # rebuild the affected chunk, so visual evidence is the snapshot
        # itself; log evidence is the print on the next dirty drain.
        var hv_ok := bool(_renderer.hide_voxel(_cfg.test_hide_voxel_vi))
        _logger.info("test_hide_voxel", {
            "vi": [_cfg.test_hide_voxel_vi.x, _cfg.test_hide_voxel_vi.y, _cfg.test_hide_voxel_vi.z],
            "ok": hv_ok,
            "still_has": _renderer.has_voxel(_cfg.test_hide_voxel_vi),
        })

    if _cfg.test_toggle_day_night:
        # CLI hook: flip to Night so the next snapshot frame captures the
        # darker visuals. Logs the new DirectionalLight.light_energy so
        # callers can grep for the value (0.55 = night, 1.6 = day).
        _env.toggle_day_night()
```

- [ ] **Step 2: main.gd 重写为最终形态(全文如下,直接覆盖)**

```gdscript
# main.gd — Layer 7 orchestrator (thin shell).
# Builds the controllers, runs the boot sequence, routes global hotkeys and
# pause-menu events. Everything else lives in its own controller:
#   boot_config (CLI) / world_session (load/swap/backup) / edit_session
#   (undo + persistence) / entity_subsystem (entity nodes) /
#   environment_controller (day-night) / test_hooks_controller (--test-*).
# See docs/architecture.md §10.7 #8.

extends Node3D

const VxwLogger = preload("res://logger.gd")
const BootConfig = preload("res://boot_config.gd")
const WorldSessionScript = preload("res://world_session.gd")
const EditSessionScript = preload("res://edit_session.gd")
const EntitySubsystemScript = preload("res://entity_subsystem.gd")
const EnvironmentControllerScript = preload("res://environment_controller.gd")
const TestHooksControllerScript = preload("res://test_hooks_controller.gd")

var logger  # VxwLog instance, untyped to avoid class_name registration issues
var cfg     # BootConfig
var ws: Node = null      # world_session
var es: Node = null      # edit_session
var ents: Node = null    # entity_subsystem
var env_ctl: Node = null # environment_controller
var _mouse_mode_before_pause: int = Input.MOUSE_MODE_VISIBLE

@onready var renderer: Node3D = $VoxelRenderer
@onready var directional_light: DirectionalLight3D = $DirectionalLight3D
@onready var world_env: WorldEnvironment = $WorldEnvironment
@onready var stereo_rig: Node3D = $StereoRig  # has stereo_rig_controller.gd
@onready var cam_ctl: Node = $CameraController
@onready var hud_ctl: Node = $HudController
@onready var snap_ctl: Node = $SnapshotController
@onready var pause_menu: CanvasLayer = $PauseMenu
@onready var voxel_editor: Node3D = $VoxelEditor
@onready var material_picker: CanvasLayer = $MaterialPicker
@onready var main_cam: Camera3D = $Camera3D
@onready var status_label: Label = $HUD/StatusLabel
@onready var mode_label: Label = $HUD/ModeLabel
@onready var pose_label: Label = $HUD/PoseLabel
@onready var edit_warning: Label = $HUD/EditWarning
@onready var left_cam: Camera3D = $HUD/LeftStereoContainer/LeftStereoViewport/LeftStereoCamera
@onready var right_cam: Camera3D = $HUD/RightStereoContainer/RightStereoViewport/RightStereoCamera
@onready var left_vp: SubViewport = $HUD/LeftStereoContainer/LeftStereoViewport
@onready var right_vp: SubViewport = $HUD/RightStereoContainer/RightStereoViewport


func _ready() -> void:
    logger = VxwLogger.new()
    cfg = BootConfig.new()
    cfg.parse_args(OS.get_cmdline_user_args())
    logger.info("config", {
        "world_path": cfg.world_path,
        "view_override": cfg.view_override,
        "rig_pose_spec": cfg.rig_pose_spec,
    })

    env_ctl = _make_controller(EnvironmentControllerScript, "EnvironmentController")
    env_ctl.init_controller(directional_light, world_env, pause_menu, logger)

    ws = _make_controller(WorldSessionScript, "WorldSession")
    ws.init_session(renderer, logger)
    if not ws.load_initial(cfg.world_path):
        status_label.text = "No voxels loaded. Path tried: " + cfg.world_path
        return

    es = _make_controller(EditSessionScript, "EditSession")
    es.init_session(ws, renderer, voxel_editor, pause_menu, material_picker,
        cam_ctl, edit_warning, logger)

    ents = _make_controller(EntitySubsystemScript, "EntitySubsystem")
    ents.build(main_cam, voxel_editor, stereo_rig, ws, es, logger)
    ents.menu_requested.connect(_toggle_pause)

    left_vp.world_3d = get_viewport().world_3d
    right_vp.world_3d = get_viewport().world_3d
    stereo_rig.init_controller(left_cam, right_cam)
    stereo_rig.reset_pose()
    cam_ctl.init_controller(main_cam, stereo_rig, logger)
    hud_ctl.init_controller(status_label, mode_label, pose_label, ws.world, stereo_rig, cam_ctl, logger)
    snap_ctl.init_controller(stereo_rig, cam_ctl, logger)
    snap_ctl.configure_from_cli(cfg.raw_args)
    voxel_editor.init_editor(renderer, main_cam)
    voxel_editor.set_edit_enabled(false)  # default OFF — opt-in via pause menu
    ws.world_loaded.connect(_on_world_loaded)
    _wire_pause_menu()
    pause_menu.set_persistence_available(true)
    pause_menu.set_edit_mode_label(false)

    _apply_cli_overrides()

    var hooks: Node = _make_controller(TestHooksControllerScript, "TestHooksController")
    hooks.init_hooks(cfg, ws, es, ents, renderer, env_ctl, logger)
    hooks.run()


func _make_controller(controller_script: Script, node_name: String) -> Node:
    var n := Node.new()
    n.set_script(controller_script)
    n.name = node_name
    add_child(n)
    return n


func _apply_cli_overrides() -> void:
    # Apply manifest spawn_hint if present and no explicit --rig-pose was
    # given. The hint is [x_m, y_m, z_m, yaw_deg]: an adapter-picked open
    # floor cell so the user lands in the middle of a room facing inward.
    if cfg.rig_pose_spec == "" and ws.world.spawn_hint.size() == 4:
        var sh: Array = ws.world.spawn_hint
        var basis_h := Basis().rotated(Vector3.UP, deg_to_rad(float(sh[3])))
        var xf_h := Transform3D(basis_h, Vector3(float(sh[0]), float(sh[1]), float(sh[2])))
        call_deferred("_set_rig_xform_deferred", xf_h)
        logger.info("spawn_hint_applied", {"pos": [sh[0], sh[1], sh[2]], "yaw_deg": sh[3]})
    # CLI-driven overrides (must happen after init_controller)
    if cfg.view_override != "":
        cam_ctl.set_view_mode_str(cfg.view_override)
    if cfg.rig_pose_spec != "":
        call_deferred("_set_rig_xform_deferred", BootConfig.parse_rig_pose(cfg.rig_pose_spec))
    if cfg.open_item_picker and ents.picker != null:
        ents.picker.open()
    for sid in cfg.spawn_items:
        if sid != "":
            ents.edit.spawn_in_front_of_rig(String(sid))


func _set_rig_xform_deferred(xf: Transform3D) -> void:
    stereo_rig.set_pose(xf)


func _on_world_loaded(world, _path: String) -> void:
    # In-place swap rebind: re-init editor with the new world, reset rig,
    # refresh HUD voxel count / size. Entity rebind lives in entity_subsystem.
    voxel_editor.init_editor(renderer, main_cam)
    stereo_rig.reset_pose()
    hud_ctl.init_controller(status_label, mode_label, pose_label, world, stereo_rig, cam_ctl, logger)


func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed:
        if event.keycode == KEY_ESCAPE:
            _toggle_pause()
        elif event.keycode == KEY_Z and event.ctrl_pressed and es != null:
            es.undo_last_edit()
        elif event.keycode == KEY_I and ents != null and ents.picker != null:
            ents.picker.toggle()
        elif event.keycode == KEY_D and event.ctrl_pressed and ents != null and ents.edit != null:
            ents.edit.duplicate_selected()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_F2 and ents != null:
            ents.open_inspector_for_selection()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_U and ents != null and ents.edit != null:
            ents.edit.use_selected()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_F5 and ws != null:
            ws.save_snapshot()
            get_viewport().set_input_as_handled()


func _wire_pause_menu() -> void:
    pause_menu.resume_requested.connect(_close_pause)
    pause_menu.toggle_view_requested.connect(func():
        cam_ctl.toggle_view_mode()
        _close_pause()
    )
    pause_menu.reset_rig_requested.connect(func():
        stereo_rig.reset_pose()
        _close_pause()
    )
    pause_menu.save_world_requested.connect(ws.save_backup)
    pause_menu.reload_world_requested.connect(func():
        _close_pause()
        ws.reload()
    )
    pause_menu.load_world_requested.connect(func(p: String):
        _close_pause()
        ws.request_load(p)
    )
    pause_menu.import_litematic_requested.connect(func(p: String):
        _close_pause()
        ws.import_litematic(p)
    )
    pause_menu.toggle_edit_mode_requested.connect(es.toggle_edit_mode)
    pause_menu.undo_requested.connect(es.undo_last_edit)
    pause_menu.material_picker_requested.connect(es.open_material_picker)
    pause_menu.toggle_day_night_requested.connect(func():
        env_ctl.toggle_day_night()
        _close_pause()
    )
    pause_menu.quit_requested.connect(func():
        logger.info("session_end", {"reason": "menu_quit"})
        get_tree().quit()
    )


func _toggle_pause() -> void:
    if pause_menu.is_open():
        _close_pause()
    else:
        _open_pause()


func _open_pause() -> void:
    _mouse_mode_before_pause = Input.mouse_mode
    Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
    get_tree().paused = true
    pause_menu.open()
    logger.info("pause", {"opened": true})


func _close_pause() -> void:
    pause_menu.close()
    get_tree().paused = false
    Input.mouse_mode = _mouse_mode_before_pause
    logger.info("pause", {"opened": false})
```

- [ ] **Step 3: 启动检查,无 SCRIPT ERROR**

- [ ] **Step 4: Commit**

```bash
git add godot_viewer/test_hooks_controller.gd godot_viewer/main.gd
git commit -m "refactor: extract test_hooks_controller.gd; main.gd is a thin shell (violation #8 paid)"
```

---

### Task 7: 总体冒烟 + 文档回填

**Files:**
- Modify: `docs/architecture.md`(§10.7 表)

- [ ] **Step 1: 总体冒烟(4 条命令,依次跑)**

```powershell
# 1) baseline 世界 + 截图
F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer --position -10000,-10000 --resolution 1280x720 --quit-after 60 -- --world=F:/slam-voxel-world/out/baseline_20cm.vxw --snapshot=F:/slam-voxel-world/out/refactor_smoke_baseline.png

# 2) 两层公寓(带 entity)+ 夜间钩子 + 截图
F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer --position -10000,-10000 --resolution 1280x720 --quit-after 60 -- --world=F:/slam-voxel-world/out/uhumans2_apt_ent.vxw --test-toggle-day-night --snapshot=F:/slam-voxel-world/out/refactor_smoke_apt_night.png

# 3) writer selftest
F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer --headless --script res://_writer_selftest.gd

# 4) editor selftest(注意:不带 --headless)
F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer --position -10000,-10000 --resolution 800x600 --quit-after 30 --script res://_editor_selftest.gd
```

通过标准:
- 命令 1/2 各自生成 PNG,且控制台无 `SCRIPT ERROR` / `push_error`;命令 2 的日志含 `day_night_toggle` 且 `directional_light_energy` 为 `0.55`。
- 命令 3 输出 `SELFTEST OK`;命令 4 无 `[EDITOR SELFTEST] FAIL`。
- 任一不过:先修再继续,不许带病提交。

- [ ] **Step 2: 回填 architecture.md §10.7**

把违规表 #8 行改为:

```markdown
| 8 | ~~`main.gd` 又涨到 450+ 行(M2+wiring 全加进去)~~ **已修(2026-06)**:拆为 boot_config / world_session / edit_session / entity_subsystem / environment_controller / test_hooks_controller,main.gd 收缩为瘦壳 | main.gd | — |
```

- [ ] **Step 3: 删除冒烟产物 PNG(out/refactor_smoke_*.png 不留 repo,本就未跟踪,确认 git status 干净即可)**

- [ ] **Step 4: Commit**

```bash
git add docs/architecture.md
git commit -m "docs: mark architecture.md violation #8 as paid (main.gd split)"
```

---

## Self-Review 结论(已自查)

- **Spec 覆盖**:6 个新文件 ✓、main.gd 瘦壳 ✓、world_loaded 信号预留 ✓、冒烟从简 ✓、§10.7 回填 ✓、#10/#11 留范围外 ✓。
- **类型一致性**:`init_session(ws, ...)`/`build(...)`/`init_hooks(...)` 签名与 main.gd 最终形态调用一一核对过;`ents.ent_renderer`/`ents.picker`/`ents.edit`/`ents.selector` 字段名在 Task 5/6 一致。
- **已知行为微调(可接受,记录在案)**:① `_refresh_undo_label` 是死代码,直接删除;② 测试钩子里 3 处重复的 entities.json 读取合并为 `_read_entity_records()`(纯 DRY,语义不变);③ `reload()` 的日志先于 `_close_pause`(原版相反),日志顺序差异无行为影响;④ 夜间能量日志注释由 0.12 修正为 0.55(与代码一致,原注释过期)。
