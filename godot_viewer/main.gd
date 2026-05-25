# main.gd — Layer 7 orchestrator.
# Composes the controllers, loads the world, routes top-level events.
# All rendering / camera / rig / UI / snapshot logic lives in its own controller.
# See docs/architecture.md.

extends Node3D

const VxwLoader = preload("res://vxw_loader.gd")
const VxwWriter = preload("res://vxw_writer.gd")
const VxwLogger = preload("res://logger.gd")
const DEFAULT_WORLD_PATH := "../out/baseline.vxw"

var logger  # VxwLog instance, untyped to avoid class_name registration issues
var _world  # loaded VxwLoader.VxwWorld
var _world_path_absolute: String = ""

@onready var renderer: Node3D = $VoxelRenderer
@onready var stereo_rig: Node3D = $StereoRig  # has stereo_rig_controller.gd
@onready var cam_ctl: Node = $CameraController
@onready var hud_ctl: Node = $HudController
@onready var snap_ctl: Node = $SnapshotController
@onready var pause_menu: CanvasLayer = $PauseMenu
@onready var voxel_editor: Node3D = $VoxelEditor
@onready var main_cam: Camera3D = $Camera3D
@onready var status_label: Label = $HUD/StatusLabel
@onready var mode_label: Label = $HUD/ModeLabel
@onready var pose_label: Label = $HUD/PoseLabel
@onready var left_cam: Camera3D = $HUD/LeftStereoContainer/LeftStereoViewport/LeftStereoCamera
@onready var right_cam: Camera3D = $HUD/RightStereoContainer/RightStereoViewport/RightStereoCamera
@onready var left_vp: SubViewport = $HUD/LeftStereoContainer/LeftStereoViewport
@onready var right_vp: SubViewport = $HUD/RightStereoContainer/RightStereoViewport

var _mouse_mode_before_pause: int = Input.MOUSE_MODE_VISIBLE


func _ready() -> void:
    logger = VxwLogger.new()

    var args: PackedStringArray = OS.get_cmdline_user_args()
    var world_path := DEFAULT_WORLD_PATH
    var view_override := ""
    var rig_pose_spec := ""
    for arg in args:
        if arg.begins_with("--world="):
            world_path = arg.substr("--world=".length())
        elif arg == "--view=1p":
            view_override = "1p"
        elif arg == "--view=3p":
            view_override = "3p"
        elif arg.begins_with("--rig-pose="):
            rig_pose_spec = arg.substr("--rig-pose=".length())

    logger.info("config", {
        "world_path": world_path,
        "view_override": view_override,
        "rig_pose_spec": rig_pose_spec,
    })

    _world_path_absolute = _resolve(world_path)
    _world = VxwLoader.load_world(_world_path_absolute)
    if _world.voxel_count() == 0:
        status_label.text = "No voxels loaded. Path tried: " + world_path
        logger.error("world", {"reason": "empty load", "path": world_path})
        return

    logger.info("world_loaded", {
        "voxels": _world.voxel_count(),
        "voxel_size_m": _world.voxel_size_meters,
        "chunk_extent": _world.chunk_extent,
        "path": _world_path_absolute,
    })

    # Wire everything up
    renderer.build(_world)
    left_vp.world_3d = get_viewport().world_3d
    right_vp.world_3d = get_viewport().world_3d
    stereo_rig.init_controller(left_cam, right_cam)
    stereo_rig.reset_pose()
    cam_ctl.init_controller(main_cam, stereo_rig, logger)
    hud_ctl.init_controller(status_label, mode_label, pose_label, _world, stereo_rig, cam_ctl, logger)
    snap_ctl.init_controller(stereo_rig, cam_ctl, logger)
    snap_ctl.configure_from_cli(args)
    voxel_editor.init_editor(_world, renderer.get_mmi(), main_cam)
    voxel_editor.voxel_destroyed.connect(_on_voxel_destroyed)
    _wire_pause_menu()
    pause_menu.set_persistence_available(true)

    # CLI-driven overrides (must happen after init_controller)
    if view_override != "":
        cam_ctl.set_view_mode_str(view_override)
    if rig_pose_spec != "":
        var xf := _parse_rig_pose(rig_pose_spec)
        call_deferred("_set_rig_xform_deferred", xf)


func _set_rig_xform_deferred(xf: Transform3D) -> void:
    stereo_rig.set_pose(xf)


func _resolve(p: String) -> String:
    if p.is_absolute_path():
        return p
    var base := ProjectSettings.globalize_path("res://")
    return base.path_join(p)


func _parse_rig_pose(spec: String) -> Transform3D:
    var parts := spec.split(",")
    if parts.size() != 6:
        push_error("[main] --rig-pose needs 6 comma-separated numbers, got %d" % parts.size())
        return Transform3D.IDENTITY
    var p := Vector3(float(parts[0]), float(parts[1]), float(parts[2]))
    var basis := Basis()
    basis = basis.rotated(Vector3.UP, deg_to_rad(float(parts[3])))     # yaw
    basis = basis.rotated(basis.x, deg_to_rad(float(parts[4])))        # pitch (local X)
    basis = basis.rotated(basis.z, deg_to_rad(float(parts[5])))        # roll (local Z)
    return Transform3D(basis, p)


func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
        _toggle_pause()


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
    pause_menu.save_world_requested.connect(_on_save_world_backup)
    pause_menu.reload_world_requested.connect(_on_reload_world)
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


# ---- R3: voxel editing + persistence ----

func _on_voxel_destroyed(world_voxel_index: Vector3i, _world_position_m: Vector3) -> void:
    # Map world voxel index → (chunk_coord, local_voxel) using FLOOR (works for negatives).
    var ce: int = _world.chunk_extent
    var fdiv := Vector3(world_voxel_index) / float(ce)
    var chunk_coord := Vector3i(int(floor(fdiv.x)), int(floor(fdiv.y)), int(floor(fdiv.z)))
    var local := world_voxel_index - chunk_coord * ce

    var chunk_path: String = _world_path_absolute + "/chunks/%d_%d_%d.chunk" % [
        chunk_coord.x, chunk_coord.y, chunk_coord.z,
    ]
    # Air cell = (material=0, semantic=0, state=0, color_idx=0)
    var air_cell := PackedByteArray([0, 0, 0, 0])
    var result := VxwWriter.patch_voxel(
        chunk_path,
        chunk_coord,
        local,
        air_cell,
        ce,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.GZIP,
    )
    logger.info("voxel_destroyed", {
        "world_voxel": [world_voxel_index.x, world_voxel_index.y, world_voxel_index.z],
        "chunk_coord": [chunk_coord.x, chunk_coord.y, chunk_coord.z],
        "local": [local.x, local.y, local.z],
        "chunk_path": chunk_path,
        "patch_result": result,  # OK=0 on success
    })


func _on_save_world_backup() -> void:
    var ts := Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
    var src := _world_path_absolute
    var dir := src.get_base_dir()
    var base := src.get_file()
    if base.ends_with(".vxw"):
        base = base.substr(0, base.length() - 4)
    var dst := "%s/%s_backup_%s.vxw" % [dir, base, ts]
    var ok := _copy_dir_recursive(src, dst)
    logger.info("world_backup", {"src": src, "dst": dst, "ok": ok})


func _on_reload_world() -> void:
    logger.info("world_reload", {"path": _world_path_absolute})
    _close_pause()
    get_tree().reload_current_scene()


func _copy_dir_recursive(src: String, dst: String) -> bool:
    if not DirAccess.dir_exists_absolute(src):
        push_error("[main] copy_dir source missing: " + src)
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
