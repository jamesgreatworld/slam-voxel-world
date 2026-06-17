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
    if cfg.watch:
        ws.enable_watch()
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
    # refresh HUD. Entity rebind lives in entity_subsystem.
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
