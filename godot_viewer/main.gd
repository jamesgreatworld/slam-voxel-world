# main.gd — Layer 7 orchestrator.
# Composes the controllers, loads the world, routes top-level events.
# All rendering / camera / rig / UI / snapshot logic lives in its own controller.
# See docs/architecture.md.

extends Node3D

const VxwLogger = preload("res://logger.gd")
const BootConfig = preload("res://boot_config.gd")
const EnvironmentControllerScript = preload("res://environment_controller.gd")
const WorldSessionScript = preload("res://world_session.gd")
const EditSessionScript = preload("res://edit_session.gd")

var logger  # VxwLog instance, untyped to avoid class_name registration issues
var ws: Node
var es: Node

@onready var renderer: Node3D = $VoxelRenderer
@onready var directional_light: DirectionalLight3D = $DirectionalLight3D
@onready var world_env: WorldEnvironment = $WorldEnvironment
const EntitySubsystemScript = preload("res://entity_subsystem.gd")
var ents: Node
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

var _mouse_mode_before_pause: int = Input.MOUSE_MODE_VISIBLE
var cfg  # BootConfig
var env_ctl: Node


func _ready() -> void:
    logger = VxwLogger.new()

    env_ctl = Node.new()
    env_ctl.set_script(EnvironmentControllerScript)
    env_ctl.name = "EnvironmentController"
    add_child(env_ctl)

    cfg = BootConfig.new()
    cfg.parse_args(OS.get_cmdline_user_args())

    logger.info("config", {
        "world_path": cfg.world_path,
        "view_override": cfg.view_override,
        "rig_pose_spec": cfg.rig_pose_spec,
    })

    ws = Node.new()
    ws.set_script(WorldSessionScript)
    ws.name = "WorldSession"
    add_child(ws)
    ws.init_session(renderer, logger)
    if not ws.load_initial(cfg.world_path):
        status_label.text = "No voxels loaded. Path tried: " + cfg.world_path
        return

    es = Node.new()
    es.set_script(EditSessionScript)
    es.name = "EditSession"
    add_child(es)
    es.init_session(ws, renderer, voxel_editor, pause_menu, material_picker,
        cam_ctl, edit_warning, logger)

    ents = Node.new()
    ents.set_script(EntitySubsystemScript)
    ents.name = "EntitySubsystem"
    add_child(ents)
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
    env_ctl.init_controller(directional_light, world_env, pause_menu, logger)
    ws.world_loaded.connect(_on_world_loaded)
    _wire_pause_menu()
    pause_menu.set_persistence_available(true)
    pause_menu.set_edit_mode_label(false)

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
        var xf := BootConfig.parse_rig_pose(cfg.rig_pose_spec)
        call_deferred("_set_rig_xform_deferred", xf)
    if cfg.open_item_picker and ents.picker != null:
        ents.picker.open()
    for sid in cfg.spawn_items:
        if sid != "":
            ents.edit.spawn_in_front_of_rig(String(sid))

    if cfg.test_delete_first or cfg.test_rotate_first_deg != 0.0 or cfg.test_grab_first_set:
        var ent_path: String = ws.world_path + "/entities.json"
        var rec_list: Array = []
        if FileAccess.file_exists(ent_path):
            var txt := FileAccess.get_file_as_string(ent_path)
            if not txt.is_empty():
                var d = JSON.parse_string(txt)
                if d != null and d.has("entities"):
                    rec_list = d.entities
        if rec_list.size() > 0:
            var first_id := String(rec_list[0].get("id", ""))
            if cfg.test_grab_first_set:
                ents.selector.grab_to(first_id, cfg.test_grab_first_to)
            if cfg.test_rotate_first_deg != 0.0:
                ents.selector.rotate_by_id(first_id, deg_to_rad(cfg.test_rotate_first_deg))
            if cfg.test_delete_first:
                ents.selector.delete_by_id(first_id)

    for _i in cfg.test_undo_times:
        es.undo_last_edit()

    if cfg.test_duplicate_first:
        var ent_path2: String = ws.world_path + "/entities.json"
        var recs: Array = []
        if FileAccess.file_exists(ent_path2):
            var t := FileAccess.get_file_as_string(ent_path2)
            if not t.is_empty():
                var dd = JSON.parse_string(t)
                if dd != null and dd.has("entities"):
                    recs = dd.entities
        if recs.size() > 0:
            var fid := String(recs[0].get("id", ""))
            ents.selector._selected_id = fid
            ents.edit.duplicate_selected()

    if cfg.test_snapshot_world:
        ws.save_snapshot()

    if cfg.test_toggle_behavior_on_first:
        # Find the first entity whose preset declares "switchable" and toggle
        # it. Logs the resulting state so the caller can assert
        # entities.json[<idx>].custom_meta.state == "on".
        var ent_path3: String = ws.world_path + "/entities.json"
        var recs3: Array = []
        if FileAccess.file_exists(ent_path3):
            var t3 := FileAccess.get_file_as_string(ent_path3)
            if not t3.is_empty():
                var dd3 = JSON.parse_string(t3)
                if dd3 != null and dd3.has("entities"):
                    recs3 = dd3.entities
        var presets3: Dictionary = ents.ent_renderer.get_item_presets()
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
            ents.selector._selected_id = picked_id
            var ok3: bool = bool(ents.edit.use_selected())
            logger.info("test_toggle_behavior_on_first",
                        {"id": picked_id, "index": picked_idx, "applied": ok3})
        else:
            logger.info("test_toggle_behavior_on_first",
                        {"status": "no switchable entity"})

    if cfg.test_hide_voxel_set:
        # Phase-2 dirty-rebuild verification hook. Hide a single voxel and
        # log whether the renderer accepted the call. The snapshot frame
        # (20 frames later by default) gives _process plenty of time to
        # rebuild the affected chunk, so visual evidence is the snapshot
        # itself; log evidence is the print on the next dirty drain.
        var hv_ok := bool(renderer.hide_voxel(cfg.test_hide_voxel_vi))
        logger.info("test_hide_voxel", {
            "vi": [cfg.test_hide_voxel_vi.x, cfg.test_hide_voxel_vi.y, cfg.test_hide_voxel_vi.z],
            "ok": hv_ok,
            "still_has": renderer.has_voxel(cfg.test_hide_voxel_vi),
        })

    if cfg.test_toggle_day_night:
        # CLI hook: flip to Night so the next snapshot frame captures the
        # darker visuals. Logs the new DirectionalLight.light_energy so
        # callers can grep for the value (0.55 = night, 1.6 = day).
        env_ctl.toggle_day_night()


func _set_rig_xform_deferred(xf: Transform3D) -> void:
    stereo_rig.set_pose(xf)


func _on_world_loaded(world, path: String) -> void:
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
        elif event.keycode == KEY_F5:
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
    pause_menu.toggle_edit_mode_requested.connect(es.toggle_edit_mode)
    pause_menu.undo_requested.connect(es.undo_last_edit)
    pause_menu.material_picker_requested.connect(es.open_material_picker)
    pause_menu.import_litematic_requested.connect(func(p: String):
        _close_pause()
        ws.import_litematic(p)
    )
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

