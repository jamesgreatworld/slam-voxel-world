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

# Undo stack: each entry = {op: "destroy"|"place", vi: Vector3i, material_id: int}
# - "destroy" entry: the voxel was destroyed; undo re-places it with material_id
# - "place" entry: a new voxel was placed; undo destroys it
const _UNDO_CAP: int = 50
var _undo_stack: Array = []

@onready var renderer: Node3D = $VoxelRenderer
const EntityRendererScript = preload("res://entity_renderer.gd")
const ItemPickerScript = preload("res://item_picker.gd")
const EntityPlacerScript = preload("res://entity_placer.gd")
const EntitySelectorScript = preload("res://entity_selector.gd")
const EntityEditControllerScript = preload("res://entity_edit_controller.gd")
const EntityInspectorScript = preload("res://entity_inspector.gd")
var entity_renderer: Node3D = null
var item_picker: CanvasLayer = null
var entity_placer: Node3D = null
var entity_selector: Node3D = null
var entity_edit: Node = null
var entity_inspector: CanvasLayer = null
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
var _test_delete_first: bool = false
var _test_rotate_first_deg: float = 0.0
var _test_grab_first_to: Vector3 = Vector3.ZERO
var _test_grab_first_set: bool = false
var _test_undo_times: int = 0
var _test_duplicate_first: bool = false
var _test_snapshot_world: bool = false


func _ready() -> void:
    logger = VxwLogger.new()

    var args: PackedStringArray = OS.get_cmdline_user_args()
    var world_path := DEFAULT_WORLD_PATH
    var view_override := ""
    var rig_pose_spec := ""
    var open_item_picker := false
    var spawn_items: Array = []
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
            _test_delete_first = true
        elif arg.begins_with("--test-rotate-first="):
            _test_rotate_first_deg = float(arg.substr("--test-rotate-first=".length()))
        elif arg.begins_with("--test-grab-first-to="):
            var parts := arg.substr("--test-grab-first-to=".length()).split(",")
            if parts.size() == 3:
                _test_grab_first_to = Vector3(
                    float(parts[0]), float(parts[1]), float(parts[2])
                )
                _test_grab_first_set = true
        elif arg.begins_with("--test-undo-times="):
            _test_undo_times = int(arg.substr("--test-undo-times=".length()))
        elif arg == "--test-duplicate-first":
            _test_duplicate_first = true
        elif arg == "--test-snapshot-world":
            _test_snapshot_world = true

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
    entity_renderer = Node3D.new()
    entity_renderer.set_script(EntityRendererScript)
    entity_renderer.name = "EntityRenderer"
    add_child(entity_renderer)
    entity_renderer.init_renderer(logger)
    entity_renderer.load_entities(_world_path_absolute, _world.palette_rgb)

    item_picker = CanvasLayer.new()
    item_picker.set_script(ItemPickerScript)
    item_picker.name = "ItemPicker"
    add_child(item_picker)
    item_picker.init_picker(logger)
    item_picker.set_presets(entity_renderer.get_item_presets())

    entity_placer = Node3D.new()
    entity_placer.set_script(EntityPlacerScript)
    entity_placer.name = "EntityPlacer"
    add_child(entity_placer)
    entity_placer.init_placer(main_cam, voxel_editor, logger)

    entity_selector = Node3D.new()
    entity_selector.set_script(EntitySelectorScript)
    entity_selector.name = "EntitySelector"
    add_child(entity_selector)
    entity_selector.init_selector(
        main_cam, _world_path_absolute, entity_renderer,
        entity_placer, voxel_editor, logger
    )

    entity_edit = Node.new()
    entity_edit.set_script(EntityEditControllerScript)
    entity_edit.name = "EntityEditController"
    add_child(entity_edit)
    entity_edit.init_controller(
        _world_path_absolute, _world,
        entity_renderer, item_picker, entity_placer, entity_selector,
        stereo_rig, logger
    )
    entity_edit.entity_undo_push.connect(_push_undo)

    entity_inspector = CanvasLayer.new()
    entity_inspector.set_script(EntityInspectorScript)
    entity_inspector.name = "EntityInspector"
    add_child(entity_inspector)
    entity_inspector.init_inspector(_world_path_absolute, logger)
    entity_inspector.entity_committed.connect(_on_entity_inspector_committed)

    left_vp.world_3d = get_viewport().world_3d
    right_vp.world_3d = get_viewport().world_3d
    stereo_rig.init_controller(left_cam, right_cam)
    stereo_rig.reset_pose()
    cam_ctl.init_controller(main_cam, stereo_rig, logger)
    hud_ctl.init_controller(status_label, mode_label, pose_label, _world, stereo_rig, cam_ctl, logger)
    snap_ctl.init_controller(stereo_rig, cam_ctl, logger)
    snap_ctl.configure_from_cli(args)
    voxel_editor.init_editor(renderer, main_cam)
    voxel_editor.voxel_destroyed.connect(_on_voxel_destroyed)
    voxel_editor.voxel_placed.connect(_on_voxel_placed)
    voxel_editor.set_edit_enabled(false)  # default OFF — opt-in via pause menu
    _wire_pause_menu()
    pause_menu.set_persistence_available(true)
    pause_menu.set_edit_mode_label(false)

    # CLI-driven overrides (must happen after init_controller)
    if view_override != "":
        cam_ctl.set_view_mode_str(view_override)
    if rig_pose_spec != "":
        var xf := _parse_rig_pose(rig_pose_spec)
        call_deferred("_set_rig_xform_deferred", xf)
    if open_item_picker and item_picker != null:
        item_picker.open()
    for sid in spawn_items:
        if sid != "":
            entity_edit.spawn_in_front_of_rig(String(sid))

    if _test_delete_first or _test_rotate_first_deg != 0.0 or _test_grab_first_set:
        var ent_path := _world_path_absolute + "/entities.json"
        var rec_list: Array = []
        if FileAccess.file_exists(ent_path):
            var txt := FileAccess.get_file_as_string(ent_path)
            if not txt.is_empty():
                var d = JSON.parse_string(txt)
                if d != null and d.has("entities"):
                    rec_list = d.entities
        if rec_list.size() > 0:
            var first_id := String(rec_list[0].get("id", ""))
            if _test_grab_first_set:
                entity_selector.grab_to(first_id, _test_grab_first_to)
            if _test_rotate_first_deg != 0.0:
                entity_selector.rotate_by_id(first_id, deg_to_rad(_test_rotate_first_deg))
            if _test_delete_first:
                entity_selector.delete_by_id(first_id)

    for _i in _test_undo_times:
        undo_last_edit()

    if _test_duplicate_first:
        var ent_path2 := _world_path_absolute + "/entities.json"
        var recs: Array = []
        if FileAccess.file_exists(ent_path2):
            var t := FileAccess.get_file_as_string(ent_path2)
            if not t.is_empty():
                var dd = JSON.parse_string(t)
                if dd != null and dd.has("entities"):
                    recs = dd.entities
        if recs.size() > 0:
            var fid := String(recs[0].get("id", ""))
            entity_selector._selected_id = fid
            entity_edit.duplicate_selected()

    if _test_snapshot_world:
        _save_world_snapshot()


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
    if event is InputEventKey and event.pressed:
        if event.keycode == KEY_ESCAPE:
            _toggle_pause()
        elif event.keycode == KEY_Z and event.ctrl_pressed:
            undo_last_edit()
        elif event.keycode == KEY_I and item_picker != null:
            item_picker.toggle()
        elif event.keycode == KEY_D and event.ctrl_pressed and entity_edit != null:
            entity_edit.duplicate_selected()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_F2 and entity_inspector != null and entity_selector != null:
            _open_inspector_for_selection()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_F5:
            _save_world_snapshot()
            get_viewport().set_input_as_handled()


func _open_inspector_for_selection() -> void:
    if entity_inspector == null or entity_selector == null:
        return
    var sel_id: String = String(entity_selector._selected_id)
    if sel_id == "":
        if logger != null:
            logger.info("entity_inspector_open", {"status": "no selection"})
        return
    var path := _world_path_absolute + "/entities.json"
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
            entity_inspector.set_world_path(_world_path_absolute)
            entity_inspector.open_for(e)
            return


func _on_entity_inspector_committed(_updated: Dictionary) -> void:
    if entity_renderer != null and _world != null:
        entity_renderer.load_entities(_world_path_absolute, _world.palette_rgb)


func _save_world_snapshot() -> void:
    # Snapshot the current world dir into out/snapshots/<base>_<ts>/
    # Uses _copy_dir_recursive to grab manifest/palette/entities.json/chunks.
    var src := _world_path_absolute
    if src == "" or not DirAccess.dir_exists_absolute(src):
        if logger != null:
            logger.error("world_snapshot", {"reason": "src missing", "src": src})
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
    if logger != null:
        logger.info("world_snapshot", {"src": src, "dst": dst, "ok": ok})


# Entity-layer mutations live on entity_edit (see entity_edit_controller.gd).
# main.gd only routes undo entries through it.


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
    pause_menu.load_world_requested.connect(_on_load_world_requested)
    pause_menu.toggle_edit_mode_requested.connect(_on_toggle_edit_mode)
    pause_menu.undo_requested.connect(undo_last_edit)
    pause_menu.material_picker_requested.connect(_on_material_picker_requested)
    pause_menu.import_litematic_requested.connect(_on_import_litematic_requested)
    material_picker.material_selected.connect(_on_material_selected)
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

func _on_toggle_edit_mode() -> void:
    var new_state: bool = not bool(voxel_editor.is_edit_enabled())
    voxel_editor.set_edit_enabled(new_state)
    cam_ctl.set_orbit_enabled(not new_state)
    pause_menu.set_edit_mode_label(new_state)
    edit_warning.visible = new_state
    logger.info("edit_mode", {"enabled": new_state})


func _on_voxel_destroyed(world_voxel_index: Vector3i, _world_position_m: Vector3) -> void:
    # We don't know the material_id of the destroyed voxel from the signal,
    # so for undo we restore as material=1 (stone) — pragmatic fallback.
    # When voxel_editor tracks original material on hover we can pass it through.
    _push_undo({"op": "destroy", "vi": world_voxel_index, "material_id": 1})
    _patch_voxel_on_disk(world_voxel_index, 0, 0)
    logger.info("voxel_destroyed", {
        "world_voxel": [world_voxel_index.x, world_voxel_index.y, world_voxel_index.z],
    })


func _on_voxel_placed(world_voxel_index: Vector3i, _world_position_m: Vector3, material_id: int) -> void:
    _push_undo({"op": "place", "vi": world_voxel_index, "material_id": material_id})
    _patch_voxel_on_disk(world_voxel_index, material_id, 0)
    logger.info("voxel_placed", {
        "world_voxel": [world_voxel_index.x, world_voxel_index.y, world_voxel_index.z],
        "material_id": material_id,
    })


func _push_undo(entry: Dictionary) -> void:
    _undo_stack.append(entry)
    if _undo_stack.size() > _UNDO_CAP:
        _undo_stack.pop_front()
    pause_menu.set_undo_count(_undo_stack.size())


func _refresh_undo_label() -> void:
    pause_menu.set_undo_count(_undo_stack.size())


func _on_material_picker_requested() -> void:
    if _world == null:
        return
    material_picker.set_current(voxel_editor.get_current_material())
    material_picker.open(_world)


func _on_material_selected(mid: int) -> void:
    voxel_editor.set_current_material(mid)
    logger.info("material_selected", {"material_id": mid})


func _on_import_litematic_requested(path: String) -> void:
    logger.info("import_litematic", {"src": path})
    _close_pause()
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
        logger.error("import_litematic", {"exit_code": exit_code, "output": output})
        return
    logger.info("import_litematic_ok", {"out": out_vxw})
    _load_world_in_place(out_vxw)


func undo_last_edit() -> bool:
    if _undo_stack.is_empty():
        logger.info("undo", {"status": "stack empty"})
        return false
    var entry: Dictionary = _undo_stack.pop_back()
    var op: String = entry["op"]
    if op == "destroy":
        var vi: Vector3i = entry["vi"]
        var mid: int = int(entry["material_id"])
        if renderer.add_voxel(vi, mid):
            voxel_editor._occupied[vi] = true
            _patch_voxel_on_disk(vi, mid, 0)
    elif op == "place":
        var vi2: Vector3i = entry["vi"]
        if renderer.hide_voxel(vi2):
            voxel_editor._occupied.erase(vi2)
            _patch_voxel_on_disk(vi2, 0, 0)
    elif op.begins_with("entity_"):
        entity_edit.apply_undo(entry)
    pause_menu.set_undo_count(_undo_stack.size())
    logger.info("undo", {"op": op, "stack_left": _undo_stack.size()})
    return true




# Map a world voxel index to its (chunk_coord, local_voxel) and patch the chunk
# file with a single voxel cell. material_id=0 means clear to air.
func _patch_voxel_on_disk(world_voxel_index: Vector3i, material_id: int, semantic_id: int) -> void:
    var ce: int = _world.chunk_extent
    var fdiv := Vector3(world_voxel_index) / float(ce)
    var chunk_coord := Vector3i(int(floor(fdiv.x)), int(floor(fdiv.y)), int(floor(fdiv.z)))
    var local := world_voxel_index - chunk_coord * ce
    var chunk_path: String = _world_path_absolute + "/chunks/%d_%d_%d.chunk" % [
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
    _load_world_in_place(_world_path_absolute)


func _on_load_world_requested(new_path: String) -> void:
    logger.info("world_load_requested", {"new_path": new_path, "old_path": _world_path_absolute})
    _close_pause()
    if not DirAccess.dir_exists_absolute(new_path):
        logger.error("world_load_requested", {"reason": "dir missing", "path": new_path})
        return
    if not FileAccess.file_exists(new_path + "/manifest.json"):
        logger.error("world_load_requested", {"reason": "manifest.json missing in selected dir", "path": new_path})
        return
    _load_world_in_place(new_path)


func _load_world_in_place(world_path: String) -> void:
    # In-place world swap: tear down renderer children + re-init controllers.
    # No scene reload needed — keeps CanvasLayer / menu state intact.
    var new_world = VxwLoader.load_world(world_path)
    if new_world.voxel_count() == 0:
        logger.error("world_load_in_place", {"reason": "empty world", "path": world_path})
        return
    _world = new_world
    _world_path_absolute = world_path
    # Tear down old renderer children (MMIs, plane, axes)
    for child in renderer.get_children():
        child.queue_free()
    renderer.build(_world)
    entity_renderer.load_entities(_world_path_absolute, _world.palette_rgb)
    if entity_selector != null:
        entity_selector.set_world_path(_world_path_absolute)
    if entity_edit != null:
        entity_edit.set_world(_world, _world_path_absolute)
    if entity_inspector != null:
        entity_inspector.set_world_path(_world_path_absolute)
    # Re-init editor with new world
    voxel_editor.init_editor(renderer, main_cam)
    # Reset rig + reset stereo cams sync
    stereo_rig.reset_pose()
    # Refresh HUD with new world voxel count / size
    hud_ctl.init_controller(status_label, mode_label, pose_label, _world, stereo_rig, cam_ctl, logger)
    logger.info("world_loaded", {
        "voxels": _world.voxel_count(),
        "voxel_size_m": _world.voxel_size_meters,
        "chunk_extent": _world.chunk_extent,
        "path": _world_path_absolute,
    })


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
