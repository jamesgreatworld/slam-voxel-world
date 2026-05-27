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
var entity_renderer: Node3D = null
var item_picker: CanvasLayer = null
var entity_placer: Node3D = null
var entity_selector: Node3D = null
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
    item_picker.item_chosen.connect(_on_item_chosen)

    entity_placer = Node3D.new()
    entity_placer.set_script(EntityPlacerScript)
    entity_placer.name = "EntityPlacer"
    add_child(entity_placer)
    entity_placer.init_placer(main_cam, voxel_editor, logger)
    entity_placer.placement_committed.connect(_on_placement_committed)

    entity_selector = Node3D.new()
    entity_selector.set_script(EntitySelectorScript)
    entity_selector.name = "EntitySelector"
    add_child(entity_selector)
    entity_selector.init_selector(
        main_cam, _world_path_absolute, entity_renderer, entity_placer, logger
    )
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
            var presets: Dictionary = entity_renderer.get_item_presets()
            if presets.has(sid):
                _spawn_item_in_front_of_rig(String(sid), presets[sid])

    if _test_delete_first or _test_rotate_first_deg != 0.0:
        var rec_list := _read_entities_json(_world_path_absolute + "/entities.json")
        if rec_list.size() > 0:
            var first_id := String(rec_list[0].get("id", ""))
            if _test_rotate_first_deg != 0.0:
                entity_selector.rotate_by_id(first_id, deg_to_rad(_test_rotate_first_deg))
            if _test_delete_first:
                entity_selector.delete_by_id(first_id)


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


func _on_item_chosen(item_id: String) -> void:
    var presets: Dictionary = entity_renderer.get_item_presets()
    if not presets.has(item_id):
        push_error("[main] item_chosen for unknown preset: " + item_id)
        return
    # Hand off to the placer: the user moves the ghost with the mouse and
    # commits with LMB. The placer emits placement_committed → we spawn.
    if entity_placer != null:
        entity_placer.start(item_id, presets[item_id])
        return
    # Fallback (placer not ready): spawn in front of rig immediately.
    _spawn_item_in_front_of_rig(item_id, presets[item_id])


func _on_placement_committed(item_id: String, world_pos: Vector3, yaw_rad: float) -> void:
    var presets: Dictionary = entity_renderer.get_item_presets()
    if not presets.has(item_id):
        return
    _spawn_entity(item_id, presets[item_id], world_pos, yaw_rad)


func _spawn_item_in_front_of_rig(item_id: String, preset: Dictionary) -> void:
    var rig_xf: Transform3D = stereo_rig.global_transform
    var forward: Vector3 = -rig_xf.basis.z.normalized()
    var pos: Vector3 = rig_xf.origin + forward * 1.5
    var yaw: float = rig_xf.basis.get_euler().y
    _spawn_entity(item_id, preset, pos, yaw)


func _spawn_entity(item_id: String, preset: Dictionary, pos: Vector3, yaw_rad: float) -> void:
    var extents = preset.get("overall_extents_m", [0.5, 0.5, 0.5])
    var label: int = int(preset.get("default_label", 0))
    var entity_id := _uuid4()
    var entity: Dictionary = {
        "id": entity_id,
        "label": label,
        "label_name": item_id,
        "position": [pos.x, pos.y, pos.z],
        "rotation": _quat_from_yaw(yaw_rad),
        "bbox_dims": [float(extents[0]), float(extents[1]), float(extents[2])],
        "voxel_count": 0,
        "custom_meta": {"mc_item": item_id, "spawned_at": Time.get_unix_time_from_system()},
    }
    var ent_path := _world_path_absolute + "/entities.json"
    var existing := _read_entities_json(ent_path)
    existing.append(entity)
    _write_entities_json(ent_path, existing)
    logger.info("entity_spawned", {"id": entity_id, "item": item_id,
                                   "pos": [pos.x, pos.y, pos.z],
                                   "yaw_deg": rad_to_deg(yaw_rad),
                                   "total_entities": existing.size()})
    entity_renderer.load_entities(_world_path_absolute, _world.palette_rgb)


func _quat_from_yaw(yaw_rad: float) -> Array:
    var half := yaw_rad * 0.5
    return [0.0, sin(half), 0.0, cos(half)]   # [qx, qy, qz, qw]


func _uuid4() -> String:
    # RFC 4122 v4 — enough randomness for our purposes; not cryptographic.
    var rng := RandomNumberGenerator.new()
    rng.randomize()
    var b := PackedByteArray()
    for i in 16:
        b.append(rng.randi() & 0xff)
    b[6] = (b[6] & 0x0f) | 0x40
    b[8] = (b[8] & 0x3f) | 0x80
    var hex := b.hex_encode()
    return "%s-%s-%s-%s-%s" % [
        hex.substr(0, 8), hex.substr(8, 4), hex.substr(12, 4),
        hex.substr(16, 4), hex.substr(20, 12),
    ]


func _read_entities_json(path: String) -> Array:
    if not FileAccess.file_exists(path):
        return []
    var txt := FileAccess.get_file_as_string(path)
    if txt.is_empty():
        return []
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return []
    return d.entities


func _write_entities_json(path: String, entities: Array) -> void:
    var payload := {"format_version": "1.0", "entities": entities}
    var f := FileAccess.open(path, FileAccess.WRITE)
    if f == null:
        push_error("[main] cannot write entities.json: " + path)
        return
    f.store_string(JSON.stringify(payload, "  "))
    f.close()


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
    var vi: Vector3i = entry["vi"]
    var op: String = entry["op"]
    var mid: int = int(entry["material_id"])
    if op == "destroy":
        # Undo a destroy → re-place the voxel
        if renderer.add_voxel(vi, mid):
            voxel_editor._occupied[vi] = true
            _patch_voxel_on_disk(vi, mid, 0)
    elif op == "place":
        # Undo a place → destroy the voxel
        if renderer.hide_voxel(vi):
            voxel_editor._occupied.erase(vi)
            _patch_voxel_on_disk(vi, 0, 0)
    pause_menu.set_undo_count(_undo_stack.size())
    logger.info("undo", {"op": op, "vi": [vi.x, vi.y, vi.z], "stack_left": _undo_stack.size()})
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
