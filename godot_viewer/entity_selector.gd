# entity_selector.gd — click-to-select / Delete / R-rotate for placed entities.
#
# Skipped if the placer is active (the placer eats LMB for placement). When
# idle:
#   - LMB on an entity (via PhysicsRay → entity_renderer's pick proxy) selects
#     it. A wireframe-yellow outline AABB is drawn at the entity's transform.
#   - LMB on empty world cancels the selection.
#   - Delete removes the selected entity from entities.json and reloads.
#   - R rotates the selected entity 45° (yaw) and rewrites entities.json.
#   - ESC cancels the selection.
#
# Outside of these explicit actions we don't touch input — the camera /
# voxel_editor / pause menu continue to receive their normal events.

extends Node3D

signal entity_selected(id: String)
signal entity_changed(id: String)
signal entity_deleted(id: String)
signal selection_cleared

# Pre-change snapshots for undo. Emitted *before* the mutation so the
# listener (main.gd) can stash the old state in its undo stack.
signal will_delete(entity_dict: Dictionary)
signal will_move(id: String, from_pos: Array, to_pos: Array)
signal will_rotate(id: String, from_quat: Array, to_quat: Array)

const _ROTATION_STEP_RAD := PI / 4.0
const _OUTLINE_INFLATE := 0.04
# Mirrors entity_renderer.ENTITY_META_KEY; duplicated to avoid a runtime
# script-import cycle. Keep in sync if you change the canonical constant.
const ENTITY_META_KEY := "vxw_entity"

var _cam: Camera3D = null
var _world_path: String = ""
var _entity_renderer = null
var _placer = null
var _voxel_editor = null
var _logger = null

var _selected_id: String = ""
var _selected_node: Node3D = null
var _outline: MeshInstance3D = null

# Grab mode state: when not "" we're moving _selected_node with the mouse.
var _grab_active: bool = false
var _grab_original_pos: Vector3 = Vector3.ZERO


func init_selector(cam: Camera3D,
                   world_path: String,
                   entity_renderer,
                   placer,
                   voxel_editor,
                   logger = null) -> void:
    _cam = cam
    _world_path = world_path
    _entity_renderer = entity_renderer
    _placer = placer
    _voxel_editor = voxel_editor
    _logger = logger
    _build_outline()


func set_world_path(p: String) -> void:
    _world_path = p
    clear_selection()


func clear_selection() -> void:
    if _selected_id == "":
        return
    if _logger != null:
        _logger.info("entity_deselected", {"id": _selected_id})
    _selected_id = ""
    _selected_node = null
    if _outline != null:
        _outline.visible = false
    emit_signal("selection_cleared")


func _build_outline() -> void:
    _outline = MeshInstance3D.new()
    _outline.name = "SelectionOutline"
    var mesh := BoxMesh.new()
    mesh.size = Vector3.ONE
    _outline.mesh = mesh
    var mat := StandardMaterial3D.new()
    mat.albedo_color = Color(1.0, 0.95, 0.2, 0.35)
    mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    mat.cull_mode = BaseMaterial3D.CULL_DISABLED
    mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    mat.no_depth_test = true
    _outline.material_override = mat
    _outline.visible = false
    add_child(_outline)


func _is_placer_active() -> bool:
    return _placer != null and _placer.has_method("is_active") and _placer.is_active()


func _input(event: InputEvent) -> void:
    if _is_placer_active():
        return
    # Grab mode owns LMB (confirm) / ESC (cancel); everything else passes
    # through to whichever subsystem normally handles it.
    if _grab_active:
        if event is InputEventMouseButton and event.pressed:
            if event.button_index == MOUSE_BUTTON_LEFT:
                _commit_grab()
                get_viewport().set_input_as_handled()
            elif event.button_index == MOUSE_BUTTON_RIGHT:
                _cancel_grab()
                get_viewport().set_input_as_handled()
        elif event is InputEventKey and event.pressed:
            if event.keycode == KEY_ESCAPE:
                _cancel_grab()
                get_viewport().set_input_as_handled()
        return

    if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
        _try_pick_at_mouse()
    elif event is InputEventKey and event.pressed:
        if event.keycode == KEY_DELETE and _selected_id != "":
            _delete_selected()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_R and _selected_id != "":
            _rotate_selected(_ROTATION_STEP_RAD)
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_G and _selected_id != "":
            _start_grab()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_ESCAPE and _selected_id != "":
            clear_selection()
            get_viewport().set_input_as_handled()


func _process(_delta: float) -> void:
    if _grab_active:
        _update_grab_position()


func _try_pick_at_mouse() -> void:
    if _cam == null:
        return
    var vp := _cam.get_viewport()
    if vp == null:
        return
    var mouse: Vector2 = vp.get_mouse_position()
    var ro: Vector3 = _cam.project_ray_origin(mouse)
    var rd: Vector3 = _cam.project_ray_normal(mouse)
    var space := _cam.get_world_3d().direct_space_state
    var params := PhysicsRayQueryParameters3D.create(ro, ro + rd * 200.0)
    params.collide_with_areas = false
    params.collide_with_bodies = true
    var hit := space.intersect_ray(params)
    if hit.is_empty():
        clear_selection()
        return
    var entity_node := _find_entity_ancestor(hit.collider)
    if entity_node == null:
        # Hit something else (e.g. a voxel collision mesh) → clear selection.
        clear_selection()
        return
    var meta: Dictionary = entity_node.get_meta(ENTITY_META_KEY)
    var id := String(meta.get("id", ""))
    if id == "":
        return
    _select(id, entity_node)


func _find_entity_ancestor(node: Node) -> Node3D:
    var n: Node = node
    while n != null:
        if n is Node3D and n.has_meta(ENTITY_META_KEY):
            return n
        n = n.get_parent()
    return null


func _select(id: String, node: Node3D) -> void:
    _selected_id = id
    _selected_node = node
    _update_outline()
    if _logger != null:
        _logger.info("entity_selected", {"id": id})
    emit_signal("entity_selected", id)


func _update_outline() -> void:
    if _outline == null or _selected_node == null:
        return
    var meta: Dictionary = _selected_node.get_meta(ENTITY_META_KEY)
    var cm: Dictionary = meta.get("custom_meta", {})
    var dims: Vector3 = Vector3.ONE
    var presets: Dictionary = _entity_renderer.get_item_presets() \
        if _entity_renderer != null else {}
    var preset_id: String = String(cm.get("mc_item", ""))
    if preset_id != "" and presets.has(preset_id):
        var ext = presets[preset_id].get("overall_extents_m", [1, 1, 1])
        dims = Vector3(float(ext[0]), float(ext[1]), float(ext[2]))
    else:
        # Fallback: read bbox from the entities.json record itself.
        var rec := _find_record(_selected_id)
        if not rec.is_empty():
            var b = rec.get("bbox_dims", [1, 1, 1])
            dims = Vector3(float(b[0]), float(b[1]), float(b[2]))
    var inflated := dims + Vector3.ONE * _OUTLINE_INFLATE
    (_outline.mesh as BoxMesh).size = inflated
    _outline.global_transform = _selected_node.global_transform
    _outline.visible = true


func _find_record(id: String) -> Dictionary:
    var entities := _read_entities()
    for e in entities:
        if String(e.get("id", "")) == id:
            return e
    return {}


func _read_entities() -> Array:
    var path := _world_path + "/entities.json"
    if not FileAccess.file_exists(path):
        return []
    var txt := FileAccess.get_file_as_string(path)
    if txt.is_empty():
        return []
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return []
    return d.entities


func _write_entities(entities: Array) -> void:
    var path := _world_path + "/entities.json"
    var f := FileAccess.open(path, FileAccess.WRITE)
    if f == null:
        push_error("[selector] cannot write entities.json: " + path)
        return
    f.store_string(JSON.stringify({"format_version": "1.0", "entities": entities}, "  "))
    f.close()


# ---------------------------------------------------------------------------
# Grab mode
# ---------------------------------------------------------------------------

func _start_grab() -> void:
    if _selected_node == null or _voxel_editor == null:
        return
    _grab_active = true
    _grab_original_pos = _selected_node.global_position
    if _logger != null:
        _logger.info("entity_grab_started", {"id": _selected_id})


func _update_grab_position() -> void:
    if _selected_node == null or _cam == null or _voxel_editor == null:
        return
    var vp := _cam.get_viewport()
    if vp == null:
        return
    var mouse: Vector2 = vp.get_mouse_position()
    var ro: Vector3 = _cam.project_ray_origin(mouse)
    var rd: Vector3 = _cam.project_ray_normal(mouse)
    var result: Dictionary = _voxel_editor._march_find_with_prev(ro, rd, 200.0)
    if not result.has_hit:
        return
    var vsize: float = _voxel_editor._voxel_size
    var prev: Vector3i = result.prev
    var meta: Dictionary = _selected_node.get_meta(ENTITY_META_KEY)
    var rec := _find_record(_selected_id)
    var dy := 1.0
    if not rec.is_empty():
        var bb = rec.get("bbox_dims", [1, 1, 1])
        dy = float(bb[1])
    var pos := Vector3(
        Vector3(prev).x * vsize + vsize * 0.5,
        Vector3(prev).y * vsize + dy * 0.5,
        Vector3(prev).z * vsize + vsize * 0.5,
    )
    var xf := _selected_node.global_transform
    xf.origin = pos
    _selected_node.global_transform = xf
    if _outline != null:
        var ox := _outline.global_transform
        ox.origin = pos
        _outline.global_transform = ox


func _commit_grab() -> void:
    if _selected_node == null:
        _grab_active = false
        return
    var new_pos := _selected_node.global_position
    var entities := _read_entities()
    var changed := false
    for e in entities:
        if String(e.get("id", "")) != _selected_id:
            continue
        var old_pos = e.get("position", [0, 0, 0])
        var new_pos_arr := [new_pos.x, new_pos.y, new_pos.z]
        emit_signal("will_move", _selected_id,
                    [float(old_pos[0]), float(old_pos[1]), float(old_pos[2])],
                    new_pos_arr)
        e["position"] = new_pos_arr
        changed = true
        break
    if changed:
        _write_entities(entities)
        if _logger != null:
            _logger.info("entity_moved",
                         {"id": _selected_id, "pos": [new_pos.x, new_pos.y, new_pos.z]})
        emit_signal("entity_changed", _selected_id)
    _grab_active = false
    _reload_entities()


func _cancel_grab() -> void:
    if _selected_node != null:
        var xf := _selected_node.global_transform
        xf.origin = _grab_original_pos
        _selected_node.global_transform = xf
        if _outline != null:
            var ox := _outline.global_transform
            ox.origin = _grab_original_pos
            _outline.global_transform = ox
    _grab_active = false
    if _logger != null:
        _logger.info("entity_grab_cancelled", {"id": _selected_id})


func grab_to(id: String, world_pos: Vector3) -> void:
    """Programmatic move (for tests). Writes entities.json directly without
    raycasting; useful for snapshot verification."""
    var entities := _read_entities()
    var changed := false
    for e in entities:
        if String(e.get("id", "")) != id:
            continue
        var old_pos = e.get("position", [0, 0, 0])
        var new_pos_arr := [world_pos.x, world_pos.y, world_pos.z]
        emit_signal("will_move", id,
                    [float(old_pos[0]), float(old_pos[1]), float(old_pos[2])],
                    new_pos_arr)
        e["position"] = new_pos_arr
        changed = true
        break
    if changed:
        _write_entities(entities)
        if _logger != null:
            _logger.info("entity_moved",
                         {"id": id, "pos": [world_pos.x, world_pos.y, world_pos.z]})
        _reload_entities()


func delete_by_id(id: String) -> bool:
    """Programmatic delete (for tests / CLI). Returns true if found."""
    var prev_id := _selected_id
    _selected_id = id
    _delete_selected()
    if _selected_id != prev_id and prev_id != "" and prev_id != id:
        # restore prior selection
        _selected_id = prev_id
    return true


func rotate_by_id(id: String, yaw_delta_rad: float) -> void:
    var prev_id := _selected_id
    _selected_id = id
    _rotate_selected(yaw_delta_rad)
    _selected_id = prev_id


func _delete_selected() -> void:
    var id := _selected_id
    var entities := _read_entities()
    var kept: Array = []
    var removed: Dictionary = {}
    for e in entities:
        if String(e.get("id", "")) == id:
            removed = e
            continue
        kept.append(e)
    if removed.is_empty():
        return
    emit_signal("will_delete", removed)
    _write_entities(kept)
    if _logger != null:
        _logger.info("entity_deleted", {"id": id, "remaining": kept.size()})
    emit_signal("entity_deleted", id)
    clear_selection()
    _reload_entities()


func _rotate_selected(yaw_delta_rad: float) -> void:
    var id := _selected_id
    var entities := _read_entities()
    var changed := false
    for e in entities:
        if String(e.get("id", "")) != id:
            continue
        var rot = e.get("rotation", [0, 0, 0, 1])
        var cur := Quaternion(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
        var step := Quaternion(Vector3.UP, yaw_delta_rad)
        var new_q := step * cur
        var from_q := [rot[0], rot[1], rot[2], rot[3]]
        var to_q := [new_q.x, new_q.y, new_q.z, new_q.w]
        emit_signal("will_rotate", id, from_q, to_q)
        e["rotation"] = to_q
        changed = true
        break
    if changed:
        _write_entities(entities)
        if _logger != null:
            _logger.info("entity_rotated",
                         {"id": id, "yaw_deg": rad_to_deg(yaw_delta_rad)})
        emit_signal("entity_changed", id)
        _reload_entities()


func _reload_entities() -> void:
    if _entity_renderer == null:
        return
    var palette = null
    var root := get_parent()
    if root != null and root.has_method("get") and "_world" in root:
        palette = root._world.palette_rgb
    if palette == null:
        return
    _entity_renderer.load_entities(_world_path, palette)
    # Re-resolve our selected node after the reload (nodes were freed).
    _selected_node = _find_node_by_id(_selected_id) if _selected_id != "" else null
    if _selected_node == null:
        clear_selection()
    else:
        _update_outline()


func _find_node_by_id(id: String) -> Node3D:
    if _entity_renderer == null:
        return null
    for child in _entity_renderer.get_children():
        if child is Node3D and child.has_meta(ENTITY_META_KEY):
            var m: Dictionary = child.get_meta(ENTITY_META_KEY)
            if String(m.get("id", "")) == id:
                return child
    return null


