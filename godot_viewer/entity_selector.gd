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

const _ROTATION_STEP_RAD := PI / 4.0
const _OUTLINE_INFLATE := 0.04
# Mirrors entity_renderer.ENTITY_META_KEY; duplicated to avoid a runtime
# script-import cycle. Keep in sync if you change the canonical constant.
const ENTITY_META_KEY := "vxw_entity"

var _cam: Camera3D = null
var _world_path: String = ""
var _entity_renderer = null
var _placer = null
var _logger = null

var _selected_id: String = ""
var _selected_node: Node3D = null
var _outline: MeshInstance3D = null


func init_selector(cam: Camera3D,
                   world_path: String,
                   entity_renderer,
                   placer,
                   logger = null) -> void:
    _cam = cam
    _world_path = world_path
    _entity_renderer = entity_renderer
    _placer = placer
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
    if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
        _try_pick_at_mouse()
    elif event is InputEventKey and event.pressed:
        if event.keycode == KEY_DELETE and _selected_id != "":
            _delete_selected()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_R and _selected_id != "":
            _rotate_selected(_ROTATION_STEP_RAD)
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_ESCAPE and _selected_id != "":
            clear_selection()
            get_viewport().set_input_as_handled()


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
    var removed := false
    for e in entities:
        if String(e.get("id", "")) == id:
            removed = true
            continue
        kept.append(e)
    if removed:
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
        e["rotation"] = [new_q.x, new_q.y, new_q.z, new_q.w]
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


