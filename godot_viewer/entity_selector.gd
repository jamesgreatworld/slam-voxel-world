# entity_selector.gd — INPUT + SELECTION only (mouse → intent).
#
# Single responsibility: turn mouse input into selection + edit intents. It
#   - raycasts the entity-pick layer to find the entity under the cursor,
#   - owns the current selection (id + node) and its yellow outline,
#   - calls EntityEditController for every mutation (rotate / move / delete /
#     duplicate live there — the one owner of entities.json + reload).
#
# Mouse-only model (no keyboard):
#   • LMB an entity        → select (outline + bottom bar)
#   • LMB empty world      → deselect
#   • 移动 button           → "awaiting target"; the NEXT LMB places the entity at
#                            the clicked floor point (RMB cancels). It does NOT
#                            follow the cursor.
#   • ⟲ ⟳ 复制 属性 删除   → bottom-bar buttons → controller methods
#
# Runs in _unhandled_input so clicks the GUI already consumed (bottom bar, top
# toolbar, menus) never reach the picker and clear the active selection.

extends Node3D

signal entity_selected(id: String)
signal selection_cleared
signal move_state_changed(active: bool)

const _OUTLINE_INFLATE := 0.04
# Mirrors entity_renderer.ENTITY_META_KEY; duplicated to avoid a runtime
# script-import cycle. Keep in sync if you change the canonical constant.
const ENTITY_META_KEY := "vxw_entity"

var _cam: Camera3D = null
var _entity_renderer = null
var _placer = null
var _voxel_editor = null
var _edit = null            # EntityEditController (set via set_edit_controller)
var _logger = null

var _selected_id: String = ""
var _selected_node: Node3D = null
var _outline: MeshInstance3D = null
var _awaiting_move: bool = false   # move mode: the next LMB places the entity


func init_selector(cam: Camera3D,
                   entity_renderer,
                   placer,
                   voxel_editor,
                   logger = null) -> void:
    _cam = cam
    _entity_renderer = entity_renderer
    _placer = placer
    _voxel_editor = voxel_editor
    _logger = logger
    _build_outline()


func set_edit_controller(edit) -> void:
    _edit = edit


func on_world_changed() -> void:
    clear_selection()


# ---------------------------------------------------------------------------
# Selection state
# ---------------------------------------------------------------------------

func clear_selection() -> void:
    if _selected_id == "":
        return
    if _logger != null:
        _logger.info("entity_deselected", {"id": _selected_id})
    _selected_id = ""
    _selected_node = null
    _awaiting_move = false
    if _outline != null:
        _outline.visible = false
    emit_signal("selection_cleared")


func get_selected_id() -> String:
    return _selected_id


func get_selected_label_name() -> String:
    if not is_instance_valid(_selected_node):
        return ""
    var m: Dictionary = _selected_node.get_meta(ENTITY_META_KEY) \
        if _selected_node.has_meta(ENTITY_META_KEY) else {}
    return String(m.get("label_name", ""))


func _select(id: String, node: Node3D) -> void:
    _selected_id = id
    _selected_node = node
    _update_outline()
    if _logger != null:
        _logger.info("entity_selected", {"id": id})
    emit_signal("entity_selected", id)


# Programmatic select by id (used by test hooks; mouse path uses _select).
func select_by_id(id: String) -> bool:
    var node := _find_node_by_id(id)
    if node == null:
        return false
    _select(id, node)
    return true


# Called by the edit controller after it mutates + reloads (the old nodes were
# freed): re-resolve our node by id and re-sync the outline.
func refresh_selection() -> void:
    if _selected_id == "":
        return
    _selected_node = _find_node_by_id(_selected_id)
    if _selected_node == null:
        clear_selection()
    else:
        _update_outline()


# ---------------------------------------------------------------------------
# Input
# ---------------------------------------------------------------------------

func _is_placer_active() -> bool:
    return _placer != null and _placer.has_method("is_active") and _placer.is_active()


func _unhandled_input(event: InputEvent) -> void:
    if _is_placer_active():
        return
    if not (event is InputEventMouseButton) or not event.pressed:
        return
    # Move mode: the entity is parked; this click chooses where it goes.
    if _awaiting_move:
        if event.button_index == MOUSE_BUTTON_LEFT:
            var p = _floor_point_under_mouse()
            if p != null and _edit != null:
                _edit.move_selected_to(p)
            _set_move_mode(false)
            get_viewport().set_input_as_handled()
        elif event.button_index == MOUSE_BUTTON_RIGHT:
            _set_move_mode(false)
            get_viewport().set_input_as_handled()
        return
    if event.button_index != MOUSE_BUTTON_LEFT:
        return
    # Idle: click an entity to select, click empty world to deselect. Consume
    # only when acting on an entity, so an empty-space click with nothing
    # selected still reaches the camera (double-click walk).
    var ent := _entity_under_mouse()
    if ent == null:
        if _selected_id != "":
            clear_selection()
            get_viewport().set_input_as_handled()
        return
    var id := String((ent.get_meta(ENTITY_META_KEY) as Dictionary).get("id", ""))
    if id == "":
        return
    if id != _selected_id:
        _select(id, ent)
    get_viewport().set_input_as_handled()


# Called by the bottom-bar 移动 button: arm move mode. The entity stays put;
# the next left-click in the world places it at the clicked floor point.
func start_move() -> void:
    if _selected_id == "" or not is_instance_valid(_selected_node):
        return
    _set_move_mode(true)


func _set_move_mode(active: bool) -> void:
    _awaiting_move = active
    emit_signal("move_state_changed", active)
    if _logger != null:
        _logger.info("entity_move_mode", {"active": active, "id": _selected_id})


# ---------------------------------------------------------------------------
# Raycasting
# ---------------------------------------------------------------------------

# The entity Node3D under the cursor (via the pick-layer ray) or null.
func _entity_under_mouse() -> Node3D:
    if _cam == null:
        return null
    var vp := _cam.get_viewport()
    if vp == null:
        return null
    var mouse: Vector2 = vp.get_mouse_position()
    var ro: Vector3 = _cam.project_ray_origin(mouse)
    var rd: Vector3 = _cam.project_ray_normal(mouse)
    var space := _cam.get_world_3d().direct_space_state
    var params := PhysicsRayQueryParameters3D.create(ro, ro + rd * 200.0)
    params.collide_with_areas = false
    params.collide_with_bodies = true
    params.collision_mask = 4   # entity-pick layer ONLY — ignores voxel mesh + rig
    var hit := space.intersect_ray(params)
    if hit.is_empty():
        return null
    return _find_entity_ancestor(hit.collider)


# The floor point under the cursor (cell-snapped, base resting on the surface),
# or null if the ray misses geometry. Used to place an entity in move mode.
func _floor_point_under_mouse():
    if _cam == null or _voxel_editor == null:
        return null
    var vp := _cam.get_viewport()
    if vp == null:
        return null
    var mouse: Vector2 = vp.get_mouse_position()
    var ro: Vector3 = _cam.project_ray_origin(mouse)
    var rd: Vector3 = _cam.project_ray_normal(mouse)
    var result: Dictionary = _voxel_editor._march_find_with_prev(ro, rd, 200.0)
    if not result.has_hit:
        return null
    var vsize: float = _voxel_editor._voxel_size
    var prev: Vector3i = result.prev
    var dy: float = _selected_dims().y
    return Vector3(
        float(prev.x) * vsize + vsize * 0.5,
        float(prev.y) * vsize + dy * 0.5,
        float(prev.z) * vsize + vsize * 0.5,
    )


func _find_entity_ancestor(node: Node) -> Node3D:
    var n: Node = node
    while n != null:
        if n is Node3D and n.has_meta(ENTITY_META_KEY):
            return n
        n = n.get_parent()
    return null


func _find_node_by_id(id: String) -> Node3D:
    if _entity_renderer == null:
        return null
    for child in _entity_renderer.get_children():
        if child is Node3D and not child.is_queued_for_deletion() \
                and child.has_meta(ENTITY_META_KEY):
            var m: Dictionary = child.get_meta(ENTITY_META_KEY)
            if String(m.get("id", "")) == id:
                return child
    return null


# ---------------------------------------------------------------------------
# Outline
# ---------------------------------------------------------------------------

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


# The selected entity's footprint (preset extents, else a unit box).
func _selected_dims() -> Vector3:
    if not is_instance_valid(_selected_node):
        return Vector3.ONE
    var meta: Dictionary = _selected_node.get_meta(ENTITY_META_KEY)
    var cm: Dictionary = meta.get("custom_meta", {})
    var presets: Dictionary = _entity_renderer.get_item_presets() \
        if _entity_renderer != null else {}
    var pid := String(cm.get("mc_item", ""))
    if pid != "" and presets.has(pid):
        var ext = presets[pid].get("overall_extents_m", [1, 1, 1])
        return Vector3(float(ext[0]), float(ext[1]), float(ext[2]))
    return Vector3.ONE


func _update_outline() -> void:
    if _outline == null or not is_instance_valid(_selected_node):
        return
    (_outline.mesh as BoxMesh).size = _selected_dims() + Vector3.ONE * _OUTLINE_INFLATE
    # Match the entity's full transform (rotation + position) exactly so the box
    # stays glued to the model at every angle.
    _outline.global_transform = _selected_node.global_transform
    _outline.visible = true


# ---------------------------------------------------------------------------
# Headless diagnostic (--entity-pick-selftest)
# ---------------------------------------------------------------------------

# For every spawned entity, raycast straight down through its origin on the
# entity-pick layer (mask 4) and report whether a click there would select it.
func selftest_pick_all() -> void:
    if _entity_renderer == null:
        print("[entity-selftest] FAIL no entity_renderer"); return
    var space := get_world_3d().direct_space_state
    var total := 0
    var ok := 0
    for root in _entity_renderer.get_children():
        if not (root is Node3D) or not root.has_meta(ENTITY_META_KEY):
            continue
        total += 1
        var gp: Vector3 = (root as Node3D).global_position
        var q := PhysicsRayQueryParameters3D.create(gp + Vector3(0, 6, 0), gp + Vector3(0, -6, 0))
        q.collision_mask = 4
        var hit := space.intersect_ray(q)
        var meta: Dictionary = root.get_meta(ENTITY_META_KEY)
        var nm := String(meta.get("label_name", "?"))
        if hit.is_empty():
            print("[entity-selftest] MISS  %-12s @%v — nothing on pick layer 4" % [nm, gp])
        elif _find_entity_ancestor(hit.collider) == root:
            ok += 1
            print("[entity-selftest] HIT   %-12s @%v" % [nm, gp])
        else:
            print("[entity-selftest] WRONG %-12s resolved to a different node" % nm)
    print("[entity-selftest] RESULT %d/%d entities pickable on layer 4" % [ok, total])
