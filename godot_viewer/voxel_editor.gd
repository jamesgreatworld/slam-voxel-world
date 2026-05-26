# voxel_editor.gd
# Click-to-destroy voxel editor with mouse-cursor picking + hover highlight.
# Decoupled from the renderer's internal MMI layout: talks only to the renderer's
# public API (has_voxel / hide_voxel / get_voxel_size / get_world_voxel_indices),
# so per-material rendering (R4.2) doesn't require any editor changes.

extends Node3D

signal voxel_destroyed(world_voxel_index: Vector3i, world_position_m: Vector3)
signal voxel_placed(world_voxel_index: Vector3i, world_position_m: Vector3, material_id: int)

var _renderer: Node3D = null
var _cam: Camera3D = null
var _voxel_size: float = 0.10
var _edit_enabled: bool = false  # Editor is OFF by default; user opts in via menu
var _current_material_id: int = 1  # what RMB places (1 = stone in MC palette)

# Local mirror of which voxels still exist (for fast iteration during march).
var _occupied: Dictionary = {}   # Vector3i → true

# Highlight box
var _highlight: MeshInstance3D = null
var _placement_highlight: MeshInstance3D = null
var _hovered_vi: Vector3i = Vector3i.ZERO
var _placement_vi: Vector3i = Vector3i.ZERO   # cell-before-hit, where RMB would place
var _has_hover: bool = false

const _ZERO_BASIS := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
const _SENTINEL_FAR := 99999


# New canonical signature: take the renderer (not the raw MMI).
func init_editor(renderer_or_world, mmi_or_cam, cam_arg = null) -> void:
    # Backward compatibility:
    #   init_editor(world, mmi, cam) — pre-R4.2 callers
    #   init_editor(renderer, cam)   — new callers
    if cam_arg != null:
        # 3-arg legacy form
        _renderer = null
        var world = renderer_or_world
        _cam = cam_arg
        _voxel_size = float(world.voxel_size_meters)
        _occupied.clear()
        for i in world.positions.size():
            var p: Vector3 = world.positions[i]
            _occupied[_world_to_voxel_index(p)] = true
    else:
        # 2-arg new form
        _renderer = renderer_or_world
        _cam = mmi_or_cam
        _voxel_size = _renderer.get_voxel_size()
        _occupied.clear()
        for vi in _renderer.get_world_voxel_indices():
            _occupied[vi] = true
    _build_highlight()


func _build_highlight() -> void:
    if _highlight != null:
        return
    # Destroy highlight: yellow box around hovered (existing) voxel
    var hbox := BoxMesh.new()
    hbox.size = Vector3.ONE * _voxel_size * 1.04
    var hmat := StandardMaterial3D.new()
    hmat.albedo_color = Color(1.0, 0.95, 0.2, 0.30)
    hmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    hmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    hmat.cull_mode = BaseMaterial3D.CULL_DISABLED
    hbox.material = hmat
    _highlight = MeshInstance3D.new()
    _highlight.name = "HoverHighlight"
    _highlight.mesh = hbox
    _highlight.visible = false
    add_child(_highlight)

    # Placement preview: green box around the empty cell where RMB would place
    var pbox := BoxMesh.new()
    pbox.size = Vector3.ONE * _voxel_size * 1.0
    var pmat := StandardMaterial3D.new()
    pmat.albedo_color = Color(0.3, 1.0, 0.4, 0.30)
    pmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    pmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    pmat.cull_mode = BaseMaterial3D.CULL_DISABLED
    pbox.material = pmat
    _placement_highlight = MeshInstance3D.new()
    _placement_highlight.name = "PlacementHighlight"
    _placement_highlight.mesh = pbox
    _placement_highlight.visible = false
    add_child(_placement_highlight)


func set_edit_enabled(enabled: bool) -> void:
    _edit_enabled = enabled
    if not enabled:
        if _highlight != null: _highlight.visible = false
        if _placement_highlight != null: _placement_highlight.visible = false
        _has_hover = false


func is_edit_enabled() -> bool:
    return _edit_enabled


func set_current_material(mid: int) -> void:
    _current_material_id = mid


func get_current_material() -> int:
    return _current_material_id


func _process(_delta: float) -> void:
    if not _edit_enabled:
        return
    _update_hover()


func _update_hover() -> void:
    if _cam == null or _highlight == null:
        return
    var vp := _cam.get_viewport()
    if vp == null:
        return
    var mouse_pos: Vector2 = vp.get_mouse_position()
    var ro: Vector3 = _cam.project_ray_origin(mouse_pos)
    var rd: Vector3 = _cam.project_ray_normal(mouse_pos)
    var result := _march_find_with_prev(ro, rd, 200.0)
    if not result.has_hit:
        if _has_hover:
            _highlight.visible = false
            _placement_highlight.visible = false
            _has_hover = false
    else:
        _hovered_vi = result.hit
        _placement_vi = result.prev
        _has_hover = true
        _highlight.global_position = Vector3(_hovered_vi) * _voxel_size
        _highlight.visible = true
        # Show placement preview only when it's actually a distinct empty cell
        if not _occupied.has(_placement_vi):
            _placement_highlight.global_position = Vector3(_placement_vi) * _voxel_size
            _placement_highlight.visible = true
        else:
            _placement_highlight.visible = false


func try_destroy_at_viewport_center(max_distance_m: float = 50.0) -> bool:
    if _cam == null:
        return false
    var vp := _cam.get_viewport()
    if vp == null:
        return false
    var centre: Vector2 = vp.get_visible_rect().size * 0.5
    var ro: Vector3 = _cam.project_ray_origin(centre)
    var rd: Vector3 = _cam.project_ray_normal(centre)
    return _march_and_destroy(ro, rd, max_distance_m)


func try_destroy_at_mouse_position() -> bool:
    if _cam == null:
        return false
    var vp := _cam.get_viewport()
    if vp == null:
        return false
    var pos: Vector2 = vp.get_mouse_position()
    var ro: Vector3 = _cam.project_ray_origin(pos)
    var rd: Vector3 = _cam.project_ray_normal(pos)
    return _march_and_destroy(ro, rd, 200.0)


func try_destroy_along_ray(ray_origin: Vector3, ray_direction: Vector3, max_distance_m: float = 50.0) -> bool:
    return _march_and_destroy(ray_origin, ray_direction, max_distance_m)


func _march_find(ro: Vector3, rd: Vector3, max_distance_m: float) -> Vector3i:
    if rd.length_squared() < 1e-12:
        return Vector3i(_SENTINEL_FAR, _SENTINEL_FAR, _SENTINEL_FAR)
    var dir := rd.normalized()
    var step_len: float = max(0.0001, _voxel_size * 0.5)
    var n_steps: int = int(ceil(max_distance_m / step_len))
    var last_vi: Vector3i = _world_to_voxel_index(ro) + Vector3i(999999, 999999, 999999)
    for s in n_steps + 1:
        var p: Vector3 = ro + dir * (float(s) * step_len)
        var vi: Vector3i = _world_to_voxel_index(p)
        if vi == last_vi:
            continue
        last_vi = vi
        if _occupied.has(vi):
            return vi
    return Vector3i(_SENTINEL_FAR, _SENTINEL_FAR, _SENTINEL_FAR)


# Same as _march_find but also returns the LAST empty cell before the hit
# (the cell where RMB would place a new voxel). Result dict keys:
#   has_hit: bool, hit: Vector3i, prev: Vector3i.
func _march_find_with_prev(ro: Vector3, rd: Vector3, max_distance_m: float) -> Dictionary:
    var r := {"has_hit": false, "hit": Vector3i.ZERO, "prev": Vector3i.ZERO}
    if rd.length_squared() < 1e-12:
        return r
    var dir := rd.normalized()
    var step_len: float = max(0.0001, _voxel_size * 0.5)
    var n_steps: int = int(ceil(max_distance_m / step_len))
    var last_vi: Vector3i = _world_to_voxel_index(ro) + Vector3i(999999, 999999, 999999)
    var prev_empty: Vector3i = _world_to_voxel_index(ro)
    for s in n_steps + 1:
        var p: Vector3 = ro + dir * (float(s) * step_len)
        var vi: Vector3i = _world_to_voxel_index(p)
        if vi == last_vi:
            continue
        if _occupied.has(vi):
            r.has_hit = true
            r.hit = vi
            r.prev = prev_empty
            return r
        prev_empty = vi
        last_vi = vi
    return r


func _march_and_destroy(ro: Vector3, rd: Vector3, max_distance_m: float) -> bool:
    var hit: Vector3i = _march_find(ro, rd, max_distance_m)
    if hit.x == _SENTINEL_FAR:
        return false
    return _destroy_voxel(hit)


func _destroy_voxel(vi: Vector3i) -> bool:
    if not _occupied.has(vi):
        return false
    var ok := true
    if _renderer != null:
        ok = _renderer.hide_voxel(vi)
    if not ok:
        return false
    _occupied.erase(vi)
    if _has_hover and _hovered_vi == vi:
        _highlight.visible = false
        _has_hover = false
    var world_pos := Vector3(vi) * _voxel_size
    emit_signal("voxel_destroyed", vi, world_pos)
    return true


func _world_to_voxel_index(p: Vector3) -> Vector3i:
    var inv: float = 1.0 / _voxel_size
    return Vector3i(
        int(round(p.x * inv)),
        int(round(p.y * inv)),
        int(round(p.z * inv))
    )


func try_place_at_mouse_position(material_id: int = -1) -> bool:
    if not _has_hover:
        return false
    if _occupied.has(_placement_vi):
        return false
    var mid: int = _current_material_id if material_id < 0 else material_id
    if _renderer == null:
        return false
    var ok: bool = _renderer.add_voxel(_placement_vi, mid)
    if not ok:
        return false
    _occupied[_placement_vi] = true
    _placement_highlight.visible = false
    var world_pos := Vector3(_placement_vi) * _voxel_size
    emit_signal("voxel_placed", _placement_vi, world_pos, mid)
    return true


func _input(event: InputEvent) -> void:
    if not _edit_enabled:
        return
    if event is InputEventMouseButton and event.pressed:
        if event.button_index == MOUSE_BUTTON_LEFT:
            try_destroy_at_mouse_position()
        elif event.button_index == MOUSE_BUTTON_RIGHT:
            try_place_at_mouse_position()
