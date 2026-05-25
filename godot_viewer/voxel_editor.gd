# voxel_editor.gd
# Click-to-destroy voxel editor with mouse-cursor picking + hover highlight.
# Decoupled from the renderer's internal MMI layout: talks only to the renderer's
# public API (has_voxel / hide_voxel / get_voxel_size / get_world_voxel_indices),
# so per-material rendering (R4.2) doesn't require any editor changes.

extends Node3D

signal voxel_destroyed(world_voxel_index: Vector3i, world_position_m: Vector3)

var _renderer: Node3D = null
var _cam: Camera3D = null
var _voxel_size: float = 0.10
var _edit_enabled: bool = false  # Editor is OFF by default; user opts in via menu

# Local mirror of which voxels still exist (for fast iteration during march).
var _occupied: Dictionary = {}   # Vector3i → true

# Highlight box
var _highlight: MeshInstance3D = null
var _hovered_vi: Vector3i = Vector3i.ZERO
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
    var box := BoxMesh.new()
    box.size = Vector3.ONE * _voxel_size * 1.04
    var mat := StandardMaterial3D.new()
    mat.albedo_color = Color(1.0, 0.95, 0.2, 0.30)
    mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    mat.cull_mode = BaseMaterial3D.CULL_DISABLED
    box.material = mat
    _highlight = MeshInstance3D.new()
    _highlight.name = "HoverHighlight"
    _highlight.mesh = box
    _highlight.visible = false
    add_child(_highlight)


func set_edit_enabled(enabled: bool) -> void:
    _edit_enabled = enabled
    if not enabled and _highlight != null and _has_hover:
        _highlight.visible = false
        _has_hover = false


func is_edit_enabled() -> bool:
    return _edit_enabled


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
    var hit: Vector3i = _march_find(ro, rd, 200.0)
    if hit.x == _SENTINEL_FAR:
        if _has_hover:
            _highlight.visible = false
            _has_hover = false
    else:
        _hovered_vi = hit
        _has_hover = true
        _highlight.global_position = Vector3(hit) * _voxel_size
        _highlight.visible = true


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


func _input(event: InputEvent) -> void:
    if not _edit_enabled:
        return
    if event is InputEventMouseButton and event.pressed:
        if event.button_index == MOUSE_BUTTON_LEFT:
            try_destroy_at_mouse_position()
