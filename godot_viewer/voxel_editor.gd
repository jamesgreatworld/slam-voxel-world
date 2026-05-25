# voxel_editor.gd
# Click-to-destroy voxel editor with mouse-cursor picking + hover highlight.
#
# Picking modes:
#   - try_destroy_at_mouse_position(): use the current mouse cursor pos
#     (PC editor style, primary entry point — used by _input on left-click)
#   - try_destroy_at_viewport_center(): screen-centre crosshair pick
#     (legacy; kept for selftest)
#   - try_destroy_along_ray(): explicit ray (kept for selftest)
#
# Hover highlight: each frame raycasts from the current mouse position and
# positions a translucent yellow box around the targeted voxel. Click destroys it.
#
# Removal: MultiMesh.instance_count is fixed; hidden instances get a zero-basis
# transform. The internal spatial hash entry is erased so subsequent picks skip
# the removed voxel.

extends Node3D

signal voxel_destroyed(world_voxel_index: Vector3i, world_position_m: Vector3)

var _world = null                   # VxwLoader.VxwWorld (untyped)
var _mmi: MultiMeshInstance3D = null
var _cam: Camera3D = null
var _voxel_size: float = 0.10
var _index_to_instance: Dictionary = {}   # Vector3i → int

# Highlight selection box
var _highlight: MeshInstance3D = null
var _hovered_vi: Vector3i = Vector3i.ZERO
var _has_hover: bool = false

const _ZERO_BASIS := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
const _SENTINEL_FAR := 99999


func init_editor(world, mmi: MultiMeshInstance3D, cam: Camera3D) -> void:
    _world = world
    _mmi = mmi
    _cam = cam
    _voxel_size = float(world.voxel_size_meters)
    _index_to_instance.clear()
    var n: int = world.positions.size()
    for i in n:
        var vi := _world_to_voxel_index(world.positions[i])
        _index_to_instance[vi] = i
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
    mat.no_depth_test = false
    box.material = mat
    _highlight = MeshInstance3D.new()
    _highlight.name = "HoverHighlight"
    _highlight.mesh = box
    _highlight.visible = false
    add_child(_highlight)


func _process(_delta: float) -> void:
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


# Crosshair-style pick (legacy / selftest)
func try_destroy_at_viewport_center(max_distance_m: float = 50.0) -> bool:
    if _cam == null or _mmi == null:
        return false
    var vp := _cam.get_viewport()
    if vp == null:
        return false
    var centre: Vector2 = vp.get_visible_rect().size * 0.5
    var ro: Vector3 = _cam.project_ray_origin(centre)
    var rd: Vector3 = _cam.project_ray_normal(centre)
    return _march_and_destroy(ro, rd, max_distance_m)


func try_destroy_at_mouse_position() -> bool:
    if _cam == null or _mmi == null:
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
        if _index_to_instance.has(vi):
            return vi
    return Vector3i(_SENTINEL_FAR, _SENTINEL_FAR, _SENTINEL_FAR)


func _march_and_destroy(ro: Vector3, rd: Vector3, max_distance_m: float) -> bool:
    var hit: Vector3i = _march_find(ro, rd, max_distance_m)
    if hit.x == _SENTINEL_FAR:
        return false
    if not _index_to_instance.has(hit):
        return false
    var inst_idx: int = _index_to_instance[hit]
    _mmi.multimesh.set_instance_transform(
        inst_idx,
        Transform3D(_ZERO_BASIS, Vector3.ZERO)
    )
    _index_to_instance.erase(hit)
    if _has_hover and _hovered_vi == hit:
        _highlight.visible = false
        _has_hover = false
    var world_pos := Vector3(hit) * _voxel_size
    emit_signal("voxel_destroyed", hit, world_pos)
    return true


func _world_to_voxel_index(p: Vector3) -> Vector3i:
    var inv: float = 1.0 / _voxel_size
    return Vector3i(
        int(round(p.x * inv)),
        int(round(p.y * inv)),
        int(round(p.z * inv))
    )


func _input(event: InputEvent) -> void:
    if event is InputEventMouseButton and event.pressed:
        if event.button_index == MOUSE_BUTTON_LEFT:
            try_destroy_at_mouse_position()
