# entity_placer.gd — interactive placement loop for MC item entities.
#
# Flow:
#   1. main.gd starts placement: placer.start(item_id, preset).
#   2. We build a translucent multi-box ghost following the mouse cursor.
#      Position is the world-space centre of the voxel cell the cursor's
#      ray points at, lifted by half the preset's height so the bottom
#      of the item sits on that cell's top face.
#   3. LMB → commit: emit `placement_committed(item_id, world_pos, yaw)`.
#      main.gd writes a new entity to entities.json and reloads.
#   4. RMB / ESC → cancel: emit `placement_cancelled`.
#   5. R rotates the ghost yaw 45° (cumulative).
#
# We delegate voxel raycasting to the existing voxel_editor (it owns the
# occupied-voxel set + DDA marcher). Tight coupling on _march_find_with_prev
# is acceptable — they share the same world coordinate convention.

extends Node3D

signal placement_committed(item_id: String, world_pos: Vector3, yaw_rad: float)
signal placement_cancelled

const _ROTATION_STEP_RAD := PI / 4.0   # 45°

var _item_id: String = ""
var _preset: Dictionary = {}
var _ghost: Node3D = null
var _yaw: float = 0.0

var _cam: Camera3D = null
var _voxel_editor: Node = null
var _logger = null


func init_placer(cam: Camera3D, voxel_editor: Node, logger = null) -> void:
    _cam = cam
    _voxel_editor = voxel_editor
    _logger = logger


func is_active() -> bool:
    return _item_id != ""


func start(item_id: String, preset: Dictionary) -> void:
    _item_id = item_id
    _preset = preset
    _yaw = 0.0
    _build_ghost()
    if _logger != null:
        _logger.info("placement_started", {"item": item_id})


func cancel() -> void:
    if not is_active():
        return
    if _logger != null:
        _logger.info("placement_cancelled", {"item": _item_id})
    _teardown()
    emit_signal("placement_cancelled")


func _teardown() -> void:
    _item_id = ""
    if _ghost != null:
        _ghost.queue_free()
        _ghost = null


func _process(_delta: float) -> void:
    if not is_active() or _ghost == null or _cam == null:
        return
    _update_ghost_pose()


func _update_ghost_pose() -> void:
    var vp := _cam.get_viewport()
    if vp == null:
        return
    var mouse: Vector2 = vp.get_mouse_position()
    var ro: Vector3 = _cam.project_ray_origin(mouse)
    var rd: Vector3 = _cam.project_ray_normal(mouse)
    var result: Dictionary = _voxel_editor._march_find_with_prev(ro, rd, 200.0)
    if not result.has_hit:
        _ghost.visible = false
        return
    var voxel_size: float = _voxel_editor._voxel_size
    var prev: Vector3i = result.prev
    # Place the item so its bottom touches the *hit* voxel's top face. The
    # "prev" voxel (empty cell above the hit) gives us roughly the right z
    # plane; align entity y so its lowest sub-box sits at the cell's bottom.
    var base: Vector3 = Vector3(prev) * voxel_size
    var dy: float = float(_preset.get("overall_extents_m", [0, 0, 0])[1])
    var pos := Vector3(
        base.x + voxel_size * 0.5,
        base.y + dy * 0.5,                  # centre is half-height above the floor cell
        base.z + voxel_size * 0.5,
    )
    _ghost.global_transform = Transform3D(
        Basis(Quaternion(Vector3.UP, _yaw)),
        pos,
    )
    _ghost.visible = true


func _input(event: InputEvent) -> void:
    if not is_active():
        return
    if event is InputEventMouseButton and event.pressed:
        if event.button_index == MOUSE_BUTTON_LEFT:
            _commit()
            get_viewport().set_input_as_handled()
        elif event.button_index == MOUSE_BUTTON_RIGHT:
            cancel()
            get_viewport().set_input_as_handled()
    elif event is InputEventKey and event.pressed:
        if event.keycode == KEY_ESCAPE:
            cancel()
            get_viewport().set_input_as_handled()
        elif event.keycode == KEY_R:
            _yaw += _ROTATION_STEP_RAD
            get_viewport().set_input_as_handled()


func _commit() -> void:
    if _ghost == null or not _ghost.visible:
        return     # cursor was off the world; ignore the click
    var pos := _ghost.global_position
    var yaw := _yaw
    var item := _item_id
    if _logger != null:
        _logger.info("placement_committed",
                     {"item": item, "pos": [pos.x, pos.y, pos.z], "yaw_deg": rad_to_deg(yaw)})
    _teardown()
    emit_signal("placement_committed", item, pos, yaw)


func _build_ghost() -> void:
    if _ghost != null:
        _ghost.queue_free()
    _ghost = Node3D.new()
    _ghost.name = "ItemGhost"
    add_child(_ghost)
    for box in _preset.get("boxes", []):
        var mi := _box_to_mesh(box)
        if mi != null:
            _ghost.add_child(mi)


func _box_to_mesh(box: Dictionary) -> MeshInstance3D:
    var bmin = box.get("min")
    var bmax = box.get("max")
    if bmin == null or bmax == null or bmin.size() < 3 or bmax.size() < 3:
        return null
    var dx: float = float(bmax[0]) - float(bmin[0])
    var dy: float = float(bmax[1]) - float(bmin[1])
    var dz: float = float(bmax[2]) - float(bmin[2])
    var cx: float = (float(bmax[0]) + float(bmin[0])) * 0.5
    var cy: float = (float(bmax[1]) + float(bmin[1])) * 0.5
    var cz: float = (float(bmax[2]) + float(bmin[2])) * 0.5
    var r := 0.0; var g := 0.0; var bl := 0.0; var n := 0
    var fc: Dictionary = box.get("face_colors") if box.has("face_colors") else {}
    for c in fc.values():
        if c.size() < 3: continue
        r += float(c[0]); g += float(c[1]); bl += float(c[2]); n += 1
    var col := Color(0.6, 0.6, 0.6, 0.45)
    if n > 0:
        col = Color(r / n / 255.0, g / n / 255.0, bl / n / 255.0, 0.5)
    var mesh := BoxMesh.new()
    mesh.size = Vector3(dx, dy, dz)
    var mat := StandardMaterial3D.new()
    mat.albedo_color = col
    mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    mat.cull_mode = BaseMaterial3D.CULL_DISABLED
    mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    var mi := MeshInstance3D.new()
    mi.mesh = mesh
    mi.material_override = mat
    mi.transform = Transform3D(Basis(), Vector3(cx, cy, cz))
    return mi
