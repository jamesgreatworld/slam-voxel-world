# voxel_renderer.gd — Layer 4 (rendering).
# Per-material renderer: one MultiMeshInstance3D per material_id present in the
# world, each with its own StandardMaterial3D configured from the Material
# data (transparent / emission / metallic / roughness). This is what makes
# glass look like glass, lamps glow, metal shine.
#
# Public API:
#   build(world)                       build per-material MMIs + reference helpers
#   hide_voxel(world_voxel_index)      remove a voxel from its bucket (returns bool)
#   has_voxel(world_voxel_index)       does the renderer still draw this voxel?
#   get_voxel_size()                   meters per voxel (forwarded from world)
#   get_world_voxel_indices()          iterable of all currently-drawn voxel indices

extends Node3D

var _world = null
# voxel grid index → [MultiMeshInstance3D, local_instance_index, material_id]
var _voxel_to_instance: Dictionary = {}
# material_id → MultiMeshInstance3D (for diagnostics)
var _mmi_by_material: Dictionary = {}
# StaticBody3D for collision (R7)
var _collision_body: StaticBody3D = null
var _collision_shape: CollisionShape3D = null
# Placed-voxel scratch bucket (R8): a separate MMI with pre-allocated slots so we
# can append new voxels at runtime without reallocating the per-material MMIs.
var _placed_mmi: MultiMeshInstance3D = null
var _placed_count: int = 0
const _PLACED_CAPACITY: int = 2000

const _ZERO_BASIS := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)


func build(world) -> void:
    _world = world
    _voxel_to_instance.clear()
    _mmi_by_material.clear()

    # 1) group voxel indices by material_id
    var indices_by_mat: Dictionary = {}
    var n: int = world.voxel_count()
    for i in n:
        var mid: int = int(world.material_ids[i]) if i < world.material_ids.size() else 1
        if not indices_by_mat.has(mid):
            indices_by_mat[mid] = []
        indices_by_mat[mid].append(i)

    # 2) one MMI per material
    for mid in indices_by_mat.keys():
        var indices: Array = indices_by_mat[mid]
        var mat_def: Dictionary = world.palette_materials.get(mid, {})
        _build_material_bucket(mid, indices, world, mat_def)

    _add_reference_helpers(_compute_min_y(world))
    _build_collision_mesh(world)
    _build_placed_voxel_bucket(world)


func hide_voxel(vi: Vector3i) -> bool:
    if not _voxel_to_instance.has(vi):
        return false
    var entry = _voxel_to_instance[vi]
    var mmi: MultiMeshInstance3D = entry[0]
    var idx: int = entry[1]
    mmi.multimesh.set_instance_transform(idx, Transform3D(_ZERO_BASIS, Vector3.ZERO))
    _voxel_to_instance.erase(vi)
    _rebuild_collision_mesh()
    return true


# Place a new voxel at vi with the given material. Returns true on success.
# Visual goes into the pre-allocated PlacedVoxels MMI; collision mesh is
# rebuilt so 1P walking sees the new block.
func add_voxel(vi: Vector3i, material_id: int) -> bool:
    if _voxel_to_instance.has(vi):
        return false   # already occupied
    if _placed_mmi == null or _placed_count >= _PLACED_CAPACITY:
        push_warning("[renderer] placed voxel bucket full (cap=%d)" % _PLACED_CAPACITY)
        return false
    var world_pos := Vector3(vi) * get_voxel_size()
    var t := Transform3D(Basis.IDENTITY, world_pos)
    _placed_mmi.multimesh.set_instance_transform(_placed_count, t)
    _placed_mmi.multimesh.set_instance_color(_placed_count, _color_for_material(material_id))
    _voxel_to_instance[vi] = [_placed_mmi, _placed_count, material_id]
    _placed_count += 1
    _rebuild_collision_mesh()
    return true


func _color_for_material(material_id: int) -> Color:
    if _world != null and material_id >= 0 and material_id < _world.palette_rgb.size():
        return _world.palette_rgb[material_id]
    return Color(1, 1, 1, 1)


func has_voxel(vi: Vector3i) -> bool:
    return _voxel_to_instance.has(vi)


func get_voxel_size() -> float:
    return 0.10 if _world == null else _world.voxel_size_meters


func get_world_voxel_indices() -> Array:
    return _voxel_to_instance.keys()


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------


func _build_material_bucket(mid: int, indices: Array, world, mat_def: Dictionary) -> void:
    var voxel_size: float = world.voxel_size_meters
    var box := BoxMesh.new()
    box.size = Vector3.ONE * voxel_size

    var mm := MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.mesh = box
    mm.instance_count = indices.size()

    var mmi := MultiMeshInstance3D.new()
    mmi.name = "Voxels_Mat%d" % mid
    mmi.material_override = _make_standard_material(mat_def)
    mmi.multimesh = mm
    add_child(mmi)
    _mmi_by_material[mid] = mmi

    # Populate transforms + per-instance colours, and global lookup
    for k in indices.size():
        var i: int = indices[k]
        var p: Vector3 = world.positions[i]
        var t := Transform3D(Basis.IDENTITY, p)
        mm.set_instance_transform(k, t)
        mm.set_instance_color(k, world.colors[i])
        var vi: Vector3i = _world_to_voxel_index(p, voxel_size)
        _voxel_to_instance[vi] = [mmi, k, mid]


func _make_standard_material(m: Dictionary) -> StandardMaterial3D:
    var sm := StandardMaterial3D.new()
    sm.vertex_color_use_as_albedo = true
    if m.is_empty():
        sm.roughness = 0.75
        return sm
    sm.roughness = clamp(float(m.get("roughness", 0.75)), 0.0, 1.0)
    sm.metallic = clamp(float(m.get("metallic", 0.0)), 0.0, 1.0)
    if bool(m.get("transparent", false)):
        sm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
        # Force a moderate alpha; per-instance colour alpha still multiplies in.
        var rgb = m.get("color_rgb", [255, 255, 255])
        sm.albedo_color = Color(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0, 0.45)
        sm.vertex_color_use_as_albedo = false  # use the explicit alpha
    var energy: float = float(m.get("emission_energy", 0.0))
    if energy > 0.0:
        sm.emission_enabled = true
        var er = m.get("emission_rgb", [255, 255, 255])
        sm.emission = Color(er[0] / 255.0, er[1] / 255.0, er[2] / 255.0)
        sm.emission_energy_multiplier = energy
    return sm


func _world_to_voxel_index(p: Vector3, voxel_size: float) -> Vector3i:
    var inv: float = 1.0 / voxel_size
    return Vector3i(
        int(round(p.x * inv)),
        int(round(p.y * inv)),
        int(round(p.z * inv))
    )


func _compute_min_y(world) -> float:
    var y_min: float = INF
    for i in world.voxel_count():
        y_min = min(y_min, world.positions[i].y)
    return y_min


# Rebuild collision mesh from the current _voxel_to_instance set. Called on
# add_voxel / hide_voxel so 1P walking always matches the visible geometry.
# For 1K voxels: ~3K triangles, ~3ms. For 40K SLAM voxels: ~120K triangles,
# ~50ms — acceptable per-event but would need incremental update if rapid.
func _rebuild_collision_mesh() -> void:
    if _world == null or _collision_shape == null:
        return
    _build_collision_mesh(_world)


# Builds a single StaticBody3D + ConcavePolygonShape3D from triangle soup of
# all externally-facing voxel faces. Internal faces (shared by 2 occupied
# neighbors) are skipped to reduce triangle count. This gives 1P walking
# the geometry it needs to collide with.
func _build_collision_mesh(world) -> void:
    var vs: float = world.voxel_size_meters
    var occupied: Dictionary = {}
    for vi in _voxel_to_instance.keys():
        occupied[vi] = true

    # face definitions: (neighbor_offset, 4 corner offsets in CCW order facing outward)
    var faces := [
        # +X (right)
        [Vector3i(1, 0, 0), [Vector3(0.5,-0.5,-0.5), Vector3(0.5,-0.5,0.5), Vector3(0.5,0.5,0.5), Vector3(0.5,0.5,-0.5)]],
        # -X (left)
        [Vector3i(-1, 0, 0), [Vector3(-0.5,-0.5,0.5), Vector3(-0.5,-0.5,-0.5), Vector3(-0.5,0.5,-0.5), Vector3(-0.5,0.5,0.5)]],
        # +Y (top)
        [Vector3i(0, 1, 0), [Vector3(-0.5,0.5,-0.5), Vector3(0.5,0.5,-0.5), Vector3(0.5,0.5,0.5), Vector3(-0.5,0.5,0.5)]],
        # -Y (bottom)
        [Vector3i(0, -1, 0), [Vector3(-0.5,-0.5,0.5), Vector3(0.5,-0.5,0.5), Vector3(0.5,-0.5,-0.5), Vector3(-0.5,-0.5,-0.5)]],
        # +Z (forward) — in Godot Z+ is back; either way, just an external face
        [Vector3i(0, 0, 1), [Vector3(0.5,-0.5,0.5), Vector3(-0.5,-0.5,0.5), Vector3(-0.5,0.5,0.5), Vector3(0.5,0.5,0.5)]],
        # -Z (back)
        [Vector3i(0, 0, -1), [Vector3(-0.5,-0.5,-0.5), Vector3(0.5,-0.5,-0.5), Vector3(0.5,0.5,-0.5), Vector3(-0.5,0.5,-0.5)]],
    ]

    var triangles := PackedVector3Array()
    for vi in occupied.keys():
        var centre := Vector3(vi) * vs
        for face in faces:
            var neighbor: Vector3i = vi + face[0]
            if occupied.has(neighbor):
                continue
            var corners: Array = face[1]
            var v0: Vector3 = centre + corners[0] * vs
            var v1: Vector3 = centre + corners[1] * vs
            var v2: Vector3 = centre + corners[2] * vs
            var v3: Vector3 = centre + corners[3] * vs
            # 2 triangles per quad (CCW)
            triangles.append(v0); triangles.append(v1); triangles.append(v2)
            triangles.append(v0); triangles.append(v2); triangles.append(v3)

    print("[renderer] built collision mesh: %d voxels → %d triangles" %
        [occupied.size(), triangles.size() / 3])

    # On rebuild, reuse the existing CollisionShape3D so PhysicsServer reslices
    # incrementally rather than tearing down the StaticBody3D.
    if _collision_shape == null:
        _collision_shape = CollisionShape3D.new()
        _collision_shape.name = "VoxelCollisionShape"
        _collision_body = StaticBody3D.new()
        _collision_body.name = "VoxelCollisionBody"
        _collision_body.add_child(_collision_shape)
        add_child(_collision_body)
    var shape := ConcavePolygonShape3D.new()
    shape.set_faces(triangles)
    _collision_shape.shape = shape


# Pre-allocate a MultiMesh with PLACED_CAPACITY hidden instance slots; add_voxel
# fills them in order. All slots start invisible (zero-basis transform).
func _build_placed_voxel_bucket(world) -> void:
    var box := BoxMesh.new()
    box.size = Vector3.ONE * world.voxel_size_meters
    var mm := MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.mesh = box
    mm.instance_count = _PLACED_CAPACITY
    for i in _PLACED_CAPACITY:
        mm.set_instance_transform(i, Transform3D(_ZERO_BASIS, Vector3.ZERO))
        mm.set_instance_color(i, Color(1, 1, 1, 1))
    _placed_mmi = MultiMeshInstance3D.new()
    _placed_mmi.name = "PlacedVoxels"
    var mat := StandardMaterial3D.new()
    mat.vertex_color_use_as_albedo = true
    mat.roughness = 0.75
    _placed_mmi.material_override = mat
    _placed_mmi.multimesh = mm
    _placed_count = 0
    add_child(_placed_mmi)


func _add_reference_helpers(plane_y: float) -> void:
    var plane := MeshInstance3D.new()
    plane.name = "GroundPlane"
    var pm := PlaneMesh.new()
    pm.size = Vector2(500, 500)
    plane.mesh = pm
    var pmat := StandardMaterial3D.new()
    pmat.albedo_color = Color(0.2, 0.22, 0.25, 1.0)
    pmat.roughness = 1.0
    plane.material_override = pmat
    plane.position = Vector3(0, plane_y - 0.5, 0)
    add_child(plane)

    var axis_dirs: Array[Vector3] = [Vector3.RIGHT, Vector3.UP, Vector3.FORWARD]
    var axis_cols: Array[Color] = [Color.RED, Color.GREEN, Color.BLUE]
    for axis_idx in 3:
        var im := ImmediateMesh.new()
        var mi := MeshInstance3D.new()
        mi.name = "Axis%d" % axis_idx
        mi.mesh = im
        var lmat := StandardMaterial3D.new()
        lmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
        lmat.albedo_color = axis_cols[axis_idx]
        im.surface_begin(Mesh.PRIMITIVE_LINES, lmat)
        im.surface_add_vertex(Vector3.ZERO)
        im.surface_add_vertex(axis_dirs[axis_idx] * 5.0)
        im.surface_end()
        add_child(mi)


# Legacy compatibility for callers that used to call renderer.get_mmi()
# during the single-MMI era. Returns the first bucket so simple tools still work.
func get_mmi() -> MultiMeshInstance3D:
    if _mmi_by_material.is_empty():
        return null
    var first_mid = _mmi_by_material.keys()[0]
    return _mmi_by_material[first_mid]
