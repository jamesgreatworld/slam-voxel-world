# voxel_renderer.gd — Layer 4 (rendering).
# Per-voxel cube renderer: every occupied voxel is rendered as an individual
# solid BoxMesh cube (Minecraft-style one-block-per-voxel) via a single
# MultiMeshInstance3D (_world_mmi).  A full MMI rebuild is done whenever the
# voxel set changes (hide/add), which is cheap at ~21k voxels on the apartment
# world.
#
# Editing API (used by voxel_editor / stereo_rig physics / main.gd undo):
#   hide_voxel(vi)        removes the voxel from the occupancy set, rebuilds
#                         _world_mmi, and updates the collision mesh.
#   add_voxel(vi, mid)    inserts an immediate green-preview cube via the
#                         pre-allocated PlacedVoxels MultiMesh AND marks
#                         _world_mmi dirty so _process rebuilds it next tick.
#
# Public API (unchanged contract):
#   build(world)
#   hide_voxel(vi) / add_voxel(vi, mid) / has_voxel(vi)
#   get_voxel_size() / get_world_voxel_indices()
#   get_mmi()  legacy shim

extends Node3D

var _world = null
# Cached world params so editing paths don't need a world handle.
var _voxel_size: float = 0.10
var _chunk_extent: int = 32
# Occupancy + reverse lookup. Every entry is a stub [null, -1, mid].
# For voxels just placed via add_voxel the entry holds [_placed_mmi, slot_idx, mid]
# so hide_voxel can immediately blank the MMI slot while the full rebuild is pending.
var _voxel_to_instance: Dictionary = {}
# Dirty flag: true when _world_mmi needs rebuilding.
var _world_mmi_dirty: bool = false
# StaticBody3D for collision (R7)
var _collision_body: StaticBody3D = null
var _collision_shape: CollisionShape3D = null
# World-voxel cube MultiMesh: one BoxMesh instance per occupied world voxel.
var _world_mmi: MultiMeshInstance3D = null
# Placed-voxel scratch bucket: a MMI with pre-allocated slots so add_voxel
# can show a cube immediately, before the next _process tick rebuilds _world_mmi.
var _placed_mmi: MultiMeshInstance3D = null
var _placed_count: int = 0
const _PLACED_CAPACITY: int = 2000

const _ZERO_BASIS := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
const VOXEL_GRID_SHADER := preload("res://voxel_grid.gdshader")


func build(world) -> void:
    _world = world
    _voxel_size = world.voxel_size_meters
    _chunk_extent = max(1, int(world.chunk_extent))
    _voxel_to_instance.clear()
    _world_mmi_dirty = false

    var t0_us := Time.get_ticks_usec()

    # 1) Populate _voxel_to_instance (occupancy + material lookup).
    var n: int = world.voxel_count()
    for i in n:
        var p: Vector3 = world.positions[i]
        var vi: Vector3i = _world_to_voxel_index(p, _voxel_size)
        var mid: int = int(world.material_ids[i]) if i < world.material_ids.size() else 1
        _voxel_to_instance[vi] = [null, -1, mid]

    var t_bucket_us := Time.get_ticks_usec()

    # 2) Build the per-voxel cube MultiMesh (no greedy merging).
    _rebuild_world_mmi()

    var t_commit_us := Time.get_ticks_usec()
    var dt_total: float = (t_commit_us - t0_us) / 1000.0
    var dt_bucket: float = (t_bucket_us - t0_us) / 1000.0
    var dt_mmi: float = (t_commit_us - t_bucket_us) / 1000.0
    print("[renderer] built per-voxel cube MMI: %d voxels, %.1fms (bucket=%.1f mmi=%.1f)"
        % [n, dt_total, dt_bucket, dt_mmi])

    _add_reference_helpers(_compute_min_y(world))
    _build_collision_mesh(world)
    _build_placed_voxel_bucket(world)


func hide_voxel(vi: Vector3i) -> bool:
    if not _voxel_to_instance.has(vi):
        return false
    var entry = _voxel_to_instance[vi]
    var mmi = entry[0]
    var idx: int = entry[1]
    # If this voxel was a placed-MMI instance, blank the slot immediately so
    # there is no visual residual until _world_mmi is rebuilt.
    if mmi != null and idx >= 0:
        mmi.multimesh.set_instance_transform(idx, Transform3D(_ZERO_BASIS, Vector3.ZERO))
    _voxel_to_instance.erase(vi)
    # Mark world MMI dirty; _process will rebuild it next tick.
    _world_mmi_dirty = true
    _rebuild_collision_mesh()
    return true


# Place a new voxel at vi with the given material. Returns true on success.
# Goes through the pre-allocated PlacedVoxels MMI so the user sees the cube
# instantly (in the editor's green-preview style). _world_mmi is also marked
# dirty so _process folds the voxel into the world MMI on the next tick —
# at which point the placed-MMI slot is released back to the pool.
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
    _world_mmi_dirty = true
    _rebuild_collision_mesh()
    return true


func _process(_delta: float) -> void:
    if not _world_mmi_dirty:
        return
    _world_mmi_dirty = false
    var t0_us := Time.get_ticks_usec()
    _rebuild_world_mmi()
    var dt_ms: float = (Time.get_ticks_usec() - t0_us) / 1000.0
    print("[renderer] world MMI rebuilt: %d voxels in %.2fms"
        % [_voxel_to_instance.size(), dt_ms])


# Build (or rebuild) the per-voxel cube MultiMesh for all occupied world voxels.
# Uses a single MultiMesh with one BoxMesh instance per occupied voxel.
# Per-instance color = palette color for that voxel's material.
# Any placed-MMI preview slots are also rewritten to stub entries since the
# world MMI now covers those voxels.
func _rebuild_world_mmi() -> void:
    # Collect all voxels that are in _voxel_to_instance (stub or placed).
    var all_vis: Array = _voxel_to_instance.keys()
    var count: int = all_vis.size()

    var box := BoxMesh.new()
    box.size = Vector3.ONE * _voxel_size
    var mm := MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.mesh = box
    mm.instance_count = count

    for i in count:
        var vi: Vector3i = all_vis[i]
        var entry = _voxel_to_instance[vi]
        var mid: int = int(entry[2])
        var world_pos := Vector3(vi) * _voxel_size
        mm.set_instance_transform(i, Transform3D(Basis.IDENTITY, world_pos))
        mm.set_instance_color(i, _color_for_material(mid))
        # Rewrite any placed-MMI slot entries to stub form (the world MMI now
        # represents this voxel visually, so the placed slot can be blanked).
        if entry[0] != null and int(entry[1]) >= 0:
            var old_mmi = entry[0]
            var old_idx: int = int(entry[1])
            if is_instance_valid(old_mmi) and old_mmi.multimesh != null:
                old_mmi.multimesh.set_instance_transform(old_idx, Transform3D(_ZERO_BASIS, Vector3.ZERO))
            _voxel_to_instance[vi] = [null, -1, mid]

    # Reset placed-count since all placed voxels are now in the world MMI.
    _placed_count = 0

    # Replace or create the world MMI node.
    if _world_mmi != null and is_instance_valid(_world_mmi):
        _world_mmi.queue_free()
        _world_mmi = null
    var mmi := MultiMeshInstance3D.new()
    mmi.name = "WorldVoxelCubes"
    var mat := StandardMaterial3D.new()
    mat.vertex_color_use_as_albedo = true
    mat.roughness = 1.0
    mat.metallic = 0.0
    mmi.material_override = mat
    mmi.multimesh = mm
    add_child(mmi)
    _world_mmi = mmi


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
# Misc internals (palette / collision / helpers — unchanged behavior)
# ---------------------------------------------------------------------------


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
func _rebuild_collision_mesh() -> void:
    if _world == null or _collision_shape == null:
        return
    _build_collision_mesh(_world)


# Builds a single StaticBody3D + ConcavePolygonShape3D from triangle soup of
# all externally-facing voxel faces. Internal faces (shared by 2 occupied
# neighbors) are skipped to reduce triangle count.
func _build_collision_mesh(world) -> void:
    var vs: float = world.voxel_size_meters
    var occupied: Dictionary = {}
    for vi in _voxel_to_instance.keys():
        occupied[vi] = true

    var faces := [
        [Vector3i(1, 0, 0), [Vector3(0.5,-0.5,-0.5), Vector3(0.5,-0.5,0.5), Vector3(0.5,0.5,0.5), Vector3(0.5,0.5,-0.5)]],
        [Vector3i(-1, 0, 0), [Vector3(-0.5,-0.5,0.5), Vector3(-0.5,-0.5,-0.5), Vector3(-0.5,0.5,-0.5), Vector3(-0.5,0.5,0.5)]],
        [Vector3i(0, 1, 0), [Vector3(-0.5,0.5,-0.5), Vector3(0.5,0.5,-0.5), Vector3(0.5,0.5,0.5), Vector3(-0.5,0.5,0.5)]],
        [Vector3i(0, -1, 0), [Vector3(-0.5,-0.5,0.5), Vector3(0.5,-0.5,0.5), Vector3(0.5,-0.5,-0.5), Vector3(-0.5,-0.5,-0.5)]],
        [Vector3i(0, 0, 1), [Vector3(0.5,-0.5,0.5), Vector3(-0.5,-0.5,0.5), Vector3(-0.5,0.5,0.5), Vector3(0.5,0.5,0.5)]],
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
            triangles.append(v0); triangles.append(v1); triangles.append(v2)
            triangles.append(v0); triangles.append(v2); triangles.append(v3)

    print("[renderer] built collision mesh: %d voxels → %d triangles" %
        [occupied.size(), triangles.size() / 3])

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


# Legacy compatibility for callers that used to call renderer.get_mmi(). The
# greedy mesh path no longer has a "primary MMI", so we return the
# PlacedVoxels MMI (the only MultiMesh still in the tree). Simple tools that
# only need a non-null MultiMeshInstance3D handle continue to work.
func get_mmi() -> MultiMeshInstance3D:
    return _placed_mmi
