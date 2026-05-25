# voxel_renderer.gd — Layer 4 (rendering).
# Owns the MultiMeshInstance3D that draws every occupied voxel, plus reference
# helpers (ground plane, world origin axes). The caller hands it a VxwWorld;
# the renderer creates child nodes and renders.
#
# Public API (see docs/architecture.md §3.3):
#   build(world)                      build the MMI + helpers from a loaded world
#   get_mmi()                         the MultiMeshInstance3D — exposed for editor's spatial pick
#   get_world()                       the world passed to build() (for editor convenience)
#   get_voxel_count()                 number of voxels currently drawn
#   get_bounds_min_y()                lowest voxel Y (for ground plane positioning)
#
# Replaceability: this whole file can be swapped for a greedy-mesher or
# SVO-raymarch renderer without affecting any other layer, as long as the
# public API stays the same.

extends Node3D

var _world = null
var _mmi: MultiMeshInstance3D = null
var _bounds_min_y: float = 0.0


func build(world) -> void:
    _world = world
    _bounds_min_y = _compute_min_y(world)
    _build_multimesh(world)
    _add_reference_helpers(_bounds_min_y)


func get_mmi() -> MultiMeshInstance3D:
    return _mmi


func get_world():
    return _world


func get_voxel_count() -> int:
    return 0 if _world == null else _world.voxel_count()


func get_bounds_min_y() -> float:
    return _bounds_min_y


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------


func _compute_min_y(world) -> float:
    var y_min: float = INF
    for i in world.voxel_count():
        y_min = min(y_min, world.positions[i].y)
    return y_min


func _build_multimesh(world) -> void:
    var box := BoxMesh.new()
    box.size = Vector3.ONE * world.voxel_size_meters

    var mm := MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.mesh = box
    mm.instance_count = world.voxel_count()

    # Colour by height for visual depth (until per-material rendering arrives in R4)
    var y_min: float = INF
    var y_max: float = -INF
    for i in world.voxel_count():
        var py: float = world.positions[i].y
        y_min = min(y_min, py)
        y_max = max(y_max, py)
    var y_span: float = max(0.01, y_max - y_min)

    for i in world.voxel_count():
        var p: Vector3 = world.positions[i]
        var t := Transform3D(Basis.IDENTITY, p)
        mm.set_instance_transform(i, t)
        var h: float = (p.y - y_min) / y_span
        var c: Color = Color(h, 1.0 - abs(h - 0.5) * 2.0, 1.0 - h, 1.0) * 0.9 + Color(0.1, 0.1, 0.1, 0)
        mm.set_instance_color(i, c)

    _mmi = MultiMeshInstance3D.new()
    _mmi.name = "Voxels"
    var mat := StandardMaterial3D.new()
    mat.vertex_color_use_as_albedo = true
    mat.roughness = 0.75
    mat.metallic = 0.0
    _mmi.material_override = mat
    _mmi.multimesh = mm
    add_child(_mmi)


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
