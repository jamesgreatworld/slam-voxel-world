# voxel_renderer.gd — Layer 4 (rendering).
# Greedy-meshed renderer: groups voxels into chunks, runs the classic
# 0fps/Mikola-Lysenko 2012 greedy meshing algorithm per chunk per material per
# axis-direction (3 axes × 2 sides = 6 sweeps), and outputs ONE ArrayMesh per
# material as a single MeshInstance3D. Replaces the previous per-voxel
# MultiMesh-of-cubes path which was hitting ~1.5M triangles on the full
# apartment (404K voxels). Greedy meshing typically lands in the 50–100K
# triangle range — a 15–30× reduction, with proportional load-time savings.
#
# Editing API (used by voxel_editor / stereo_rig physics):
#   hide_voxel(vi)   removes the voxel from the occupancy set and rebuilds the
#                    collision mesh. The visual mesh keeps a small residual
#                    face for now (phase-1 trade-off — fully correct repaint
#                    would require per-chunk greedy-mesh rebuild).
#   add_voxel(vi, mid) places a new voxel via the pre-allocated PlacedVoxels
#                    MultiMesh (single-cube instances), matches the editor's
#                    green-preview style and avoids a greedy rebuild on every
#                    click.
#
# Public API (unchanged contract):
#   build(world)
#   hide_voxel(vi) / add_voxel(vi, mid) / has_voxel(vi)
#   get_voxel_size() / get_world_voxel_indices()
#   get_mmi()  legacy shim

extends Node3D

var _world = null
# Occupancy + reverse lookup. For greedy-mesh voxels we just store a stub
# entry [null, -1, mid] — the editor only inspects keys / material_id and
# calls hide_voxel which clears the entry. For placed voxels (PlacedVoxels
# MMI), the entry holds [mmi, slot_idx, mid] like before.
var _voxel_to_instance: Dictionary = {}
# material_id → MeshInstance3D holding the greedy ArrayMesh
var _mesh_by_material: Dictionary = {}
# StaticBody3D for collision (R7)
var _collision_body: StaticBody3D = null
var _collision_shape: CollisionShape3D = null
# Placed-voxel scratch bucket (R8): a separate MMI with pre-allocated slots so
# we can append new voxels at runtime without rebuilding the greedy mesh.
var _placed_mmi: MultiMeshInstance3D = null
var _placed_count: int = 0
const _PLACED_CAPACITY: int = 2000

const _ZERO_BASIS := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)


func build(world) -> void:
    _world = world
    _voxel_to_instance.clear()
    _mesh_by_material.clear()

    var t0_us := Time.get_ticks_usec()

    # 1) bucket voxel indices into chunks of size CHUNK_EXTENT. Per chunk we
    #    keep a *dense* flat PackedByteArray of size extent³ — direct O(1)
    #    indexed access, no hash lookups in the inner mesher. Stub entries
    #    are added to _voxel_to_instance so the editor's occupancy / collision
    #    code keeps working.
    var voxel_size: float = world.voxel_size_meters
    var chunk_extent: int = max(1, int(world.chunk_extent))
    var ee: int = chunk_extent * chunk_extent
    var eee: int = ee * chunk_extent
    var chunks: Dictionary = {}  # Vector3i chunk_coord → PackedByteArray of size eee
    # Per-chunk bounding-box of occupied voxels in chunk-local coords.
    # bbox[c] = [min_x, min_y, min_z, max_x, max_y, max_z] (inclusive).
    # Used to skip empty slabs in the greedy mesher (the apartment has many
    # sparsely-filled boundary chunks where most slices are air).
    var bboxes: Dictionary = {}
    var n: int = world.voxel_count()
    for i in n:
        var p: Vector3 = world.positions[i]
        var vi: Vector3i = _world_to_voxel_index(p, voxel_size)
        var mid: int = int(world.material_ids[i]) if i < world.material_ids.size() else 1
        var cx: int = _floor_div(vi.x, chunk_extent)
        var cy: int = _floor_div(vi.y, chunk_extent)
        var cz: int = _floor_div(vi.z, chunk_extent)
        var c := Vector3i(cx, cy, cz)
        var lx: int = vi.x - cx * chunk_extent
        var ly: int = vi.y - cy * chunk_extent
        var lz: int = vi.z - cz * chunk_extent
        if not chunks.has(c):
            var buf := PackedByteArray()
            buf.resize(eee)
            chunks[c] = buf
            bboxes[c] = [lx, ly, lz, lx, ly, lz]
        var arr: PackedByteArray = chunks[c]
        # Layout: linear = x*ee + y*extent + z.
        arr[lx * ee + ly * chunk_extent + lz] = mid
        chunks[c] = arr
        var bb: Array = bboxes[c]
        if lx < bb[0]: bb[0] = lx
        if ly < bb[1]: bb[1] = ly
        if lz < bb[2]: bb[2] = lz
        if lx > bb[3]: bb[3] = lx
        if ly > bb[4]: bb[4] = ly
        if lz > bb[5]: bb[5] = lz
        bboxes[c] = bb
        # Stub entry for occupancy lookups (editor / collision).
        _voxel_to_instance[vi] = [null, -1, mid]

    var t_bucket_us := Time.get_ticks_usec()

    # 2) accumulate greedy quads per material across all chunks.
    var quads_by_material: Dictionary = {}  # mid → PackedVector3Array of positions
    var normals_by_material: Dictionary = {}  # mid → PackedVector3Array of normals
    var total_tris := 0
    for chunk_coord in chunks.keys():
        total_tris += _greedy_mesh_chunk(
            chunk_coord, chunks[chunk_coord], chunk_extent, voxel_size,
            quads_by_material, normals_by_material,
            bboxes[chunk_coord],
        )

    var t_greedy_us := Time.get_ticks_usec()

    # 3) commit one ArrayMesh + MeshInstance3D per material.
    for mid in quads_by_material.keys():
        var verts: PackedVector3Array = quads_by_material[mid]
        if verts.is_empty():
            continue
        var norms: PackedVector3Array = normals_by_material[mid]
        var arr := []
        arr.resize(Mesh.ARRAY_MAX)
        arr[Mesh.ARRAY_VERTEX] = verts
        arr[Mesh.ARRAY_NORMAL] = norms
        var mesh := ArrayMesh.new()
        mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
        var mi := MeshInstance3D.new()
        mi.name = "VoxelsGreedy_Mat%d" % mid
        mi.mesh = mesh
        var mat_def: Dictionary = world.palette_materials.get(mid, {})
        mi.material_override = _make_standard_material_for_mid(mid, mat_def, world)
        add_child(mi)
        _mesh_by_material[mid] = mi

    var t_commit_us := Time.get_ticks_usec()
    var dt_total: float = (t_commit_us - t0_us) / 1000.0
    var dt_bucket: float = (t_bucket_us - t0_us) / 1000.0
    var dt_greedy: float = (t_greedy_us - t_bucket_us) / 1000.0
    var dt_commit: float = (t_commit_us - t_greedy_us) / 1000.0
    print("[renderer] built greedy mesh: %d voxels, %d chunks, %d tris, %d materials, %.1fms (bucket=%.1f greedy=%.1f commit=%.1f)"
        % [n, chunks.size(), total_tris, _mesh_by_material.size(), dt_total, dt_bucket, dt_greedy, dt_commit])

    _add_reference_helpers(_compute_min_y(world))
    _build_collision_mesh(world)
    _build_placed_voxel_bucket(world)


func hide_voxel(vi: Vector3i) -> bool:
    if not _voxel_to_instance.has(vi):
        return false
    var entry = _voxel_to_instance[vi]
    var mmi = entry[0]
    var idx: int = entry[1]
    # Placed-voxel path: hide the MultiMesh instance in the PlacedVoxels bucket.
    if mmi != null and idx >= 0:
        mmi.multimesh.set_instance_transform(idx, Transform3D(_ZERO_BASIS, Vector3.ZERO))
    # Greedy-mesh voxels: there is no per-voxel instance — the greedy face is
    # baked into a static ArrayMesh. Phase 1 accepts the visual residual; the
    # voxel still leaves the occupancy set so collision + editor logic is
    # consistent.
    _voxel_to_instance.erase(vi)
    _rebuild_collision_mesh()
    return true


# Place a new voxel at vi with the given material. Returns true on success.
# Goes through the pre-allocated PlacedVoxels MMI so we don't have to rebuild
# the (much larger) greedy mesh on every click.
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
# Greedy meshing internals
# ---------------------------------------------------------------------------


# Classic 0fps / Mikola Lysenko (2012) greedy mesher, run on a single chunk.
# Three axes × two sides = six sweeps. Each sweep iterates slices along the
# sweep axis, builds a 2D mask of exposed-face material-ids on each slice,
# then greedy-merges same-mid rectangles into quads.
#
# Performance: the chunk voxel grid is a flat PackedByteArray with layout
# linear = x*ee + y*extent + z. The three sweep paths use specialised
# inner-loop expressions (one per axis) to avoid dispatched index math —
# this is roughly 5–10× faster than the abstract / generic version.
#
# voxels: PackedByteArray of size extent³  (0 = air, otherwise material_id)
func _greedy_mesh_chunk(
    chunk_coord: Vector3i, voxels: PackedByteArray, extent: int, vs: float,
    out_quads: Dictionary, out_normals: Dictionary, bbox: Array,
) -> int:
    var origin := Vector3(chunk_coord) * (float(extent) * vs)
    var ee: int = extent * extent
    var tri_count := 0
    # bbox = [min_x, min_y, min_z, max_x, max_y, max_z], inclusive in
    # chunk-local coords. Slices outside [min-1, max+1] are always all-air.
    var min_x: int = bbox[0]; var min_y: int = bbox[1]; var min_z: int = bbox[2]
    var max_x: int = bbox[3]; var max_y: int = bbox[4]; var max_z: int = bbox[5]
    # Reuse a single mask buffer across all 6 sweeps. We overwrite every cell
    # we read, so no fill() is needed if we drive _greedy_merge_emit with the
    # active sub-rectangle. To keep that simple we just track u/v ranges.
    var mask: PackedInt32Array = PackedInt32Array()
    mask.resize(ee)

    # ---- Sweep axis X (d=0). Slice plane (u=Y, v=Z). ----
    for sign_i in 2:
        var nrm_sign: int = 1 if sign_i == 0 else -1
        var x_lo: int = max(0, min_x)
        var x_hi: int = min(extent - 1, max_x)
        for x in range(x_lo, x_hi + 1):
            var nb_x: int = x + nrm_sign
            var nb_in_chunk: bool = nb_x >= 0 and nb_x < extent
            # Build mask over [min_y..max_y] × [min_z..max_z]; pass that
            # window as the active region to the merger. No fill needed —
            # the merger reads only inside the window.
            for y in range(min_y, max_y + 1):
                var row_base: int = x * ee + y * extent
                var nb_row_base: int = nb_x * ee + y * extent
                var mask_row: int = y * extent
                if nb_in_chunk:
                    for z in range(min_z, max_z + 1):
                        var a_mid: int = voxels[row_base + z]
                        var b_mid: int = voxels[nb_row_base + z]
                        mask[mask_row + z] = a_mid if (a_mid != 0 and b_mid == 0) else 0
                else:
                    for z in range(min_z, max_z + 1):
                        mask[mask_row + z] = voxels[row_base + z]
            tri_count += _greedy_merge_emit(
                mask, extent, out_quads, out_normals,
                origin, vs, 0, 1, 2,
                float(x) + (1.0 if nrm_sign == 1 else 0.0), nrm_sign,
                min_y, max_y, min_z, max_z,
            )

    # ---- Sweep axis Y (d=1). Slice plane (u=Z, v=X). ----
    for sign_i in 2:
        var nrm_sign: int = 1 if sign_i == 0 else -1
        var y_lo: int = max(0, min_y)
        var y_hi: int = min(extent - 1, max_y)
        for y in range(y_lo, y_hi + 1):
            var nb_y: int = y + nrm_sign
            var nb_in_chunk: bool = nb_y >= 0 and nb_y < extent
            for z in range(min_z, max_z + 1):
                var mask_row: int = z * extent
                if nb_in_chunk:
                    for xi in range(min_x, max_x + 1):
                        var a_mid: int = voxels[xi * ee + y * extent + z]
                        var b_mid: int = voxels[xi * ee + nb_y * extent + z]
                        mask[mask_row + xi] = a_mid if (a_mid != 0 and b_mid == 0) else 0
                else:
                    for xi in range(min_x, max_x + 1):
                        mask[mask_row + xi] = voxels[xi * ee + y * extent + z]
            tri_count += _greedy_merge_emit(
                mask, extent, out_quads, out_normals,
                origin, vs, 1, 2, 0,
                float(y) + (1.0 if nrm_sign == 1 else 0.0), nrm_sign,
                min_z, max_z, min_x, max_x,
            )

    # ---- Sweep axis Z (d=2). Slice plane (u=X, v=Y). ----
    for sign_i in 2:
        var nrm_sign: int = 1 if sign_i == 0 else -1
        var z_lo: int = max(0, min_z)
        var z_hi: int = min(extent - 1, max_z)
        for z in range(z_lo, z_hi + 1):
            var nb_z: int = z + nrm_sign
            var nb_in_chunk: bool = nb_z >= 0 and nb_z < extent
            for xi in range(min_x, max_x + 1):
                var mask_row: int = xi * extent
                if nb_in_chunk:
                    for yi in range(min_y, max_y + 1):
                        var a_mid: int = voxels[xi * ee + yi * extent + z]
                        var b_mid: int = voxels[xi * ee + yi * extent + nb_z]
                        mask[mask_row + yi] = a_mid if (a_mid != 0 and b_mid == 0) else 0
                else:
                    for yi in range(min_y, max_y + 1):
                        mask[mask_row + yi] = voxels[xi * ee + yi * extent + z]
            tri_count += _greedy_merge_emit(
                mask, extent, out_quads, out_normals,
                origin, vs, 2, 0, 1,
                float(z) + (1.0 if nrm_sign == 1 else 0.0), nrm_sign,
                min_x, max_x, min_y, max_y,
            )

    return tri_count


# Greedy-merge a 2D mask (extent × extent ints, 0 = empty) into rectangles
# and emit one quad per rectangle. Mask is mutated (visited cells set to 0).
# Indexing: mask[j * extent + k]. Only the sub-rect [j_lo..j_hi] × [k_lo..k_hi]
# is examined (the caller has only populated that region; cells outside may
# contain stale values from previous slices). Returns triangles emitted.
func _greedy_merge_emit(
    mask: PackedInt32Array, extent: int,
    out_quads: Dictionary, out_normals: Dictionary,
    origin: Vector3, vs: float,
    d: int, u: int, v: int,
    face_d_pos: float, nrm_sign: int,
    j_lo: int, j_hi: int, k_lo: int, k_hi: int,
) -> int:
    var tri_count := 0
    var j2: int = j_lo
    while j2 <= j_hi:
        var k2: int = k_lo
        var row_base: int = j2 * extent
        while k2 <= k_hi:
            var m: int = mask[row_base + k2]
            if m == 0:
                k2 += 1
                continue
            var w := 1
            while k2 + w <= k_hi and mask[row_base + k2 + w] == m:
                w += 1
            var h := 1
            var done := false
            while not done and j2 + h <= j_hi:
                var hrow: int = (j2 + h) * extent + k2
                for ww in w:
                    if mask[hrow + ww] != m:
                        done = true
                        break
                if not done:
                    h += 1
            _emit_quad(
                out_quads, out_normals, m,
                origin, vs, d, u, v,
                face_d_pos, float(j2), float(k2),
                float(h), float(w), nrm_sign,
            )
            tri_count += 2
            for hh in h:
                var zrow: int = (j2 + hh) * extent + k2
                for ww in w:
                    mask[zrow + ww] = 0
            k2 += w
        j2 += 1
    return tri_count


# Emit one quad (2 triangles, 6 verts) into the per-material vertex buffer.
# Position is at face_d_pos along axis d, spanning [j,j+h) on u-axis and
# [k,k+w) on v-axis (all in voxel units, then scaled by vs and shifted by
# `origin` which is in meters). Winding is set so the outward normal points
# along nrm_sign * axis-d.
func _emit_quad(
    out_quads: Dictionary, out_normals: Dictionary, mid: int,
    origin: Vector3, vs: float,
    d: int, u: int, v: int,
    face_d_pos: float, j: float, k: float, h: float, w: float, nrm_sign: int,
) -> void:
    # The voxel at integer index vi is centered at vi * vs (matches the
    # legacy MMI box-mesh placement: BoxMesh is centered on its origin, so a
    # voxel at integer index vi occupies the cube
    #   [vi*vs - 0.5*vs, vi*vs + 0.5*vs]
    # in world space. A face at slice index `slice` on the +d side lies at
    # (slice + 0.5) * vs in world-d coordinates; on the -d side at
    # (slice - 0.5) * vs. We pass face_d_pos = slice + (1 if + else 0), so the
    # actual world-d coord is (face_d_pos - 0.5) * vs.
    var d_world: float = (face_d_pos - 0.5) * vs + _axis_component(origin, d)
    # u/v world coords for the 4 corners.
    var u0: float = (j - 0.5) * vs + _axis_component(origin, u)
    var u1: float = (j + h - 0.5) * vs + _axis_component(origin, u)
    var v0: float = (k - 0.5) * vs + _axis_component(origin, v)
    var v1: float = (k + w - 0.5) * vs + _axis_component(origin, v)

    var p00 := _make_vec3(d, u, v, d_world, u0, v0)
    var p01 := _make_vec3(d, u, v, d_world, u0, v1)
    var p10 := _make_vec3(d, u, v, d_world, u1, v0)
    var p11 := _make_vec3(d, u, v, d_world, u1, v1)

    var n := _axis_unit(d) * float(nrm_sign)

    if not out_quads.has(mid):
        out_quads[mid] = PackedVector3Array()
        out_normals[mid] = PackedVector3Array()
    var verts: PackedVector3Array = out_quads[mid]
    var norms: PackedVector3Array = out_normals[mid]

    # Two triangles per quad, winding chosen so the normal matches nrm_sign.
    # For +d (nrm_sign=+1) we want CCW when viewed from +d:
    #   tri1: p00, p10, p11
    #   tri2: p00, p11, p01
    # For -d we flip.
    if nrm_sign == 1:
        verts.append(p00); verts.append(p10); verts.append(p11)
        verts.append(p00); verts.append(p11); verts.append(p01)
    else:
        verts.append(p00); verts.append(p11); verts.append(p10)
        verts.append(p00); verts.append(p01); verts.append(p11)
    for _i in 6:
        norms.append(n)
    out_quads[mid] = verts
    out_normals[mid] = norms


static func _axis_component(p: Vector3, axis: int) -> float:
    if axis == 0: return p.x
    if axis == 1: return p.y
    return p.z


static func _axis_unit(axis: int) -> Vector3:
    if axis == 0: return Vector3(1, 0, 0)
    if axis == 1: return Vector3(0, 1, 0)
    return Vector3(0, 0, 1)


static func _make_vec3(d: int, u: int, v: int, d_val: float, u_val: float, v_val: float) -> Vector3:
    var out := Vector3.ZERO
    var vals := [0.0, 0.0, 0.0]
    vals[d] = d_val
    vals[u] = u_val
    vals[v] = v_val
    out.x = vals[0]
    out.y = vals[1]
    out.z = vals[2]
    return out


# Floor-div that handles negative integer chunk coordinates correctly. GDScript's
# `/` truncates toward zero, but we need true floor-div (so that vi=-1 with
# extent=32 lands in chunk -1, not 0).
static func _floor_div(a: int, b: int) -> int:
    if a >= 0:
        return a / b
    return -(((-a) + b - 1) / b)


# ---------------------------------------------------------------------------
# Misc internals (palette / collision / helpers — unchanged behavior)
# ---------------------------------------------------------------------------


func _make_standard_material_for_mid(mid: int, m: Dictionary, world) -> StandardMaterial3D:
    var sm := StandardMaterial3D.new()
    # No vertex colors in the greedy mesh — each material has a single
    # albedo color drawn from the palette, so we paint via albedo_color.
    if mid >= 0 and mid < world.palette_rgb.size():
        sm.albedo_color = world.palette_rgb[mid]
    if m.is_empty():
        sm.roughness = 0.75
        return sm
    sm.roughness = clamp(float(m.get("roughness", 0.75)), 0.0, 1.0)
    sm.metallic = clamp(float(m.get("metallic", 0.0)), 0.0, 1.0)
    if bool(m.get("transparent", false)):
        sm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
        var rgb = m.get("color_rgb", [255, 255, 255])
        sm.albedo_color = Color(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0, 0.45)
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
