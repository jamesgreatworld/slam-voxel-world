# entity_renderer.gd — load entities.json from a .vxw world and spawn one
# selectable Node3D per entity (BoxMesh + StandardMaterial3D using the palette
# color of the entity's label). Independent from the voxel renderer:
# entities live outside the voxel grid (M2b extracts them out of chunks).
#
# Each spawned node carries entity metadata in its `metadata` so future
# edit/move/delete tooling can identify it.
#
# Rendering modes:
#   - default: one BoxMesh (size = entity.bbox_dims), single colour from
#     palette[entity.label]
#   - MC-item composite: if entity.custom_meta.mc_item is set, we look that
#     id up in mc_item_pack/_compiled.json and emit one MeshInstance3D per
#     sub-box (each face can have its own colour). entity.bbox_dims is
#     ignored in this mode — the preset's own boxes define the geometry.

extends Node3D

const ENTITY_META_KEY := "vxw_entity"
const ITEM_PACK_COMPILED := "res://../m3_adapter/mc_item_pack/_compiled.json"

var _spawned: Array[Node3D] = []
var _item_presets: Dictionary = {}    # id -> preset dict (boxes, etc.)
var _logger = null
# Cached from the last load so reload() can refresh from disk without the
# caller having to re-supply the path / palette.
var _last_path: String = ""
var _last_palette: PackedColorArray = PackedColorArray()


func init_renderer(logger = null) -> void:
    _logger = logger
    _load_item_presets()


func _load_item_presets() -> void:
    # The pack lives outside res:// (in the python adapter tree). Resolve via
    # the project root's parent (godot_viewer/.. == project root).
    var abs_path := ProjectSettings.globalize_path("res://").path_join(
        "../m3_adapter/mc_item_pack/_compiled.json"
    )
    if not FileAccess.file_exists(abs_path):
        if _logger != null:
            _logger.info("item_presets", {"present": false, "path": abs_path})
        return
    var txt := FileAccess.get_file_as_string(abs_path)
    if txt.is_empty():
        return
    var data = JSON.parse_string(txt)
    if data == null or not data.has("presets"):
        return
    for p in data.presets:
        _item_presets[String(p.get("id", ""))] = p
    if _logger != null:
        _logger.info("item_presets_loaded",
                     {"count": _item_presets.size(), "path": abs_path})


func get_item_presets() -> Dictionary:
    return _item_presets


func clear() -> void:
    for n in _spawned:
        if is_instance_valid(n):
            # remove_child first: queue_free alone leaves the node in
            # get_children() until end of frame, so a reload-then-relookup (e.g.
            # the selector re-binding its selection) would match the STALE old
            # node instead of the freshly spawned one → outline desync.
            remove_child(n)
            n.queue_free()
    _spawned.clear()


# Load entities.json from `world_path` and spawn nodes as children of this node.
# `palette_rgb` is the world's palette (PackedColorArray, index = material_id).
# Returns the number of entities spawned.
func reload() -> int:
    # Re-read entities.json from disk using the last path + palette. Used after
    # an in-place edit (rotate / delete / move) to refresh the visuals.
    if _last_path == "":
        return 0
    return load_entities(_last_path, _last_palette)


func load_entities(world_path: String, palette_rgb: PackedColorArray) -> int:
    _last_path = world_path
    _last_palette = palette_rgb
    clear()
    var ent_path := world_path + "/entities.json"
    if not FileAccess.file_exists(ent_path):
        if _logger != null:
            _logger.info("entities", {"present": false, "path": ent_path})
        return 0
    var txt := FileAccess.get_file_as_string(ent_path)
    if txt.is_empty():
        return 0
    var data = JSON.parse_string(txt)
    if data == null or not data.has("entities"):
        if _logger != null:
            _logger.error("entities", {"reason": "parse failed", "path": ent_path})
        return 0
    var count_by_label := {}
    for e in data.entities:
        var node := _spawn_one(e, palette_rgb)
        if node != null:
            _spawned.append(node)
            add_child(node)
            var l: int = int(e.label)
            count_by_label[l] = int(count_by_label.get(l, 0)) + 1
    if _logger != null:
        _logger.info("entities_loaded", {
            "count": _spawned.size(),
            "by_label": count_by_label,
            "path": ent_path,
        })
    return _spawned.size()


# Build one Node3D with either a single BoxMesh (fallback) or a multi-box
# composite (when entity.custom_meta.mc_item references a known preset).
func _spawn_one(e: Dictionary, palette_rgb: PackedColorArray) -> Node3D:
    var pos = e.get("position")
    var rot = e.get("rotation")
    var dims = e.get("bbox_dims")
    if pos == null or rot == null:
        return null
    if pos.size() < 3 or rot.size() < 4:
        return null

    var label: int = int(e.get("label", 0))
    var label_name: String = String(e.get("label_name", "?"))
    var custom_meta: Dictionary = e.get("custom_meta", {})
    var mc_item_id: String = String(custom_meta.get("mc_item", ""))
    var use_mc := mc_item_id != "" and _item_presets.has(mc_item_id)

    var quat := Quaternion(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
    if use_mc:
        quat = _yaw_only_quat(quat)
    var root := Node3D.new()
    root.name = "entity_%s_%s" % [label_name, str(e.get("id", "")).substr(0, 8)]
    root.transform = Transform3D(
        quat,
        Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
    )

    var state: String = String(custom_meta.get("state", "off"))
    var behaviors: Array = []
    if use_mc:
        var preset: Dictionary = _item_presets[mc_item_id]
        behaviors = preset.get("behaviors", []) if preset.has("behaviors") else []
        _build_mc_composite(root, preset, state)
    else:
        var col: Color = Color(0.5, 0.5, 0.5)
        if label >= 0 and label < palette_rgb.size():
            col = palette_rgb[label]
        var box_dims = dims if (dims != null and dims.size() >= 3) else [0.5, 0.5, 0.5]
        _build_single_box(root, col, box_dims)

    # Picking + physics proxy. Default freeze=true so the entity stays put
    # (selection-time / grab-time semantics). custom_meta.physics_dynamic
    # toggles freeze off so gravity + collisions take over.
    var pick_dims = dims if (dims != null and dims.size() >= 3) else [0.5, 0.5, 0.5]
    if use_mc:
        var ext = _item_presets[mc_item_id].get("overall_extents_m", null)
        if ext != null and ext.size() >= 3:
            pick_dims = ext
    var dynamic := bool(custom_meta.get("physics_dynamic", false))
    root.add_child(_make_physics_body(pick_dims, dynamic))

    root.set_meta(ENTITY_META_KEY, {
        "id": String(e.get("id", "")),
        "label": label,
        "label_name": label_name,
        "voxel_count": int(e.get("voxel_count", 0)),
        "custom_meta": custom_meta,
        "behaviors": behaviors,
    })
    return root


# Reduce a full 3D rotation to a yaw-only rotation about world +Y, so upright-
# authored mc_item models stay upright (their full PCA orientation would tip
# them over and, when physics_dynamic, topple them under gravity). Robust to
# the degenerate case where the dominant basis axis is near-vertical: pick the
# basis axis with the largest horizontal (XZ) projection as the facing
# reference. World convention is Y-up, -Z forward.
func _yaw_only_quat(q: Quaternion) -> Quaternion:
    var b := Basis(q)
    var best := Vector3.ZERO
    var best_len := -1.0
    for axis in [b.x, b.y, b.z]:
        var horiz := Vector3(axis.x, 0.0, axis.z)
        var l := horiz.length()
        if l > best_len:
            best_len = l
            best = horiz
    if best_len < 1e-4:
        return Quaternion.IDENTITY   # fully vertical basis -> no meaningful yaw
    best = best.normalized()
    var yaw := atan2(best.x, best.z)
    return Quaternion(Vector3.UP, yaw)


func _make_physics_body(dims, dynamic: bool) -> RigidBody3D:
    var body := RigidBody3D.new()
    body.name = "PickProxy"
    body.freeze = not dynamic
    # When dynamic = false, freeze mode "static" (the default) means the body
    # never moves and acts identically to a StaticBody for raycasts. When
    # dynamic = true, freeze=false, and gravity + collisions take over.
    body.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
    body.mass = max(0.5, float(dims[0]) * float(dims[1]) * float(dims[2]) * 100.0)
    # Bit 1 = world layer (player/physics collides with furniture).
    # Bit 3 = entity-pick layer (selector ray queries ONLY this layer,
    #         so the ray ignores solid voxel geometry and the CharacterBody3D rig).
    body.collision_layer = 1 | 4   # bits: world + entity-pick
    body.collision_mask  = 1       # collide with world voxels (for physics_dynamic)
    body.contact_monitor = false   # we don't need callbacks yet
    var cs := CollisionShape3D.new()
    var shape := BoxShape3D.new()
    shape.size = Vector3(float(dims[0]), float(dims[1]), float(dims[2]))
    cs.shape = shape
    body.add_child(cs)
    return body


func _build_single_box(root: Node3D, col: Color, dims) -> void:
    var box := BoxMesh.new()
    box.size = Vector3(float(dims[0]), float(dims[1]), float(dims[2]))
    var mat := StandardMaterial3D.new()
    mat.albedo_color = col
    mat.metallic = 0.05
    mat.roughness = 0.65
    var mi := MeshInstance3D.new()
    mi.mesh = box
    mi.material_override = mat
    root.add_child(mi)


# Phase-2 multi-box composite: each MC sub-box becomes ONE solid BoxMesh,
# textured with the dominant_texture (triplanar) or the average face colour.
# This replaces the previous 6-PlaneMesh approach that produced see-through
# decals; a BoxMesh is fully solid and visible from all angles.
func _build_mc_composite(root: Node3D, preset: Dictionary, state: String = "off") -> void:
    var behaviors: Array = preset.get("behaviors", []) if preset.has("behaviors") else []
    var glow_on := state == "on" and behaviors.has("switchable")
    for b in preset.get("boxes", []):
        var bmin = b.get("min")
        var bmax = b.get("max")
        if bmin == null or bmax == null or bmin.size() < 3 or bmax.size() < 3:
            continue
        var dims := Vector3(
            float(bmax[0]) - float(bmin[0]),
            float(bmax[1]) - float(bmin[1]),
            float(bmax[2]) - float(bmin[2]),
        )
        var center := Vector3(
            (float(bmax[0]) + float(bmin[0])) * 0.5,
            (float(bmax[1]) + float(bmin[1])) * 0.5,
            (float(bmax[2]) + float(bmin[2])) * 0.5,
        )
        var box := BoxMesh.new()
        box.size = dims
        var mat := StandardMaterial3D.new()
        mat.roughness = 1.0
        mat.metallic = 0.0
        # Use dominant texture (triplanar) if available, else average face colour.
        var tex_path := String(b.get("dominant_texture", ""))
        var tex: Texture2D = _load_pack_texture(tex_path) if tex_path != "" else null
        if tex != null:
            mat.albedo_texture = tex
            mat.albedo_color = Color.WHITE
            mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
            mat.uv1_triplanar = true
            mat.uv1_scale = Vector3(5, 5, 5)
        else:
            var fc: Dictionary = b.get("face_colors") if b.has("face_colors") else {}
            mat.albedo_color = _avg_color(fc)
        if glow_on:
            mat.emission_enabled = true
            mat.emission = mat.albedo_color * 1.2
            mat.emission_energy_multiplier = 2.5
        var mi := MeshInstance3D.new()
        mi.mesh = box
        mi.material_override = mat
        mi.position = center
        root.add_child(mi)


func _avg_color(fc: Dictionary) -> Color:
    var r := 0.0; var g := 0.0; var b := 0.0; var n := 0
    for f in fc.values():
        if f is Array and f.size() >= 3:
            r += float(f[0]); g += float(f[1]); b += float(f[2]); n += 1
    if n == 0:
        return Color(0.6, 0.6, 0.6)
    return Color(r / n / 255.0, g / n / 255.0, b / n / 255.0)


var _tex_cache: Dictionary = {}    # rel path → Texture2D

# Texture PNGs live next to the compiled pack JSON, under
# m3_adapter/mc_item_pack/textures/... — same root as _compiled.json.
func _load_pack_texture(rel_path: String) -> Texture2D:
    if rel_path == "":
        return null
    if _tex_cache.has(rel_path):
        return _tex_cache[rel_path]
    var abs_path := ProjectSettings.globalize_path("res://").path_join(
        "../m3_adapter/mc_item_pack"
    ).path_join(rel_path)
    if not FileAccess.file_exists(abs_path):
        _tex_cache[rel_path] = null
        return null
    var img := Image.load_from_file(abs_path)
    if img == null:
        _tex_cache[rel_path] = null
        return null
    var tex := ImageTexture.create_from_image(img)
    _tex_cache[rel_path] = tex
    return tex


func entity_count() -> int:
    return _spawned.size()
