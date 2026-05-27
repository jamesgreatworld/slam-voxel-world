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
            n.queue_free()
    _spawned.clear()


# Load entities.json from `world_path` and spawn nodes as children of this node.
# `palette_rgb` is the world's palette (PackedColorArray, index = material_id).
# Returns the number of entities spawned.
func load_entities(world_path: String, palette_rgb: PackedColorArray) -> int:
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

    var root := Node3D.new()
    root.name = "entity_%s_%s" % [label_name, str(e.get("id", "")).substr(0, 8)]
    root.transform = Transform3D(
        Quaternion(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])),
        Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
    )

    if mc_item_id != "" and _item_presets.has(mc_item_id):
        _build_mc_composite(root, _item_presets[mc_item_id])
    else:
        var col: Color = Color(0.5, 0.5, 0.5)
        if label >= 0 and label < palette_rgb.size():
            col = palette_rgb[label]
        var box_dims = dims if (dims != null and dims.size() >= 3) else [0.5, 0.5, 0.5]
        _build_single_box(root, col, box_dims)

    # Picking proxy: a single AABB-sized StaticBody so PhysicsServer rays from
    # the cursor can hit this entity even though its visuals are composite.
    var pick_dims = dims if (dims != null and dims.size() >= 3) else [0.5, 0.5, 0.5]
    root.add_child(_make_pick_body(pick_dims))

    root.set_meta(ENTITY_META_KEY, {
        "id": String(e.get("id", "")),
        "label": label,
        "label_name": label_name,
        "voxel_count": int(e.get("voxel_count", 0)),
        "custom_meta": custom_meta,
    })
    return root


func _make_pick_body(dims) -> StaticBody3D:
    var body := StaticBody3D.new()
    body.name = "PickProxy"
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


# Phase-1 multi-box composite: one BoxMesh per sub-box. Each box uses its
# `dominant_texture` (most-frequently-referenced face PNG) as a single
# albedo_texture covering all 6 faces. If no PNG is on disk for that
# texture name we fall back to the averaged face colour. Phase 2 will
# split into 6 textured PlaneMesh quads when per-face textures matter.
func _build_mc_composite(root: Node3D, preset: Dictionary) -> void:
    for b in preset.get("boxes", []):
        var bmin = b.get("min")
        var bmax = b.get("max")
        if bmin == null or bmax == null or bmin.size() < 3 or bmax.size() < 3:
            continue
        var dx: float = float(bmax[0]) - float(bmin[0])
        var dy: float = float(bmax[1]) - float(bmin[1])
        var dz: float = float(bmax[2]) - float(bmin[2])
        var cx: float = (float(bmax[0]) + float(bmin[0])) * 0.5
        var cy: float = (float(bmax[1]) + float(bmin[1])) * 0.5
        var cz: float = (float(bmax[2]) + float(bmin[2])) * 0.5

        var r := 0.0; var g := 0.0; var bl := 0.0; var n := 0
        var fc: Dictionary = b.get("face_colors") if b.has("face_colors") else {}
        for f in fc.values():
            if f.size() < 3: continue
            r += float(f[0]); g += float(f[1]); bl += float(f[2]); n += 1
        var col := Color(0.6, 0.6, 0.6) if n == 0 else \
            Color(r / n / 255.0, g / n / 255.0, bl / n / 255.0)

        var box := BoxMesh.new()
        box.size = Vector3(dx, dy, dz)
        var mat := StandardMaterial3D.new()
        var dom: String = String(b.get("dominant_texture", ""))
        var tex: Texture2D = _load_pack_texture(dom) if dom != "" else null
        if tex != null:
            mat.albedo_texture = tex
            mat.albedo_color = Color.WHITE
            mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST     # crisp pixel art
        else:
            mat.albedo_color = col
        mat.metallic = 0.05
        mat.roughness = 0.7
        var mi := MeshInstance3D.new()
        mi.mesh = box
        mi.material_override = mat
        mi.transform = Transform3D(Basis(), Vector3(cx, cy, cz))
        root.add_child(mi)


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
