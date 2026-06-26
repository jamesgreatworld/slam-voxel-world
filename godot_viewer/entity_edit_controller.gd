# entity_edit_controller.gd — owns the entity-layer mutation pipeline.
#
# Lives next to main.gd in the boot tree. Wraps:
#   - spawning (from item picker / placement / CLI fallback)
#   - entities.json read/write (single point that touches the file)
#   - undo entry creation for each mutation, surfaced back to main via
#     `entity_undo_push` so the global LIFO stack stays mixed with voxel
#     undo entries (Ctrl+Z reverses the most recent action regardless of
#     which subsystem produced it)
#
# Doesn't touch voxel rendering, camera, HUD, or the pause menu. Re-uses
# entity_renderer / item_picker / entity_placer / entity_selector that main
# already created.

extends Node

signal entity_undo_push(entry: Dictionary)

var _world_path: String = ""
var _world = null
var _entity_renderer = null
var _item_picker = null
var _entity_placer = null
var _entity_selector = null
var _stereo_rig: Node3D = null
var _logger = null


func init_controller(
    world_path: String,
    world,
    entity_renderer,
    item_picker,
    entity_placer,
    entity_selector,
    stereo_rig: Node3D,
    logger = null,
) -> void:
    _world_path = world_path
    _world = world
    _entity_renderer = entity_renderer
    _item_picker = item_picker
    _entity_placer = entity_placer
    _entity_selector = entity_selector
    _stereo_rig = stereo_rig
    _logger = logger
    # Hook entity picker / placer signals. (Selection mutations now call this
    # controller directly — see rotate_selected / move_selected_to / delete_selected.)
    item_picker.item_chosen.connect(_on_item_chosen)
    entity_placer.placement_committed.connect(_on_placement_committed)


func set_world(world, world_path: String) -> void:
    _world = world
    _world_path = world_path


# ---------------------------------------------------------------------------
# Selection mutations — the single entry point for editing the selected entity.
# The selector / context bar call these; each reads entities.json, mutates the
# selected record, writes, pushes an undo entry, then reloads + re-syncs the
# selection so the screen (and the outline) update at once.
# ---------------------------------------------------------------------------

func _sel_id() -> String:
    return _entity_selector.get_selected_id() if _entity_selector != null else ""


func rotate_selected(yaw_delta_rad: float) -> bool:
    var sid := _sel_id()
    if sid == "":
        return false
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    for e in entities:
        if String(e.get("id", "")) != sid:
            continue
        var rot = e.get("rotation", [0, 0, 0, 1])
        var cur := Quaternion(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
        var new_q := Quaternion(Vector3.UP, yaw_delta_rad) * cur
        var to_q := [new_q.x, new_q.y, new_q.z, new_q.w]
        emit_signal("entity_undo_push", {"op": "entity_rotate", "id": sid,
            "from": [float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])], "to": to_q})
        e["rotation"] = to_q
        _write_entities_json(ent_path, entities)
        if _logger != null:
            _logger.info("entity_rotated", {"id": sid, "yaw_deg": rad_to_deg(yaw_delta_rad)})
        _reload_and_resync()
        return true
    return false


func move_selected_to(world_pos: Vector3) -> bool:
    var sid := _sel_id()
    if sid == "":
        return false
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    for e in entities:
        if String(e.get("id", "")) != sid:
            continue
        var old = e.get("position", [0, 0, 0])
        var newp := [world_pos.x, world_pos.y, world_pos.z]
        emit_signal("entity_undo_push", {"op": "entity_move", "id": sid,
            "from": [float(old[0]), float(old[1]), float(old[2])], "to": newp})
        e["position"] = newp
        _write_entities_json(ent_path, entities)
        if _logger != null:
            _logger.info("entity_moved", {"id": sid, "pos": newp})
        _reload_and_resync()
        return true
    return false


func delete_selected() -> bool:
    var sid := _sel_id()
    if sid == "":
        return false
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    var kept: Array = []
    var removed: Dictionary = {}
    for e in entities:
        if String(e.get("id", "")) == sid:
            removed = e
        else:
            kept.append(e)
    if removed.is_empty():
        return false
    emit_signal("entity_undo_push", {"op": "entity_delete", "entity": removed.duplicate(true)})
    _write_entities_json(ent_path, kept)
    if _logger != null:
        _logger.info("entity_deleted", {"id": sid, "remaining": kept.size()})
    if _entity_selector != null:
        _entity_selector.clear_selection()
    _reload_and_resync()
    return true


# Single reload path: refresh visuals from disk, then let the selector re-bind
# its node + outline to the (newly re-spawned) selected entity.
func _reload_and_resync() -> void:
    if _entity_renderer != null:
        _entity_renderer.reload()
    if _entity_selector != null and _entity_selector.has_method("refresh_selection"):
        _entity_selector.refresh_selection()


# ---------------------------------------------------------------------------
# Spawn flow
# ---------------------------------------------------------------------------

func _on_item_chosen(item_id: String) -> void:
    var presets: Dictionary = _entity_renderer.get_item_presets()
    if not presets.has(item_id):
        push_error("[edit] item_chosen for unknown preset: " + item_id)
        return
    # Hand off to the placer (interactive ghost). Fallback to instant spawn
    # if for some reason the placer isn't ready.
    if _entity_placer != null:
        _entity_placer.start(item_id, presets[item_id])
        return
    spawn_in_front_of_rig(item_id)


func duplicate_selected() -> bool:
    # Deep-copy the currently-selected entity, offset its position by
    # (0, +0.5, +0.5), give it a fresh uuid, append to entities.json, reload
    # the renderer and push an entity_spawn undo entry so Ctrl+Z removes it.
    if _entity_selector == null:
        return false
    var sel_id: String = String(_entity_selector._selected_id)
    if sel_id == "":
        if _logger != null:
            _logger.info("entity_duplicate", {"status": "no selection"})
        return false
    var ent_path := _world_path + "/entities.json"
    var existing := _read_entities_json(ent_path)
    var src: Dictionary = {}
    for e in existing:
        if String(e.get("id", "")) == sel_id:
            src = e
            break
    if src.is_empty():
        if _logger != null:
            _logger.info("entity_duplicate", {"status": "id not in entities.json", "id": sel_id})
        return false
    var copy: Dictionary = src.duplicate(true)
    var new_id := _uuid4()
    copy["id"] = new_id
    var pos = copy.get("position", [0, 0, 0])
    copy["position"] = [
        float(pos[0]) + 0.0,
        float(pos[1]) + 0.5,
        float(pos[2]) + 0.5,
    ]
    existing.append(copy)
    _write_entities_json(ent_path, existing)
    emit_signal("entity_undo_push", {"op": "entity_spawn", "id": new_id})
    if _logger != null:
        _logger.info("entity_duplicated", {
            "src_id": sel_id, "new_id": new_id,
            "pos": copy["position"], "total": existing.size(),
        })
    _reload_and_resync()
    return true


func _on_placement_committed(item_id: String, world_pos: Vector3, yaw_rad: float) -> void:
    var presets: Dictionary = _entity_renderer.get_item_presets()
    if not presets.has(item_id):
        return
    spawn_entity(item_id, presets[item_id], world_pos, yaw_rad)


func spawn_in_front_of_rig(item_id: String) -> void:
    var presets: Dictionary = _entity_renderer.get_item_presets()
    if not presets.has(item_id):
        return
    var preset: Dictionary = presets[item_id]
    var rig_xf: Transform3D = _stereo_rig.global_transform
    var forward: Vector3 = -rig_xf.basis.z.normalized()
    var pos: Vector3 = rig_xf.origin + forward * 1.5
    var yaw: float = rig_xf.basis.get_euler().y
    spawn_entity(item_id, preset, pos, yaw)


func spawn_entity(item_id: String, preset: Dictionary,
                  pos: Vector3, yaw_rad: float) -> void:
    var extents = preset.get("overall_extents_m", [0.5, 0.5, 0.5])
    var label: int = int(preset.get("default_label", 0))
    var entity_id := _uuid4()
    var entity: Dictionary = {
        "id": entity_id,
        "label": label,
        "label_name": item_id,
        "position": [pos.x, pos.y, pos.z],
        "rotation": _quat_from_yaw(yaw_rad),
        "bbox_dims": [float(extents[0]), float(extents[1]), float(extents[2])],
        "voxel_count": 0,
        "custom_meta": {"mc_item": item_id, "spawned_at": Time.get_unix_time_from_system()},
    }
    var ent_path := _world_path + "/entities.json"
    var existing := _read_entities_json(ent_path)
    existing.append(entity)
    _write_entities_json(ent_path, existing)
    emit_signal("entity_undo_push", {"op": "entity_spawn", "id": entity_id})
    if _logger != null:
        _logger.info("entity_spawned", {"id": entity_id, "item": item_id,
                                        "pos": [pos.x, pos.y, pos.z],
                                        "yaw_deg": rad_to_deg(yaw_rad),
                                        "total_entities": existing.size()})
    _reload_and_resync()


# ---------------------------------------------------------------------------
# Behavior toggle (entity custom_meta.state mutation, no undo entry).
# ---------------------------------------------------------------------------

# Look up the selected entity's mc_item preset and dispatch the first
# declared behavior. lamp/computer → switchable; chair → sittable.
# Falls back to a no-op (returns false) if the entity has no behaviors.
func use_selected() -> bool:
    if _entity_selector == null or _entity_renderer == null:
        return false
    var sel_id: String = _entity_selector.get_selected_id()
    if sel_id == "":
        return false
    var entities := _read_entities_json(_world_path + "/entities.json")
    var mc_item: String = ""
    for e in entities:
        if String(e.get("id", "")) == sel_id:
            var cm: Dictionary = e.get("custom_meta", {})
            mc_item = String(cm.get("mc_item", ""))
            break
    if mc_item == "":
        return apply_behavior("switchable")
    var presets: Dictionary = _entity_renderer.get_item_presets()
    if not presets.has(mc_item):
        return apply_behavior("switchable")
    var preset: Dictionary = presets[mc_item]
    var beh = preset.get("behaviors", [])
    if beh is Array and beh.size() > 0:
        return apply_behavior(String(beh[0]))
    return false


# Apply a behavior (e.g. "switchable") to the currently selected entity.
# Mutates entities.json custom_meta.state and re-renders. Returns true if
# something changed. No undo entry: behavior toggles can be high-frequency,
# and the visual state is recoverable by toggling again.
func apply_behavior(behavior: String) -> bool:
    if _entity_selector == null:
        return false
    var sel_id: String = _entity_selector.get_selected_id()
    if sel_id == "":
        return false
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    var changed := false
    var new_state := ""
    for e in entities:
        if String(e.get("id", "")) != sel_id:
            continue
        var cm: Dictionary = e.get("custom_meta", {})
        if behavior == "switchable":
            var cur := String(cm.get("state", "off"))
            new_state = "on" if cur == "off" else "off"
            cm["state"] = new_state
        elif behavior == "sittable":
            # Toggle sit on/off using the same selected chair entity.
            if _stereo_rig != null and _stereo_rig.has_method("is_seated"):
                if _stereo_rig.is_seated() and \
                   _stereo_rig.get_seated_entity_id() == sel_id:
                    _stereo_rig.exit_seat()
                    new_state = "standing"
                else:
                    var pos_arr = e.get("position", [0, 0, 0])
                    var rot_arr = e.get("rotation", [0, 0, 0, 1])
                    var ent_pos := Vector3(float(pos_arr[0]), float(pos_arr[1]), float(pos_arr[2]))
                    var ent_quat := Quaternion(float(rot_arr[0]), float(rot_arr[1]),
                                                float(rot_arr[2]), float(rot_arr[3]))
                    var yaw := ent_quat.get_euler().y
                    _stereo_rig.enter_sit(sel_id, ent_pos, yaw)
                    new_state = "sitting"
            else:
                return false
        else:
            return false
        e["custom_meta"] = cm
        changed = true
        break
    if changed:
        _write_entities_json(ent_path, entities)
        if _logger != null:
            _logger.info("entity_behavior_applied",
                         {"id": sel_id, "behavior": behavior, "state": new_state})
        _reload_and_resync()
    return changed


# ---------------------------------------------------------------------------
# Undo dispatch (called by main when entry.op starts with "entity_")
# ---------------------------------------------------------------------------

func apply_undo(entry: Dictionary) -> void:
    var op: String = String(entry.get("op", ""))
    if op == "entity_spawn":
        _undo_spawn(entry["id"])
    elif op == "entity_delete":
        _undo_delete(entry["entity"])
    elif op == "entity_move":
        _undo_move(entry["id"], entry["from"])
    elif op == "entity_rotate":
        _undo_rotate(entry["id"], entry["from"])


func _undo_spawn(id: String) -> void:
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    var kept: Array = []
    for e in entities:
        if String(e.get("id", "")) != id:
            kept.append(e)
    _write_entities_json(ent_path, kept)
    _reload_and_resync()


func _undo_delete(entity_dict: Dictionary) -> void:
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    entities.append(entity_dict)
    _write_entities_json(ent_path, entities)
    _reload_and_resync()


func _undo_move(id: String, from_pos: Array) -> void:
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    for e in entities:
        if String(e.get("id", "")) == id:
            e["position"] = from_pos
            break
    _write_entities_json(ent_path, entities)
    _reload_and_resync()


func _undo_rotate(id: String, from_quat: Array) -> void:
    var ent_path := _world_path + "/entities.json"
    var entities := _read_entities_json(ent_path)
    for e in entities:
        if String(e.get("id", "")) == id:
            e["rotation"] = from_quat
            break
    _write_entities_json(ent_path, entities)
    _reload_and_resync()


# ---------------------------------------------------------------------------
# Helpers (kept on the controller so main.gd has no entity-IO surface)
# ---------------------------------------------------------------------------

func _uuid4() -> String:
    var rng := RandomNumberGenerator.new()
    rng.randomize()
    var b := PackedByteArray()
    for i in 16:
        b.append(rng.randi() & 0xff)
    b[6] = (b[6] & 0x0f) | 0x40
    b[8] = (b[8] & 0x3f) | 0x80
    var hex := b.hex_encode()
    return "%s-%s-%s-%s-%s" % [
        hex.substr(0, 8), hex.substr(8, 4), hex.substr(12, 4),
        hex.substr(16, 4), hex.substr(20, 12),
    ]


func _quat_from_yaw(yaw_rad: float) -> Array:
    var half := yaw_rad * 0.5
    return [0.0, sin(half), 0.0, cos(half)]


func _read_entities_json(path: String) -> Array:
    if not FileAccess.file_exists(path):
        return []
    var txt := FileAccess.get_file_as_string(path)
    if txt.is_empty():
        return []
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return []
    return d.entities


func _write_entities_json(path: String, entities: Array) -> void:
    var payload := {"format_version": "1.0", "entities": entities}
    var f := FileAccess.open(path, FileAccess.WRITE)
    if f == null:
        push_error("[edit] cannot write entities.json: " + path)
        return
    f.store_string(JSON.stringify(payload, "  "))
    f.close()
