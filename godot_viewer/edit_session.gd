# edit_session.gd — voxel edit persistence + the unified undo stack.
# Voxel entries ("destroy"/"place") are handled here; "entity_*" entries
# route to entity_edit_controller (injected via set_entity_edit).
extends Node

const VxwWriter = preload("res://vxw_writer.gd")

# Undo stack: each entry = {op: "destroy"|"place", vi: Vector3i, material_id: int}
# - "destroy" entry: the voxel was destroyed; undo re-places it with material_id
# - "place" entry: a new voxel was placed; undo destroys it
const _UNDO_CAP: int = 50
var _undo_stack: Array = []

var _ws  # world_session
var _renderer: Node3D
var _voxel_editor: Node3D
var _pause_menu: CanvasLayer
var _material_picker: CanvasLayer
var _cam_ctl: Node
var _edit_warning: Label
var _entity_edit = null
var _logger


func init_session(ws, renderer: Node3D, voxel_editor: Node3D,
        pause_menu: CanvasLayer, material_picker: CanvasLayer,
        cam_ctl: Node, edit_warning: Label, logger) -> void:
    _ws = ws
    _renderer = renderer
    _voxel_editor = voxel_editor
    _pause_menu = pause_menu
    _material_picker = material_picker
    _cam_ctl = cam_ctl
    _edit_warning = edit_warning
    _logger = logger
    _voxel_editor.voxel_destroyed.connect(_on_voxel_destroyed)
    _voxel_editor.voxel_placed.connect(_on_voxel_placed)
    _material_picker.material_selected.connect(_on_material_selected)


func set_entity_edit(entity_edit) -> void:
    _entity_edit = entity_edit


func toggle_edit_mode() -> void:
    var new_state: bool = not bool(_voxel_editor.is_edit_enabled())
    _voxel_editor.set_edit_enabled(new_state)
    _cam_ctl.set_orbit_enabled(not new_state)
    _pause_menu.set_edit_mode_label(new_state)
    _edit_warning.visible = new_state
    _logger.info("edit_mode", {"enabled": new_state})


func push_undo(entry: Dictionary) -> void:
    _undo_stack.append(entry)
    if _undo_stack.size() > _UNDO_CAP:
        _undo_stack.pop_front()
    _pause_menu.set_undo_count(_undo_stack.size())


func undo_last_edit() -> bool:
    if _undo_stack.is_empty():
        _logger.info("undo", {"status": "stack empty"})
        return false
    var entry: Dictionary = _undo_stack.pop_back()
    var op: String = entry["op"]
    if op == "destroy":
        var vi: Vector3i = entry["vi"]
        var mid: int = int(entry["material_id"])
        if _renderer.add_voxel(vi, mid):
            _voxel_editor._occupied[vi] = true
            patch_voxel_on_disk(vi, mid, 0)
    elif op == "place":
        var vi2: Vector3i = entry["vi"]
        if _renderer.hide_voxel(vi2):
            _voxel_editor._occupied.erase(vi2)
            patch_voxel_on_disk(vi2, 0, 0)
    elif op.begins_with("entity_"):
        _entity_edit.apply_undo(entry)
    _pause_menu.set_undo_count(_undo_stack.size())
    _logger.info("undo", {"op": op, "stack_left": _undo_stack.size()})
    return true


func open_material_picker() -> void:
    if _ws.world == null:
        return
    _material_picker.set_current(_voxel_editor.get_current_material())
    _material_picker.open(_ws.world)


func _on_material_selected(mid: int) -> void:
    _voxel_editor.set_current_material(mid)
    _logger.info("material_selected", {"material_id": mid})


func _on_voxel_destroyed(world_voxel_index: Vector3i, _world_position_m: Vector3) -> void:
    # We don't know the material_id of the destroyed voxel from the signal,
    # so for undo we restore as material=1 (stone) — pragmatic fallback.
    # When voxel_editor tracks original material on hover we can pass it through.
    push_undo({"op": "destroy", "vi": world_voxel_index, "material_id": 1})
    patch_voxel_on_disk(world_voxel_index, 0, 0)
    _logger.info("voxel_destroyed", {
        "world_voxel": [world_voxel_index.x, world_voxel_index.y, world_voxel_index.z],
    })


func _on_voxel_placed(world_voxel_index: Vector3i, _world_position_m: Vector3, material_id: int) -> void:
    push_undo({"op": "place", "vi": world_voxel_index, "material_id": material_id})
    patch_voxel_on_disk(world_voxel_index, material_id, 0)
    _logger.info("voxel_placed", {
        "world_voxel": [world_voxel_index.x, world_voxel_index.y, world_voxel_index.z],
        "material_id": material_id,
    })


# Map a world voxel index to its (chunk_coord, local_voxel) and patch the chunk
# file with a single voxel cell. material_id=0 means clear to air.
func patch_voxel_on_disk(world_voxel_index: Vector3i, material_id: int, semantic_id: int) -> void:
    var ce: int = _ws.world.chunk_extent
    var fdiv := Vector3(world_voxel_index) / float(ce)
    var chunk_coord := Vector3i(int(floor(fdiv.x)), int(floor(fdiv.y)), int(floor(fdiv.z)))
    var local := world_voxel_index - chunk_coord * ce
    var chunk_path: String = _ws.world_path + "/chunks/%d_%d_%d.chunk" % [
        chunk_coord.x, chunk_coord.y, chunk_coord.z,
    ]
    var cell := PackedByteArray([material_id, semantic_id, 0, 0])
    VxwWriter.patch_voxel(
        chunk_path,
        chunk_coord,
        local,
        cell,
        ce,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.GZIP,
    )
