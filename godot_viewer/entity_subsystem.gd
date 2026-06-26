# entity_subsystem.gd — assembles the entity-layer nodes and their wiring:
# renderer, item picker, placer, selector, edit controller, inspector,
# top toolbar and context bar. Rebinds itself on world_session.world_loaded.
extends Node

signal menu_requested

const EntityRendererScript = preload("res://entity_renderer.gd")
const ItemPickerScript = preload("res://item_picker.gd")
const EntityPlacerScript = preload("res://entity_placer.gd")
const EntitySelectorScript = preload("res://entity_selector.gd")
const EntityEditControllerScript = preload("res://entity_edit_controller.gd")
const EntityInspectorScript = preload("res://entity_inspector.gd")
const TopToolbarScript = preload("res://top_toolbar.gd")
const EntityContextBarScript = preload("res://entity_context_bar.gd")

var ent_renderer: Node3D = null
var picker: CanvasLayer = null
var placer: Node3D = null
var selector: Node3D = null
var edit: Node = null
var inspector: CanvasLayer = null
var toolbar: CanvasLayer = null
var context_bar: CanvasLayer = null

var _ws  # world_session
var _logger


func build(main_cam: Camera3D, voxel_editor: Node3D, stereo_rig: Node3D,
        ws, edit_session, logger) -> void:
    _ws = ws
    _logger = logger

    ent_renderer = Node3D.new()
    ent_renderer.set_script(EntityRendererScript)
    ent_renderer.name = "EntityRenderer"
    add_child(ent_renderer)
    ent_renderer.init_renderer(logger)
    ent_renderer.load_entities(ws.world_path, ws.world.palette_rgb)

    picker = CanvasLayer.new()
    picker.set_script(ItemPickerScript)
    picker.name = "ItemPicker"
    add_child(picker)
    picker.init_picker(logger)
    picker.set_presets(ent_renderer.get_item_presets())

    placer = Node3D.new()
    placer.set_script(EntityPlacerScript)
    placer.name = "EntityPlacer"
    add_child(placer)
    placer.init_placer(main_cam, voxel_editor, logger)

    selector = Node3D.new()
    selector.set_script(EntitySelectorScript)
    selector.name = "EntitySelector"
    add_child(selector)
    selector.init_selector(
        main_cam, ent_renderer, placer, voxel_editor, logger
    )

    edit = Node.new()
    edit.set_script(EntityEditControllerScript)
    edit.name = "EntityEditController"
    add_child(edit)
    edit.init_controller(
        ws.world_path, ws.world,
        ent_renderer, picker, placer, selector,
        stereo_rig, logger
    )
    edit.entity_undo_push.connect(edit_session.push_undo)
    edit_session.set_entity_edit(edit)
    selector.set_edit_controller(edit)   # selector routes mutations through the controller

    inspector = CanvasLayer.new()
    inspector.set_script(EntityInspectorScript)
    inspector.name = "EntityInspector"
    add_child(inspector)
    inspector.init_inspector(ws.world_path, logger)
    inspector.entity_committed.connect(_on_inspector_committed)

    toolbar = CanvasLayer.new()
    toolbar.set_script(TopToolbarScript)
    toolbar.name = "TopToolbar"
    add_child(toolbar)
    toolbar.items_pressed.connect(func(): picker.toggle())
    toolbar.snapshot_pressed.connect(ws.save_snapshot)
    toolbar.menu_pressed.connect(func(): menu_requested.emit())

    context_bar = CanvasLayer.new()
    context_bar.set_script(EntityContextBarScript)
    context_bar.name = "EntityContextBar"
    add_child(context_bar)
    context_bar.inspector_pressed.connect(open_inspector_for_selection)
    context_bar.duplicate_pressed.connect(func():
        if edit.duplicate_selected():
            context_bar.flash("✅ 已复制副本")
    )
    context_bar.rotate_pressed.connect(func(yaw: float):
        if edit.rotate_selected(yaw):
            context_bar.flash("✅ 已旋转 %+d°" % int(round(rad_to_deg(yaw))))
    )
    context_bar.delete_pressed.connect(func(): edit.delete_selected())
    context_bar.use_pressed.connect(func(): edit.use_selected())
    context_bar.move_pressed.connect(func(): selector.start_move())
    selector.entity_selected.connect(func(id: String):
        context_bar.on_entity_selected(id, selector.get_selected_label_name())
    )
    selector.selection_cleared.connect(func():
        context_bar.on_selection_cleared()
    )
    selector.move_state_changed.connect(func(active: bool):
        context_bar.set_move_mode(active)
    )

    ws.world_loaded.connect(_on_world_loaded)

    if "--entity-pick-selftest" in OS.get_cmdline_user_args():
        _run_pick_selftest()
    if "--entity-edit-selftest" in OS.get_cmdline_user_args():
        _run_edit_selftest()


func _run_pick_selftest() -> void:
    await get_tree().physics_frame
    await get_tree().physics_frame
    selector.selftest_pick_all()
    get_tree().quit()


# Exercises the controller mutation path (select → rotate → move → delete) and
# asserts entities.json changed each time. Headless regression net for the
# selector→controller refactor.
func _run_edit_selftest() -> void:
    await get_tree().physics_frame
    await get_tree().physics_frame
    var ids: Array = []
    for c in ent_renderer.get_children():
        if c is Node3D and c.has_meta("vxw_entity"):
            ids.append(String((c.get_meta("vxw_entity") as Dictionary).get("id", "")))
    if ids.is_empty():
        print("[edit-selftest] FAIL no entities"); get_tree().quit(); return
    var id: String = ids[0]
    selector.select_by_id(id)
    print("[edit-selftest] selected=%s (sel_id=%s)" % [id, selector.get_selected_id()])

    var rot0 = _edit_rec(id).get("rotation", [0, 0, 0, 1])
    var rot_ok: bool = edit.rotate_selected(deg_to_rad(30))
    await get_tree().physics_frame
    var rot1 = _edit_rec(id).get("rotation", [0, 0, 0, 1])
    var node_yaw := rad_to_deg(selector._selected_node.global_transform.basis.get_euler().y) \
        if is_instance_valid(selector._selected_node) else 0.0
    var out_yaw := rad_to_deg(selector._outline.global_transform.basis.get_euler().y) \
        if selector._outline != null else 0.0
    print("[edit-selftest] rotate ok=%s changed=%s  node_yaw=%.1f outline_yaw=%.1f match=%s" % [
        str(rot_ok), str(rot0 != rot1), node_yaw, out_yaw, str(abs(node_yaw - out_yaw) < 0.5)])

    var mv_ok: bool = edit.move_selected_to(Vector3(2, 1, 2))
    await get_tree().physics_frame
    var pos1 = _edit_rec(id).get("position", [0, 0, 0])
    print("[edit-selftest] move ok=%s pos=%s" % [str(mv_ok), str(pos1)])

    var n0: int = _edit_all().size()
    var del_ok: bool = edit.delete_selected()
    await get_tree().physics_frame
    var n1: int = _edit_all().size()
    print("[edit-selftest] delete ok=%s count %d->%d" % [str(del_ok), n0, n1])
    print("[edit-selftest] RESULT %s" % ("PASS" if (rot_ok and rot0 != rot1 and mv_ok and del_ok and n1 == n0 - 1) else "FAIL"))
    get_tree().quit()


func _edit_all() -> Array:
    var p: String = _ws.world_path + "/entities.json"
    if not FileAccess.file_exists(p):
        return []
    var d = JSON.parse_string(FileAccess.get_file_as_string(p))
    return d.entities if (d != null and d.has("entities")) else []


func _edit_rec(id: String) -> Dictionary:
    for e in _edit_all():
        if String(e.get("id", "")) == id:
            return e
    return {}


func _on_world_loaded(world, path: String) -> void:
    ent_renderer.load_entities(path, world.palette_rgb)
    if selector != null:
        selector.on_world_changed()
    if edit != null:
        edit.set_world(world, path)
    if inspector != null:
        inspector.set_world_path(path)


func open_inspector_for_selection() -> void:
    if inspector == null or selector == null:
        return
    var sel_id: String = String(selector._selected_id)
    if sel_id == "":
        if _logger != null:
            _logger.info("entity_inspector_open", {"status": "no selection"})
        return
    var path: String = _ws.world_path + "/entities.json"
    if not FileAccess.file_exists(path):
        return
    var txt := FileAccess.get_file_as_string(path)
    if txt.is_empty():
        return
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return
    for e in d.entities:
        if String(e.get("id", "")) == sel_id:
            inspector.set_world_path(_ws.world_path)
            inspector.open_for(e)
            return


func _on_inspector_committed(_updated: Dictionary) -> void:
    if ent_renderer != null and _ws.world != null:
        ent_renderer.load_entities(_ws.world_path, _ws.world.palette_rgb)


