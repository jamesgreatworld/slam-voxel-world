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
        main_cam, ws.world_path, ent_renderer,
        placer, voxel_editor, logger
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
    context_bar.duplicate_pressed.connect(func(): edit.duplicate_selected())
    context_bar.physics_toggle_pressed.connect(func(): selector.toggle_physics_on_selected())
    context_bar.rotate_pressed.connect(_on_context_rotate)
    context_bar.delete_pressed.connect(_on_context_delete)
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


func _run_pick_selftest() -> void:
    await get_tree().physics_frame
    await get_tree().physics_frame
    selector.selftest_pick_all()
    get_tree().quit()


func _on_world_loaded(world, path: String) -> void:
    ent_renderer.load_entities(path, world.palette_rgb)
    if selector != null:
        selector.set_world_path(path)
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


func _on_context_rotate(yaw_delta_rad: float) -> void:
    var sid: String = selector.get_selected_id()
    if sid != "":
        selector.rotate_by_id(sid, yaw_delta_rad)


func _on_context_delete() -> void:
    var sid: String = selector.get_selected_id()
    if sid != "":
        selector.delete_by_id(sid)
