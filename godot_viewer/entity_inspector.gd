# entity_inspector.gd — F2-toggled property editor for the selected entity.
#
# Mirrors item_picker.gd's CanvasLayer + GDScript-only UI pattern so we don't
# touch main.tscn. main.gd instantiates this once at boot and wires:
#   - F2 to .open_for(entity_dict)  (only when entity_selector has a selection)
#   - .entity_committed(updated_dict) → entity_renderer.load_entities()
#
# Fields shown:
#   - id           (read-only)
#   - label        (LineEdit, int)
#   - label_name   (LineEdit)
#   - position     (3× LineEdit, floats — x/y/z)
#   - rotation     (4× LineEdit, floats — quat qx/qy/qz/qw)
#   - custom_meta  (key/value table; one row per key, all stringified)
#
# Save writes back to <world>/entities.json (same atomic write/JSON style as
# entity_edit_controller) and emits entity_committed. We deliberately don't
# push an undo entry — inspector edits are treated as direct-author actions,
# matching how main.gd routes the F2 reload (no Ctrl+Z entry).

extends CanvasLayer

signal entity_committed(updated: Dictionary)

var _backdrop: ColorRect = null
var _panel: PanelContainer = null
var _vbox: VBoxContainer = null
var _grid: GridContainer = null
var _meta_grid: GridContainer = null
var _title: Label = null
var _save_btn: Button = null
var _cancel_btn: Button = null

var _id_value: Label = null
var _label_edit: LineEdit = null
var _label_name_edit: LineEdit = null
var _pos_edits: Array = []       # 3× LineEdit
var _rot_edits: Array = []       # 4× LineEdit
var _meta_rows: Array = []       # [{key: LineEdit, val: LineEdit}, ...]

var _world_path: String = ""
var _current_entity: Dictionary = {}
var _logger = null


func _ready() -> void:
    process_mode = Node.PROCESS_MODE_ALWAYS
    layer = 60
    visible = false
    _build_ui()


func init_inspector(world_path: String, logger = null) -> void:
    _world_path = world_path
    _logger = logger


func set_world_path(p: String) -> void:
    _world_path = p


func is_open() -> bool:
    return visible


func close() -> void:
    visible = false


func open_for(entity_dict: Dictionary) -> void:
    _current_entity = entity_dict.duplicate(true)
    _populate_from_entity()
    visible = true


# ---------------------------------------------------------------------------
# UI construction
# ---------------------------------------------------------------------------

func _build_ui() -> void:
    _backdrop = ColorRect.new()
    _backdrop.color = Color(0, 0, 0, 0.55)
    _backdrop.anchor_right = 1.0
    _backdrop.anchor_bottom = 1.0
    _backdrop.mouse_filter = Control.MOUSE_FILTER_STOP
    add_child(_backdrop)

    _panel = PanelContainer.new()
    _panel.custom_minimum_size = Vector2(520, 460)
    _panel.set_anchors_preset(Control.PRESET_CENTER)
    _panel.set_offset(SIDE_LEFT, -260)
    _panel.set_offset(SIDE_TOP, -230)
    _panel.set_offset(SIDE_RIGHT, 260)
    _panel.set_offset(SIDE_BOTTOM, 230)
    _backdrop.add_child(_panel)

    _vbox = VBoxContainer.new()
    _vbox.add_theme_constant_override("separation", 6)
    _panel.add_child(_vbox)

    _title = Label.new()
    _title.text = "Entity Inspector (F2)"
    _title.add_theme_font_size_override("font_size", 18)
    _vbox.add_child(_title)

    var hint := Label.new()
    hint.text = "Edit fields → Save writes entities.json. ESC / Cancel closes without saving."
    hint.add_theme_font_size_override("font_size", 11)
    hint.modulate = Color(0.8, 0.8, 0.8)
    _vbox.add_child(hint)

    _grid = GridContainer.new()
    _grid.columns = 2
    _grid.add_theme_constant_override("h_separation", 8)
    _grid.add_theme_constant_override("v_separation", 4)
    _vbox.add_child(_grid)

    _add_label_row("id", _make_readonly_label())
    _add_label_row("label", _make_label_edit())
    _add_label_row("label_name", _make_label_name_edit())
    _add_label_row("position (x,y,z)", _make_vec3_row())
    _add_label_row("rotation (qx,qy,qz,qw)", _make_quat_row())

    var meta_title := Label.new()
    meta_title.text = "custom_meta"
    meta_title.add_theme_font_size_override("font_size", 14)
    _vbox.add_child(meta_title)

    var scroll := ScrollContainer.new()
    scroll.custom_minimum_size = Vector2(500, 150)
    _vbox.add_child(scroll)

    _meta_grid = GridContainer.new()
    _meta_grid.columns = 2
    _meta_grid.add_theme_constant_override("h_separation", 8)
    _meta_grid.add_theme_constant_override("v_separation", 4)
    scroll.add_child(_meta_grid)

    var btn_row := HBoxContainer.new()
    btn_row.add_theme_constant_override("separation", 8)
    _vbox.add_child(btn_row)

    _save_btn = Button.new()
    _save_btn.text = "Save"
    _save_btn.pressed.connect(_on_save_pressed)
    btn_row.add_child(_save_btn)

    _cancel_btn = Button.new()
    _cancel_btn.text = "Cancel"
    _cancel_btn.pressed.connect(close)
    btn_row.add_child(_cancel_btn)


func _add_label_row(name: String, value_node: Control) -> void:
    var l := Label.new()
    l.text = name
    l.custom_minimum_size = Vector2(160, 0)
    _grid.add_child(l)
    _grid.add_child(value_node)


func _make_readonly_label() -> Control:
    _id_value = Label.new()
    _id_value.text = ""
    _id_value.add_theme_color_override("font_color", Color(0.85, 0.85, 0.85))
    return _id_value


func _make_label_edit() -> Control:
    _label_edit = LineEdit.new()
    _label_edit.custom_minimum_size = Vector2(280, 0)
    return _label_edit


func _make_label_name_edit() -> Control:
    _label_name_edit = LineEdit.new()
    _label_name_edit.custom_minimum_size = Vector2(280, 0)
    return _label_name_edit


func _make_vec3_row() -> Control:
    var hb := HBoxContainer.new()
    hb.add_theme_constant_override("separation", 4)
    _pos_edits.clear()
    for i in 3:
        var le := LineEdit.new()
        le.custom_minimum_size = Vector2(90, 0)
        hb.add_child(le)
        _pos_edits.append(le)
    return hb


func _make_quat_row() -> Control:
    var hb := HBoxContainer.new()
    hb.add_theme_constant_override("separation", 4)
    _rot_edits.clear()
    for i in 4:
        var le := LineEdit.new()
        le.custom_minimum_size = Vector2(68, 0)
        hb.add_child(le)
        _rot_edits.append(le)
    return hb


# ---------------------------------------------------------------------------
# Populate from entity dict
# ---------------------------------------------------------------------------

func _populate_from_entity() -> void:
    _id_value.text = String(_current_entity.get("id", ""))
    _label_edit.text = str(int(_current_entity.get("label", 0)))
    _label_name_edit.text = String(_current_entity.get("label_name", ""))
    var pos = _current_entity.get("position", [0, 0, 0])
    for i in 3:
        _pos_edits[i].text = "%.4f" % float(pos[i]) if i < pos.size() else "0"
    var rot = _current_entity.get("rotation", [0, 0, 0, 1])
    for i in 4:
        _rot_edits[i].text = "%.4f" % float(rot[i]) if i < rot.size() else ("1" if i == 3 else "0")
    # Wipe + repopulate meta grid
    for child in _meta_grid.get_children():
        child.queue_free()
    _meta_rows.clear()
    var cm: Dictionary = _current_entity.get("custom_meta", {})
    for k in cm.keys():
        _add_meta_row(String(k), str(cm[k]))


func _add_meta_row(key_s: String, val_s: String) -> void:
    var ke := LineEdit.new()
    ke.text = key_s
    ke.custom_minimum_size = Vector2(180, 0)
    _meta_grid.add_child(ke)
    var ve := LineEdit.new()
    ve.text = val_s
    ve.custom_minimum_size = Vector2(280, 0)
    _meta_grid.add_child(ve)
    _meta_rows.append({"key": ke, "val": ve})


# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------

func _on_save_pressed() -> void:
    var updated := _current_entity.duplicate(true)
    updated["label"] = int(_label_edit.text)
    updated["label_name"] = _label_name_edit.text
    updated["position"] = [
        float(_pos_edits[0].text),
        float(_pos_edits[1].text),
        float(_pos_edits[2].text),
    ]
    updated["rotation"] = [
        float(_rot_edits[0].text),
        float(_rot_edits[1].text),
        float(_rot_edits[2].text),
        float(_rot_edits[3].text),
    ]
    var cm: Dictionary = {}
    for row in _meta_rows:
        var k: String = row["key"].text
        if k == "":
            continue
        cm[k] = row["val"].text
    updated["custom_meta"] = cm
    _write_back(updated)
    if _logger != null:
        _logger.info("entity_inspector_save", {"id": updated.get("id", "")})
    emit_signal("entity_committed", updated)
    close()


func _write_back(updated: Dictionary) -> void:
    var path := _world_path + "/entities.json"
    var entities := _read_entities(path)
    var found := false
    var uid := String(updated.get("id", ""))
    for i in entities.size():
        if String(entities[i].get("id", "")) == uid:
            entities[i] = updated
            found = true
            break
    if not found:
        entities.append(updated)
    _write_entities(path, entities)


func _read_entities(path: String) -> Array:
    if not FileAccess.file_exists(path):
        return []
    var txt := FileAccess.get_file_as_string(path)
    if txt.is_empty():
        return []
    var d = JSON.parse_string(txt)
    if d == null or not d.has("entities"):
        return []
    return d.entities


func _write_entities(path: String, entities: Array) -> void:
    var payload := {"format_version": "1.0", "entities": entities}
    var f := FileAccess.open(path, FileAccess.WRITE)
    if f == null:
        push_error("[inspector] cannot write entities.json: " + path)
        return
    f.store_string(JSON.stringify(payload, "  "))
    f.close()


func _input(event: InputEvent) -> void:
    if not visible:
        return
    if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
        close()
        get_viewport().set_input_as_handled()
