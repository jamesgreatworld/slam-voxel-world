# item_picker.gd — UI for browsing MC-item presets and spawning entities.
#
# Press `I` to toggle the picker. Each preset is a button coloured by the
# preset's average face colour with the item name as label. Clicking a button
# emits item_chosen(item_id); main wires that to a spawn handler.
#
# This panel is fully built in code at _ready time (no .tscn dependency), so
# we don't have to edit main.tscn — which has been brittle in this codebase.

extends CanvasLayer

signal item_chosen(item_id: String)

var _backdrop: ColorRect = null
var _panel: PanelContainer = null
var _grid: GridContainer = null
var _title: Label = null
var _hint: Label = null
var _close_btn: Button = null
var _buttons: Dictionary = {}    # item_id → Button

var _presets_by_id: Dictionary = {}
var _logger = null


func _ready() -> void:
    process_mode = Node.PROCESS_MODE_ALWAYS
    layer = 50
    visible = false
    _build_ui()


func init_picker(logger = null) -> void:
    _logger = logger


func set_presets(presets: Dictionary) -> void:
    """presets: id → preset-dict (boxes, overall_extents_m, default_label, ...)."""
    _presets_by_id = presets
    _populate()


func open() -> void:
    visible = true


func close() -> void:
    visible = false


func is_open() -> bool:
    return visible


func toggle() -> void:
    visible = not visible


# ----------------------------------------------------------------------------
# UI construction
# ----------------------------------------------------------------------------

func _build_ui() -> void:
    _backdrop = ColorRect.new()
    _backdrop.color = Color(0, 0, 0, 0.55)
    _backdrop.anchor_right = 1.0
    _backdrop.anchor_bottom = 1.0
    _backdrop.mouse_filter = Control.MOUSE_FILTER_STOP
    add_child(_backdrop)

    _panel = PanelContainer.new()
    _panel.custom_minimum_size = Vector2(560, 360)
    _panel.set_anchors_preset(Control.PRESET_CENTER)
    _panel.set_offset(SIDE_LEFT, -280)
    _panel.set_offset(SIDE_TOP, -180)
    _panel.set_offset(SIDE_RIGHT, 280)
    _panel.set_offset(SIDE_BOTTOM, 180)
    _backdrop.add_child(_panel)

    var vbox := VBoxContainer.new()
    vbox.add_theme_constant_override("separation", 8)
    _panel.add_child(vbox)

    _title = Label.new()
    _title.text = "Items — click to spawn 1.5m in front of rig"
    _title.add_theme_font_size_override("font_size", 18)
    vbox.add_child(_title)

    _hint = Label.new()
    _hint.text = "(Press I to close, RMB on an entity in the world to delete)"
    _hint.add_theme_font_size_override("font_size", 12)
    _hint.modulate = Color(0.8, 0.8, 0.8)
    vbox.add_child(_hint)

    var scroll := ScrollContainer.new()
    scroll.custom_minimum_size = Vector2(540, 250)
    vbox.add_child(scroll)

    _grid = GridContainer.new()
    _grid.columns = 3
    _grid.add_theme_constant_override("h_separation", 8)
    _grid.add_theme_constant_override("v_separation", 8)
    scroll.add_child(_grid)

    _close_btn = Button.new()
    _close_btn.text = "Close (I)"
    _close_btn.pressed.connect(close)
    vbox.add_child(_close_btn)


func _populate() -> void:
    for child in _grid.get_children():
        child.queue_free()
    _buttons.clear()

    var ids := _presets_by_id.keys()
    ids.sort()
    for id in ids:
        var preset: Dictionary = _presets_by_id[id]
        var btn := _make_swatch(String(id), preset)
        _grid.add_child(btn)
        _buttons[id] = btn


func _make_swatch(item_id: String, preset: Dictionary) -> Button:
    var btn := Button.new()
    var category: String = String(preset.get("category", "?"))
    var extents = preset.get("overall_extents_m", [0, 0, 0])
    var ext_text := "%.1f×%.1f×%.1f m" % [
        float(extents[0]), float(extents[1]), float(extents[2])
    ]
    btn.text = "%s\n%s\n%s" % [item_id, category, ext_text]
    btn.custom_minimum_size = Vector2(160, 70)
    btn.add_theme_constant_override("h_separation", 4)
    btn.clip_text = false
    btn.alignment = HORIZONTAL_ALIGNMENT_CENTER

    # Tint background by preset average colour (across all sub-boxes/faces).
    var col := _preset_avg_color(preset)
    var sb := StyleBoxFlat.new()
    sb.bg_color = col
    sb.border_width_left = 1; sb.border_width_right = 1
    sb.border_width_top = 1; sb.border_width_bottom = 1
    sb.border_color = Color(0, 0, 0, 1)
    sb.content_margin_left = 6; sb.content_margin_right = 6
    sb.content_margin_top = 4; sb.content_margin_bottom = 4
    btn.add_theme_stylebox_override("normal", sb)
    btn.add_theme_stylebox_override("hover", sb)
    btn.add_theme_stylebox_override("pressed", sb)
    var luma := 0.2126 * col.r + 0.7152 * col.g + 0.0722 * col.b
    btn.add_theme_color_override(
        "font_color", Color.BLACK if luma > 0.55 else Color.WHITE
    )
    btn.pressed.connect(func(): _on_pressed(item_id))
    return btn


func _preset_avg_color(preset: Dictionary) -> Color:
    var r := 0.0; var g := 0.0; var b := 0.0; var n := 0
    for box in preset.get("boxes", []):
        var fc: Dictionary = box.get("face_colors") if box.has("face_colors") else {}
        for c in fc.values():
            if c.size() < 3: continue
            r += float(c[0]); g += float(c[1]); b += float(c[2])
            n += 1
    if n == 0:
        return Color(0.5, 0.5, 0.5)
    return Color(r / n / 255.0, g / n / 255.0, b / n / 255.0)


func _on_pressed(item_id: String) -> void:
    if _logger != null:
        _logger.info("item_chosen", {"id": item_id})
    emit_signal("item_chosen", item_id)
    close()


func _input(event: InputEvent) -> void:
    if not visible:
        return
    if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
        close()
