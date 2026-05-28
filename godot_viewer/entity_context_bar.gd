# entity_context_bar.gd — bottom-centered toolbar that appears only when an
# entity is selected. Replicates the selection-time keyboard shortcuts as
# clickable buttons:
#
#   Inspector (F2)   open inspector
#   Duplicate (Ctrl+D)
#   Physics  (P)     toggle the selected entity's physics_dynamic flag
#   Rotate-  /  Rotate+   (R rotates +45° via keyboard; we expose both)
#   Delete (Del)
#
# Emits named signals so main.gd dispatches the existing methods.

extends CanvasLayer

signal inspector_pressed
signal duplicate_pressed
signal physics_toggle_pressed
signal rotate_pressed(yaw_delta_rad: float)
signal delete_pressed

const _BTN_MIN_SIZE := Vector2(120, 36)
const _ROT_STEP := PI / 4.0   # 45°

var _row: HBoxContainer = null
var _label: Label = null


func _ready() -> void:
    layer = 30
    visible = false
    process_mode = Node.PROCESS_MODE_ALWAYS

    var anchor := Control.new()
    anchor.set_anchors_preset(Control.PRESET_CENTER_BOTTOM)
    anchor.set_offset(SIDE_LEFT, -460)
    anchor.set_offset(SIDE_TOP, -68)
    anchor.set_offset(SIDE_RIGHT, 460)
    anchor.set_offset(SIDE_BOTTOM, -28)
    anchor.mouse_filter = Control.MOUSE_FILTER_PASS
    add_child(anchor)

    var panel := PanelContainer.new()
    panel.set_anchors_preset(Control.PRESET_FULL_RECT)
    var sb := StyleBoxFlat.new()
    sb.bg_color = Color(0, 0, 0, 0.55)
    sb.border_width_top = 1
    sb.border_width_bottom = 1
    sb.border_width_left = 1
    sb.border_width_right = 1
    sb.border_color = Color(1.0, 0.85, 0.2, 0.8)
    sb.corner_radius_top_left = 8
    sb.corner_radius_top_right = 8
    sb.corner_radius_bottom_left = 8
    sb.corner_radius_bottom_right = 8
    sb.content_margin_left = 8
    sb.content_margin_right = 8
    sb.content_margin_top = 4
    sb.content_margin_bottom = 4
    panel.add_theme_stylebox_override("panel", sb)
    anchor.add_child(panel)

    var vbox := VBoxContainer.new()
    vbox.add_theme_constant_override("separation", 4)
    panel.add_child(vbox)

    _label = Label.new()
    _label.text = "Selected: (none)"
    _label.add_theme_color_override("font_color", Color(1, 0.9, 0.4, 1))
    _label.add_theme_font_size_override("font_size", 12)
    _label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
    vbox.add_child(_label)

    _row = HBoxContainer.new()
    _row.add_theme_constant_override("separation", 6)
    vbox.add_child(_row)

    _add_button("Inspector [F2]",    func(): emit_signal("inspector_pressed"))
    _add_button("Duplicate [Ctrl+D]", func(): emit_signal("duplicate_pressed"))
    _add_button("Physics [P]",       func(): emit_signal("physics_toggle_pressed"))
    _add_button("Rotate -45° [R]",   func(): emit_signal("rotate_pressed", -_ROT_STEP))
    _add_button("Rotate +45°",       func(): emit_signal("rotate_pressed",  _ROT_STEP))
    _add_button("Delete [Del]",      func(): emit_signal("delete_pressed"))


func _add_button(text: String, on_pressed: Callable) -> void:
    var btn := Button.new()
    btn.text = text
    btn.custom_minimum_size = _BTN_MIN_SIZE
    btn.pressed.connect(on_pressed)
    _row.add_child(btn)


# main wires entity_selector.entity_selected / selection_cleared into these.
func on_entity_selected(id: String, label_name: String = "") -> void:
    visible = true
    var short_id := id.substr(0, 8)
    _label.text = "Selected: %s  (id %s)" % [label_name if label_name != "" else "?", short_id]


func on_selection_cleared() -> void:
    visible = false
    _label.text = "Selected: (none)"
