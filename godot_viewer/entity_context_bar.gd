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
signal use_pressed
signal move_pressed

const _BTN_MIN_SIZE := Vector2(120, 36)
const _ROT_STEP := PI / 18.0   # 10° per click

var _row: HBoxContainer = null
var _label: Label = null


func _ready() -> void:
    layer = 30
    visible = false
    process_mode = Node.PROCESS_MODE_ALWAYS

    var anchor := Control.new()
    anchor.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
    anchor.set_offset(SIDE_TOP, -78)
    anchor.set_offset(SIDE_BOTTOM, -24)
    anchor.mouse_filter = Control.MOUSE_FILTER_PASS
    add_child(anchor)

    # CenterContainer sizes the panel to its content and centres it, so the bar
    # is only as wide as the buttons it holds (no empty fixed-width band).
    var center := CenterContainer.new()
    center.set_anchors_preset(Control.PRESET_FULL_RECT)
    center.mouse_filter = Control.MOUSE_FILTER_PASS
    anchor.add_child(center)

    var panel := PanelContainer.new()
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
    center.add_child(panel)

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

    # Mouse-only, minimal: pick-up-to-move, rotate ±10° per click, duplicate, delete.
    _add_button("移动",     func(): emit_signal("move_pressed"))
    _add_button("⟲ -10°",  func(): emit_signal("rotate_pressed", -_ROT_STEP))
    _add_button("⟳ +10°",  func(): emit_signal("rotate_pressed",  _ROT_STEP))
    _add_button("复制",     func(): emit_signal("duplicate_pressed"))
    _add_button("删除",     func(): emit_signal("delete_pressed"))


func _add_button(text: String, on_pressed: Callable) -> void:
    var btn := Button.new()
    btn.text = text
    btn.custom_minimum_size = _BTN_MIN_SIZE
    btn.pressed.connect(on_pressed)
    _row.add_child(btn)


# main wires entity_selector.entity_selected / selection_cleared into these.
var _last_label_name: String = "?"


func on_entity_selected(id: String, label_name: String = "") -> void:
    visible = true
    _last_label_name = label_name if label_name != "" else "?"
    _set_selected_label()


func _set_selected_label() -> void:
    _label.text = "已选中:%s　—　点[移动]→移到目标→单击地面放置" % [_last_label_name]


# Toggle the move-mode banner: while moving, hide every button and show only a
# clear instruction, so the user can't click rotate/delete by mistake (which
# would teleport the followed entity) and the only ways out are place / cancel.
func set_move_mode(active: bool) -> void:
    if active:
        visible = true
        _row.visible = false
        _label.text = "🖐 移动中:把鼠标移到目标 →  单击地面放下　(右键取消)"
    else:
        _row.visible = true
        _set_selected_label()


func on_selection_cleared() -> void:
    visible = false
    _label.text = "Selected: (none)"
