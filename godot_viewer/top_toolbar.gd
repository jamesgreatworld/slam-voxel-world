# top_toolbar.gd — always-visible right-aligned toolbar with the global
# actions that used to be keyboard-only: open item picker, save a world
# snapshot, open the pause menu. Built fully in code so no .tscn edits.
#
# Buttons emit named signals; main.gd connects them to the same handlers
# the keyboard shortcuts use, so behaviour stays in sync.

extends CanvasLayer

signal items_pressed
signal snapshot_pressed
signal menu_pressed
signal fly_pressed
signal xray_pressed

const _BTN_MIN_SIZE := Vector2(96, 32)

var _row: HBoxContainer = null
var _fly_btn: Button = null
var _xray_btn: Button = null


func _ready() -> void:
    layer = 30
    process_mode = Node.PROCESS_MODE_ALWAYS

    var anchor := Control.new()
    anchor.set_anchors_preset(Control.PRESET_TOP_RIGHT)
    # 5 buttons × 96px + margins — keep the panel wide enough or the last
    # button silently overflows past the screen edge.
    anchor.set_offset(SIDE_LEFT, -560)
    anchor.set_offset(SIDE_TOP, 8)
    anchor.set_offset(SIDE_RIGHT, -8)
    anchor.set_offset(SIDE_BOTTOM, 48)
    anchor.mouse_filter = Control.MOUSE_FILTER_PASS
    add_child(anchor)

    var panel := PanelContainer.new()
    panel.set_anchors_preset(Control.PRESET_FULL_RECT)
    var sb := StyleBoxFlat.new()
    sb.bg_color = Color(0, 0, 0, 0.35)
    sb.corner_radius_top_left = 6
    sb.corner_radius_top_right = 6
    sb.corner_radius_bottom_left = 6
    sb.corner_radius_bottom_right = 6
    sb.content_margin_left = 6
    sb.content_margin_right = 6
    sb.content_margin_top = 4
    sb.content_margin_bottom = 4
    panel.add_theme_stylebox_override("panel", sb)
    anchor.add_child(panel)

    _row = HBoxContainer.new()
    _row.add_theme_constant_override("separation", 6)
    panel.add_child(_row)

    _add_button("Items  [I]",    func(): emit_signal("items_pressed"))
    _add_button("Snapshot  [F5]", func(): emit_signal("snapshot_pressed"))
    _add_button("Menu  [Esc]",   func(): emit_signal("menu_pressed"))

    _fly_btn = Button.new()
    _fly_btn.toggle_mode = true
    _fly_btn.text = "Fly  [V]"
    _fly_btn.custom_minimum_size = _BTN_MIN_SIZE
    _fly_btn.toggled.connect(func(_p): emit_signal("fly_pressed"))
    _row.add_child(_fly_btn)

    _xray_btn = Button.new()
    _xray_btn.toggle_mode = true
    _xray_btn.text = "X-Ray"
    _xray_btn.custom_minimum_size = _BTN_MIN_SIZE
    _xray_btn.toggled.connect(func(_p): emit_signal("xray_pressed"))
    _row.add_child(_xray_btn)


func set_fly_state(on: bool) -> void:
    if _fly_btn != null:
        _fly_btn.set_pressed_no_signal(on)
        _fly_btn.text = "Fly: ON  [V]" if on else "Fly  [V]"


func set_xray_state(on: bool) -> void:
    if _xray_btn != null:
        _xray_btn.set_pressed_no_signal(on)
        _xray_btn.text = "X-Ray: ON" if on else "X-Ray"


func _add_button(text: String, on_pressed: Callable) -> void:
    var btn := Button.new()
    btn.text = text
    btn.custom_minimum_size = _BTN_MIN_SIZE
    btn.pressed.connect(on_pressed)
    _row.add_child(btn)
