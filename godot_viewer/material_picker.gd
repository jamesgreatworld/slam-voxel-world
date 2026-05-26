# material_picker.gd — Layer 6 (UI).
# Floating panel attached to the pause menu showing a grid of material swatches.
# Clicking a swatch sets voxel_editor's current placement material.
#
# Built from world.palette_materials at init_picker time. Updates if a new
# world is loaded.

extends CanvasLayer

signal material_selected(material_id: int)

@onready var _panel: PanelContainer = $Backdrop/Panel
@onready var _grid: GridContainer = $Backdrop/Panel/VBox/Grid
@onready var _title: Label = $Backdrop/Panel/VBox/Title
@onready var _close_btn: Button = $Backdrop/Panel/VBox/CloseButton

var _current_id: int = 1
var _swatch_buttons: Dictionary = {}  # material_id → Button


func _ready() -> void:
    process_mode = Node.PROCESS_MODE_ALWAYS
    visible = false
    _close_btn.pressed.connect(close)


func open(world) -> void:
    _populate(world)
    visible = true


func close() -> void:
    visible = false


func is_open() -> bool:
    return visible


func set_current(material_id: int) -> void:
    _current_id = material_id
    for mid in _swatch_buttons.keys():
        var btn: Button = _swatch_buttons[mid]
        if mid == material_id:
            btn.modulate = Color(1.5, 1.5, 0.5)  # highlight selected
        else:
            btn.modulate = Color(1, 1, 1, 1)


func _populate(world) -> void:
    # Clear old swatches
    for child in _grid.get_children():
        child.queue_free()
    _swatch_buttons.clear()
    _title.text = "Choose placement material"
    var palette_materials: Dictionary = world.palette_materials
    var ids: Array = palette_materials.keys()
    ids.sort()
    for mid in ids:
        if mid == 0:
            continue   # skip air
        var info: Dictionary = palette_materials[mid]
        var btn := Button.new()
        btn.text = "%2d %s" % [mid, info.get("name", "?")]
        btn.custom_minimum_size = Vector2(170, 36)
        var rgb = info.get("color_rgb", [180, 180, 180])
        var c := Color(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0)
        # Tint the button background by setting modulate on a stylebox is heavy;
        # instead set the text colour to contrast and use modulate on the button.
        btn.add_theme_color_override("font_color", Color.BLACK if (c.r + c.g + c.b) > 1.5 else Color.WHITE)
        var sb := StyleBoxFlat.new()
        sb.bg_color = c
        sb.border_width_left = 1
        sb.border_width_right = 1
        sb.border_width_top = 1
        sb.border_width_bottom = 1
        sb.border_color = Color(0, 0, 0, 1)
        btn.add_theme_stylebox_override("normal", sb)
        btn.add_theme_stylebox_override("hover", sb)
        btn.add_theme_stylebox_override("pressed", sb)
        btn.pressed.connect(func(): _on_swatch_pressed(mid))
        _grid.add_child(btn)
        _swatch_buttons[mid] = btn
    set_current(_current_id)


func _on_swatch_pressed(mid: int) -> void:
    set_current(mid)
    emit_signal("material_selected", mid)


func _input(event: InputEvent) -> void:
    if not visible: return
    if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
        close()
