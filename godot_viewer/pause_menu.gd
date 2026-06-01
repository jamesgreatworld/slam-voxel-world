# pause_menu.gd — Layer 6 (UI). Attached to a CanvasLayer in main.tscn.
# Centered Panel + VBox of Buttons. Esc opens; each button emits a signal
# the orchestrator (main.gd) routes to the corresponding action.
#
# This menu replaces the "memorize a dozen shortcuts" UX: every action is
# reachable via mouse click. Shortcuts still work in parallel.

extends CanvasLayer

signal resume_requested
signal toggle_view_requested
signal toggle_edit_mode_requested
signal undo_requested
signal material_picker_requested
signal reset_rig_requested
signal save_world_requested
signal reload_world_requested
signal load_world_requested(path: String)
signal import_litematic_requested(path: String)
signal toggle_day_night_requested
signal quit_requested

@onready var _panel: PanelContainer = $Backdrop/Panel
@onready var _resume_btn: Button = $Backdrop/Panel/VBox/ResumeButton
@onready var _view_btn: Button = $Backdrop/Panel/VBox/ViewButton
@onready var _edit_btn: Button = $Backdrop/Panel/VBox/EditButton
@onready var _undo_btn: Button = $Backdrop/Panel/VBox/UndoButton
@onready var _mat_btn: Button = $Backdrop/Panel/VBox/MaterialButton
@onready var _reset_btn: Button = $Backdrop/Panel/VBox/ResetButton
@onready var _load_btn: Button = $Backdrop/Panel/VBox/LoadButton
@onready var _import_litematic_btn: Button = $Backdrop/Panel/VBox/ImportLitematicButton
@onready var _save_btn: Button = $Backdrop/Panel/VBox/SaveButton
@onready var _reload_btn: Button = $Backdrop/Panel/VBox/ReloadButton
@onready var _quit_btn: Button = $Backdrop/Panel/VBox/QuitButton
@onready var _file_dialog: FileDialog = $LoadDialog
@onready var _litematic_dialog: FileDialog = $LitematicImportDialog

# Day/Night button is added dynamically (main.tscn is not edited).
var _day_night_btn: Button = null
var _is_night: bool = false


func _ready() -> void:
    # Menu and its inputs must keep working when game tree is paused.
    process_mode = Node.PROCESS_MODE_ALWAYS
    visible = false
    _resume_btn.pressed.connect(func(): emit_signal("resume_requested"))
    _view_btn.pressed.connect(func(): emit_signal("toggle_view_requested"))
    _edit_btn.pressed.connect(func(): emit_signal("toggle_edit_mode_requested"))
    _undo_btn.pressed.connect(func(): emit_signal("undo_requested"))
    _mat_btn.pressed.connect(func(): emit_signal("material_picker_requested"))
    _reset_btn.pressed.connect(func(): emit_signal("reset_rig_requested"))
    _load_btn.pressed.connect(_on_load_clicked)
    _import_litematic_btn.pressed.connect(_on_import_litematic_clicked)
    _save_btn.pressed.connect(func(): emit_signal("save_world_requested"))
    _reload_btn.pressed.connect(func(): emit_signal("reload_world_requested"))
    _quit_btn.pressed.connect(func(): emit_signal("quit_requested"))
    _install_day_night_button()
    # FileDialog signals
    _file_dialog.dir_selected.connect(_on_dir_selected)
    _litematic_dialog.file_selected.connect(_on_litematic_selected)
    # Initial dir = project's out/ folder (where .vxw files live)
    var default_dir := ProjectSettings.globalize_path("res://../out")
    if DirAccess.dir_exists_absolute(default_dir):
        _file_dialog.current_dir = default_dir
        _litematic_dialog.current_dir = default_dir


func _on_load_clicked() -> void:
    _file_dialog.popup_centered(Vector2i(800, 500))


func _on_dir_selected(path: String) -> void:
    emit_signal("load_world_requested", path)


func _on_import_litematic_clicked() -> void:
    _litematic_dialog.popup_centered(Vector2i(800, 500))


func _on_litematic_selected(path: String) -> void:
    emit_signal("import_litematic_requested", path)


func set_undo_count(n: int) -> void:
    if n == 0:
        _undo_btn.text = "Undo  (Ctrl+Z)"
        _undo_btn.disabled = true
    else:
        _undo_btn.text = "Undo  (Ctrl+Z)  [%d]" % n
        _undo_btn.disabled = false


func is_open() -> bool:
    return visible


func open() -> void:
    visible = true


func close() -> void:
    visible = false


func toggle() -> void:
    visible = not visible


# Mark Save / Reload as disabled if persistence isn't wired up yet (R3 hooks them).
func set_persistence_available(available: bool) -> void:
    _save_btn.disabled = not available
    _reload_btn.disabled = not available


# Caller (main.gd) keeps the source of truth for whether editing is enabled and
# pushes the state here so the button label always matches.
func set_edit_mode_label(enabled: bool) -> void:
    if enabled:
        _edit_btn.text = "Edit Mode: ON  (LMB destroys)"
        _edit_btn.modulate = Color(1.0, 0.6, 0.4, 1)
    else:
        _edit_btn.text = "Edit Mode: OFF  (clicks ignored)"
        _edit_btn.modulate = Color(1, 1, 1, 1)


# Day/Night button is created dynamically so main.tscn does not need editing.
# Placed in the VBox right before the Quit button.
func _install_day_night_button() -> void:
    _day_night_btn = Button.new()
    _day_night_btn.name = "DayNightButton"
    _day_night_btn.text = "Switch to Night"
    _day_night_btn.custom_minimum_size = Vector2(0, 40)
    _day_night_btn.pressed.connect(func(): emit_signal("toggle_day_night_requested"))
    var vbox: VBoxContainer = $Backdrop/Panel/VBox
    vbox.add_child(_day_night_btn)
    # Move it to just above the Quit button.
    var quit_idx: int = _quit_btn.get_index()
    vbox.move_child(_day_night_btn, quit_idx)


# Called by main.gd after a successful toggle so the button label reflects the
# current state. is_night=true means it is currently night (so button offers to
# switch back to day).
func set_day_night_label(is_night: bool) -> void:
    _is_night = is_night
    if _day_night_btn == null:
        return
    if is_night:
        _day_night_btn.text = "Switch to Day"
    else:
        _day_night_btn.text = "Switch to Night"
