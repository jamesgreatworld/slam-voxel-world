# pause_menu.gd — Layer 6 (UI). Attached to a CanvasLayer in main.tscn.
# Centered Panel + VBox of Buttons. Esc opens; each button emits a signal
# the orchestrator (main.gd) routes to the corresponding action.
#
# This menu replaces the "memorize a dozen shortcuts" UX: every action is
# reachable via mouse click. Shortcuts still work in parallel.

extends CanvasLayer

signal resume_requested
signal toggle_view_requested
signal reset_rig_requested
signal save_world_requested
signal reload_world_requested
signal quit_requested

@onready var _panel: PanelContainer = $Backdrop/Panel
@onready var _resume_btn: Button = $Backdrop/Panel/VBox/ResumeButton
@onready var _view_btn: Button = $Backdrop/Panel/VBox/ViewButton
@onready var _reset_btn: Button = $Backdrop/Panel/VBox/ResetButton
@onready var _save_btn: Button = $Backdrop/Panel/VBox/SaveButton
@onready var _reload_btn: Button = $Backdrop/Panel/VBox/ReloadButton
@onready var _quit_btn: Button = $Backdrop/Panel/VBox/QuitButton


func _ready() -> void:
    # Menu and its inputs must keep working when game tree is paused.
    process_mode = Node.PROCESS_MODE_ALWAYS
    visible = false
    _resume_btn.pressed.connect(func(): emit_signal("resume_requested"))
    _view_btn.pressed.connect(func(): emit_signal("toggle_view_requested"))
    _reset_btn.pressed.connect(func(): emit_signal("reset_rig_requested"))
    _save_btn.pressed.connect(func(): emit_signal("save_world_requested"))
    _reload_btn.pressed.connect(func(): emit_signal("reload_world_requested"))
    _quit_btn.pressed.connect(func(): emit_signal("quit_requested"))


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
