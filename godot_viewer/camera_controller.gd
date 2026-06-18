# camera_controller.gd — Layer 5 (control).
# Owns the MAIN camera (the user's viewport), independent of the rig.
#
# Two view modes:
#   THIRD_PERSON  main_cam orbits around rig position; mouse drag = rotate orbit,
#                 wheel = zoom in/out. The rig is visible (red box + eyes).
#   FIRST_PERSON  main_cam = rig.global_transform (you ride the rig). The rig's
#                 visuals are hidden so you're not looking at the inside of the box.
#
# Tab toggles. The rig is always controlled by keyboard via stereo_rig_controller;
# this controller never touches the rig's transform, only the main camera's.

extends Node

enum ViewMode { THIRD_PERSON, FIRST_PERSON }

@export var orbit_distance_init: float = 8.0
@export var orbit_yaw_init: float = 0.0          # rad
@export var orbit_pitch_init: float = 0.35       # rad (~20°)
@export var mouse_sensitivity: float = 0.005
@export var zoom_step: float = 1.0
@export var nav_fly_time: float = 0.3   # seconds to fly the orbit centre to a clicked point

var view_mode: int = ViewMode.THIRD_PERSON
var orbit_distance: float
var orbit_yaw: float
var orbit_pitch: float

# First-person free-head: yaw/pitch applied on top of the rig's position
# (only — never the rig's basis), so the cursor turns the camera without
# turning the rig or the stereo PiP viewports.
var _fp_yaw: float = 0.0
var _fp_pitch: float = 0.0
const _FP_MAX_PITCH := PI * 85.0 / 180.0

var _cam: Camera3D = null
var _rig_ctl: Node3D = null   # stereo_rig_controller (Node3D w/ that script)
var _logger = null
var _right_held: bool = false
var _orbit_anchor_pos: Vector2 = Vector2.ZERO  # cursor pos when right-drag began
var _orbit_enabled: bool = true  # turned off in edit mode so RMB goes to editor
var _nav_active: bool = false
var _nav_from: Vector3 = Vector3.ZERO
var _nav_to: Vector3 = Vector3.ZERO
var _nav_t: float = 0.0


func init_controller(cam: Camera3D, rig_ctl: Node3D, logger = null) -> void:
    _cam = cam
    _rig_ctl = rig_ctl
    _logger = logger
    orbit_distance = orbit_distance_init
    orbit_yaw = orbit_yaw_init
    orbit_pitch = orbit_pitch_init
    apply_mouse_mode()
    _notify_rig_physics_mode()  # establish initial mode


func get_view_mode() -> int:
    return view_mode


func get_orbit_state() -> Dictionary:
    return {
        "distance": orbit_distance,
        "yaw": orbit_yaw,
        "pitch": orbit_pitch,
    }


func toggle_view_mode() -> void:
    if view_mode == ViewMode.THIRD_PERSON:
        view_mode = ViewMode.FIRST_PERSON
        # Seed the free-head from the rig's current yaw so the camera starts
        # looking where the rig is facing rather than snapping to world +Z.
        var e: Vector3 = _rig_ctl.global_transform.basis.get_euler()
        _fp_yaw = e.y
        _fp_pitch = clamp(e.x, -_FP_MAX_PITCH, _FP_MAX_PITCH)
    else:
        view_mode = ViewMode.THIRD_PERSON
    apply_mouse_mode()
    _notify_rig_physics_mode()
    if _logger:
        _logger.info("view_mode", {"now": "1P" if view_mode == ViewMode.FIRST_PERSON else "3P"})


func set_view_mode(m: int) -> void:
    view_mode = m
    apply_mouse_mode()
    _notify_rig_physics_mode()


# Forward the view mode to the rig so it can switch between free-fly and
# physics-based walking. Safe to call repeatedly.
func _notify_rig_physics_mode() -> void:
    if _rig_ctl != null and _rig_ctl.has_method("set_physics_mode"):
        _rig_ctl.set_physics_mode(view_mode == ViewMode.FIRST_PERSON)


# Edit mode owns RMB for placement; disable orbit while it's on. Called by
# main.gd when the user toggles edit mode in the pause menu.
func set_orbit_enabled(enabled: bool) -> void:
    _orbit_enabled = enabled
    if not enabled and _right_held:
        # Release any in-progress drag so we don't leave the mouse captured.
        Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
        Input.warp_mouse(_orbit_anchor_pos)
        _right_held = false


# Convenience for cmdline / cross-module use without accessing the enum directly.
func set_view_mode_str(s: String) -> void:
    if s == "1p" or s == "first_person":
        set_view_mode(ViewMode.FIRST_PERSON)
    elif s == "3p" or s == "third_person":
        set_view_mode(ViewMode.THIRD_PERSON)
    else:
        push_warning("[camera_controller] unknown view mode string: %s" % s)


func apply_mouse_mode() -> void:
    # 3P: free cursor for hover-picking; right-drag will temporarily capture for orbit.
    # 1P: CAPTURED — lock cursor to screen centre, FPS-style. Mouse motion is
    # routed to stereo_rig.apply_mouse_look() so the rig yaws/pitches.
    if view_mode == ViewMode.THIRD_PERSON:
        Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
    else:
        Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
    _right_held = false


func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed and event.keycode == KEY_TAB:
        toggle_view_mode()
        return
    # 1P FPS look: mouse motion only rotates main_cam (a free head on top of
    # the rig). The rig itself + the L/R stereo PiP keep their orientation —
    # the cursor never moves the red box or the side viewports.
    if view_mode == ViewMode.FIRST_PERSON:
        if event is InputEventMouseMotion:
            _fp_yaw -= event.relative.x * mouse_sensitivity
            _fp_pitch -= event.relative.y * mouse_sensitivity
            _fp_pitch = clamp(_fp_pitch, -_FP_MAX_PITCH, _FP_MAX_PITCH)
        return

    if event is InputEventMouseButton:
        if event.button_index == MOUSE_BUTTON_LEFT and event.pressed and event.double_click \
           and view_mode == ViewMode.THIRD_PERSON and _orbit_enabled:
            var mp := get_viewport().get_mouse_position()
            var from := _cam.project_ray_origin(mp)
            var dir := _cam.project_ray_normal(mp)
            var q := PhysicsRayQueryParameters3D.create(from, from + dir * 1000.0)
            var hit := _cam.get_world_3d().direct_space_state.intersect_ray(q)
            if not hit.is_empty():
                navigate_to(hit.position)
                get_viewport().set_input_as_handled()
            return
        if event.button_index == MOUSE_BUTTON_RIGHT:
            if not _orbit_enabled:
                return   # editor owns RMB while edit mode is on
            # Right-drag orbit: capture cursor on press, release on let-go.
            if event.pressed:
                _orbit_anchor_pos = get_viewport().get_mouse_position()
                Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
                _right_held = true
            else:
                Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
                Input.warp_mouse(_orbit_anchor_pos)
                _right_held = false
        elif event.pressed:
            if event.button_index == MOUSE_BUTTON_WHEEL_UP:
                orbit_distance = max(0.5, orbit_distance - zoom_step)
            elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
                orbit_distance += zoom_step
    elif event is InputEventMouseMotion and _right_held:
        orbit_yaw -= event.relative.x * mouse_sensitivity
        orbit_pitch -= event.relative.y * mouse_sensitivity
        orbit_pitch = clamp(orbit_pitch, -PI / 2 + 0.1, PI / 2 - 0.1)


# Fly the orbit centre (the rig origin) smoothly to a world point. 3P only.
# Called by double-click navigation; the camera keeps orbiting the moving
# centre, so the view reframes onto the clicked point.
func navigate_to(world_point: Vector3) -> void:
    if view_mode != ViewMode.THIRD_PERSON or _rig_ctl == null:
        return
    _nav_from = _rig_ctl.global_position
    _nav_to = world_point
    _nav_t = 0.0
    _nav_active = true


func _process(_delta: float) -> void:
    if _cam == null or _rig_ctl == null:
        return
    var rig_xf: Transform3D = _rig_ctl.global_transform
    if view_mode == ViewMode.FIRST_PERSON:
        # Position only from the rig; orientation is the free-head yaw/pitch.
        # Result: rig + L/R PiP unchanged, only main_cam swivels.
        var basis := Basis.IDENTITY \
            .rotated(Vector3.UP, _fp_yaw) \
            .rotated(Vector3.RIGHT, _fp_pitch)
        _cam.global_transform = Transform3D(basis, rig_xf.origin)
        _rig_ctl.set_visuals_visible(false)
    else:
        if _nav_active:
            _nav_t = min(1.0, _nav_t + _delta / max(0.0001, nav_fly_time))
            var ease: float = _nav_t * _nav_t * (3.0 - 2.0 * _nav_t)   # smoothstep
            _rig_ctl.global_position = _nav_from.lerp(_nav_to, ease)
            if _nav_t >= 1.0:
                _nav_active = false
        var offset := Vector3(
            cos(orbit_pitch) * sin(orbit_yaw),
            sin(orbit_pitch),
            cos(orbit_pitch) * cos(orbit_yaw)
        ) * orbit_distance
        _cam.global_position = rig_xf.origin + offset
        _cam.look_at(rig_xf.origin, Vector3.UP)
        _rig_ctl.set_visuals_visible(true)
