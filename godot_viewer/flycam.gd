# flycam.gd — 6-DOF keyboard-only fly camera (no mouse).
# Attach to a Camera3D.
#
# Controls:
#   W/S         move forward / back
#   A/D         strafe left / right
#   Space/Ctrl  move up / down
#   ←/→         yaw left / right
#   ↑/↓         pitch up / down
#   Q/E         roll left / right
#   Shift       boost (×fast_multiplier)

extends Camera3D

@export var speed: float = 10.0           # meters / second
@export var fast_multiplier: float = 4.0
@export var keyboard_look_speed: float = 1.8  # radians / second for yaw/pitch
@export var keyboard_roll_speed: float = 1.2  # radians / second for roll


func _ready() -> void:
    Input.mouse_mode = Input.MOUSE_MODE_HIDDEN  # cursor invisible, no mouse-look


func _process(delta: float) -> void:
    # --- Translation in local frame ---
    var dir := Vector3.ZERO
    if Input.is_key_pressed(KEY_W):     dir -= transform.basis.z
    if Input.is_key_pressed(KEY_S):     dir += transform.basis.z
    if Input.is_key_pressed(KEY_A):     dir -= transform.basis.x
    if Input.is_key_pressed(KEY_D):     dir += transform.basis.x
    if Input.is_key_pressed(KEY_SPACE): dir += Vector3.UP    # world-up, not local
    if Input.is_key_pressed(KEY_CTRL):  dir -= Vector3.UP
    var v := speed
    if Input.is_key_pressed(KEY_SHIFT): v *= fast_multiplier
    if dir != Vector3.ZERO:
        translate(dir.normalized() * v * delta)

    # --- Rotation in local frame (apply as local-axis rotations to preserve roll) ---
    var dyaw := 0.0
    var dpitch := 0.0
    var droll := 0.0
    if Input.is_key_pressed(KEY_LEFT):  dyaw += 1.0
    if Input.is_key_pressed(KEY_RIGHT): dyaw -= 1.0
    if Input.is_key_pressed(KEY_UP):    dpitch += 1.0
    if Input.is_key_pressed(KEY_DOWN):  dpitch -= 1.0
    if Input.is_key_pressed(KEY_Q):     droll += 1.0
    if Input.is_key_pressed(KEY_E):     droll -= 1.0
    if dpitch != 0.0:
        rotate_object_local(Vector3.RIGHT, dpitch * keyboard_look_speed * delta)
    if dyaw != 0.0:
        rotate_object_local(Vector3.UP, dyaw * keyboard_look_speed * delta)
    if droll != 0.0:
        rotate_object_local(Vector3.FORWARD, droll * keyboard_roll_speed * delta)
