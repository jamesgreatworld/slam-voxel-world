# stereo_rig_controller.gd — Layer 5 (control).
# Attached to the StereoRig Node3D in main.tscn. Owns:
#   - The RIG's 6-DOF pose (this node's own transform)
#   - WASD/arrows/QE/Space/Ctrl/Shift keyboard 6-DOF input
#   - Visibility toggle for the rig's child visuals (RedBox + Eye markers),
#     used by camera_controller in 1P mode (you're inside the rig, hide it)
#   - Per-frame sync of two stereo Camera3Ds (passed in via init_controller)
#     to the rig pose with ±baseline/2 lateral offset
#   - R key: reset pose to identity
#
# Replaceability: the rig represents a "physical stereo camera in the scene"
# (per user's mental model). To swap to a different sensor (mono camera, LiDAR
# scanner, hand-held phone), only this file and its scene children change.

extends Node3D

@export var rig_speed: float = 2.0           # m/s baseline; ×fast_multiplier with Shift
@export var fast_multiplier: float = 4.0
@export var keyboard_look_speed: float = 1.5 # rad/s for yaw + pitch
@export var keyboard_roll_speed: float = 1.2 # rad/s for roll (Q/E)
@export var stereo_baseline: float = 0.10    # meters between L/R eye centres

var _left_cam: Camera3D = null
var _right_cam: Camera3D = null
var _visuals: Array[Node3D] = []


func init_controller(left_cam: Camera3D, right_cam: Camera3D) -> void:
    _left_cam = left_cam
    _right_cam = right_cam
    _visuals.clear()
    for name in ["RedBox", "LeftEyeMarker", "RightEyeMarker"]:
        if has_node(name):
            _visuals.append(get_node(name))


func reset_pose() -> void:
    global_transform = Transform3D.IDENTITY


func set_pose(xf: Transform3D) -> void:
    global_transform = xf


func get_pose() -> Transform3D:
    return global_transform


func set_visuals_visible(v: bool) -> void:
    for n in _visuals:
        n.visible = v


func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed and event.keycode == KEY_R:
        reset_pose()


func _process(delta: float) -> void:
    _apply_keyboard(delta)
    _sync_stereo()


func _apply_keyboard(delta: float) -> void:
    var dir := Vector3.ZERO
    if Input.is_key_pressed(KEY_W):     dir -= transform.basis.z
    if Input.is_key_pressed(KEY_S):     dir += transform.basis.z
    if Input.is_key_pressed(KEY_A):     dir -= transform.basis.x
    if Input.is_key_pressed(KEY_D):     dir += transform.basis.x
    if Input.is_key_pressed(KEY_SPACE): dir += Vector3.UP
    if Input.is_key_pressed(KEY_CTRL):  dir -= Vector3.UP
    var v := rig_speed
    if Input.is_key_pressed(KEY_SHIFT): v *= fast_multiplier
    if dir != Vector3.ZERO:
        translate(dir.normalized() * v * delta)

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


func _sync_stereo() -> void:
    if _left_cam == null or _right_cam == null:
        return
    var xf := global_transform
    var offset := Vector3(stereo_baseline * 0.5, 0.0, 0.0)
    _left_cam.global_transform = xf * Transform3D(Basis.IDENTITY, -offset)
    _right_cam.global_transform = xf * Transform3D(Basis.IDENTITY, offset)
