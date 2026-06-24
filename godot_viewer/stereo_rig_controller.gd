# stereo_rig_controller.gd — Layer 5 (control).
# Attached to the StereoRig CharacterBody3D in main.tscn. Owns:
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

extends CharacterBody3D

@export var rig_speed: float = 2.0           # m/s baseline; ×fast_multiplier with Shift
@export var fast_multiplier: float = 4.0
@export var keyboard_look_speed: float = 1.5 # rad/s for yaw + pitch
@export var keyboard_roll_speed: float = 1.2 # rad/s for roll (Q/E)
@export var stereo_baseline: float = 0.10    # meters between L/R eye centres
# Physics-mode parameters
@export var gravity_mps2: float = 9.81
@export var jump_velocity_mps: float = 4.5
@export var eye_height_m: float = 1.6
@export var capsule_radius_m: float = 0.3
@export var capsule_stand_h_m: float = 1.7
@export var floor_max_angle_deg: float = 46.0
@export var max_step_m: float = 0.35
@export var capsule_crouch_h_m: float = 0.9

var _crouched: bool = false
var _spawn_origin: Vector3 = Vector3(0, 1, 0)

var _left_cam: Camera3D = null
var _right_cam: Camera3D = null
var _visuals: Array[Node3D] = []
# _physics_mode is kept for backward compatibility (no-op; movement is now
# always physics unless _fly_mode is true).
var _physics_mode: bool = true
var _body_shape: CollisionShape3D = null

# Movement-mode flag: when true the rig flies freely (no gravity/collision).
# Toggled by camera_controller via V key.
var _fly_mode: bool = false

# Camera yaw reference for camera-relative movement.
# In 1P: set to _fp_yaw (look direction).  In 3P: set to orbit_yaw.
var _move_yaw: float = 0.0

# True when camera is in THIRD_PERSON; rig faces its movement direction.
var _third_person: bool = true

# Walk-cycle animation state.
var _walk_cycle: float = 0.0
var _last_pos: Vector3 = Vector3.ZERO
var _last_pos_inited: bool = false
const _WALK_FREQ_HZ: float = 1.6
const _WALK_AMPL_RAD: float = 0.5    # ±28°, roughly natural
const _WALK_DAMP_PER_SEC: float = 8.0 # fade to 0 when not moving
const _WALK_VEL_THRESHOLD: float = 0.05  # m/s gate for "moving"
var _left_arm: Node3D = null
var _right_arm: Node3D = null
var _left_leg: Node3D = null
var _right_leg: Node3D = null

# A2 sit/lay state.
var _seated_entity_id: String = ""
const _SIT_LIFT_ABOVE_ENTITY_CENTRE: float = 0.45
const _SIT_LEG_PITCH: float = PI * 0.5

# One-shot physics self-check counter (headless smoke test).
# Counts up to 90 physics frames, then prints settled state.
var _phys_check_frame: int = 0


const HumanoidVisualScript = preload("res://humanoid_visual.gd")


func init_controller(left_cam: Camera3D, right_cam: Camera3D) -> void:
    _left_cam = left_cam
    _right_cam = right_cam
    if has_node("RedBox"):
        get_node("RedBox").visible = false
    var humanoid: Node3D = HumanoidVisualScript.build()
    add_child(humanoid)
    _visuals = [humanoid]
    for n in ["LeftEyeMarker", "RightEyeMarker"]:
        if has_node(n):
            _visuals.append(get_node(n))
    _left_arm  = humanoid.get_node_or_null("LeftArm")
    _right_arm = humanoid.get_node_or_null("RightArm")
    _left_leg  = humanoid.get_node_or_null("LeftLeg")
    _right_leg = humanoid.get_node_or_null("RightLeg")

    _body_shape = CollisionShape3D.new()
    _body_shape.name = "BodyShape"
    var cap := CapsuleShape3D.new()
    cap.radius = capsule_radius_m
    cap.height = capsule_stand_h_m
    _body_shape.shape = cap
    _body_shape.position = Vector3(0.0, -(eye_height_m - capsule_stand_h_m * 0.5), 0.0)
    add_child(_body_shape)
    self.floor_max_angle = deg_to_rad(floor_max_angle_deg)
    self.floor_snap_length = 0.3


@export var spawn_y_m: float = 1.0

func reset_pose() -> void:
    global_transform = Transform3D(Basis.IDENTITY, Vector3(0.0, spawn_y_m, 0.0))
    velocity = Vector3.ZERO
    _spawn_origin = global_position


func set_pose(xf: Transform3D) -> void:
    global_transform = xf
    velocity = Vector3.ZERO
    _spawn_origin = global_position


func get_pose() -> Transform3D:
    return global_transform


# ---------------------------------------------------------------------------
# A2 sit
# ---------------------------------------------------------------------------

func enter_sit(entity_id: String, entity_pos: Vector3, entity_yaw_rad: float) -> void:
    _seated_entity_id = entity_id
    velocity = Vector3.ZERO
    var basis := Basis().rotated(Vector3.UP, entity_yaw_rad)
    var pos := entity_pos + Vector3(0.0, _SIT_LIFT_ABOVE_ENTITY_CENTRE, 0.0)
    global_transform = Transform3D(basis, pos)
    if _left_leg  != null: _left_leg.rotation.x  = _SIT_LEG_PITCH
    if _right_leg != null: _right_leg.rotation.x = _SIT_LEG_PITCH


func exit_seat() -> void:
    _seated_entity_id = ""
    velocity = Vector3.ZERO
    if _left_leg  != null: _left_leg.rotation.x  = 0.0
    if _right_leg != null: _right_leg.rotation.x = 0.0


func is_seated() -> bool:
    return _seated_entity_id != ""


func get_seated_entity_id() -> String:
    return _seated_entity_id


# ---------------------------------------------------------------------------
# Movement mode API (called by camera_controller)
# ---------------------------------------------------------------------------

# Fly toggle (V key). When on: free-fly, no gravity/collision.
func set_fly_mode(on: bool) -> void:
    _fly_mode = on
    velocity = Vector3.ZERO
    _phys_check_frame = 0


# Camera yaw reference so WASD is relative to the camera view.
func set_move_yaw(y: float) -> void:
    _move_yaw = y


# Tell the rig which perspective the camera is using.
func set_third_person(on: bool) -> void:
    _third_person = on


# Kept for backward compatibility; physics is now always on unless _fly_mode.
# Calling this no longer has any effect on movement mode.
func set_physics_mode(enabled: bool) -> void:
    _physics_mode = enabled   # stored but ignored; movement driven by _fly_mode
    _phys_check_frame = 0


func set_visuals_visible(v: bool) -> void:
    for n in _visuals:
        n.visible = v


func _physics_process(delta: float) -> void:
    # Physics runs whenever NOT flying/seated, regardless of view mode.
    if _fly_mode or is_seated():
        return
    # Fall-through respawn safety: snap back if dropped far below spawn.
    if global_position.y < _spawn_origin.y - 50.0:
        global_position = _spawn_origin
        velocity = Vector3.ZERO
        return
    _apply_keyboard_physics(delta)
    _sync_stereo()

    # One-shot physics self-check (headless smoke).
    if _phys_check_frame >= 0:
        _phys_check_frame += 1
        if _phys_check_frame == 90:
            print("[rig] 1P self-check y=%.2f on_floor=%s vel_y=%.2f" % [global_position.y, str(is_on_floor()), velocity.y])
            _phys_check_frame = -1


func _process(delta: float) -> void:
    if is_seated():
        _sync_stereo()
        return
    if _fly_mode:
        _apply_keyboard_freefly(delta)
        _sync_stereo()
    _animate_walk(delta)


# Advance the walk cycle when moving horizontally.
func _animate_walk(delta: float) -> void:
    if _left_arm == null:
        return
    var pos := global_position
    var horizontal_speed := 0.0
    if _last_pos_inited:
        var dx := pos.x - _last_pos.x
        var dz := pos.z - _last_pos.z
        horizontal_speed = sqrt(dx * dx + dz * dz) / max(delta, 0.0001)
    _last_pos = pos
    _last_pos_inited = true

    var moving := horizontal_speed > _WALK_VEL_THRESHOLD
    var target_speed := horizontal_speed if moving else 0.0
    var cycle_rate: float = min(target_speed, 4.0) * _WALK_FREQ_HZ
    _walk_cycle = fmod(_walk_cycle + cycle_rate * delta, TAU)

    var swing := sin(_walk_cycle) * _WALK_AMPL_RAD
    if not moving:
        swing = lerp(_get_current_swing(), 0.0,
                     clamp(delta * _WALK_DAMP_PER_SEC, 0.0, 1.0))

    if _left_leg  != null: _left_leg.rotation.x  = swing
    if _right_leg != null: _right_leg.rotation.x = -swing
    if _left_arm  != null: _left_arm.rotation.x  = -swing
    if _right_arm != null: _right_arm.rotation.x = swing


func _get_current_swing() -> float:
    if _left_leg == null:
        return 0.0
    return _left_leg.rotation.x


# Free-fly mode: ignores gravity and collision.
func _apply_keyboard_freefly(delta: float) -> void:
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

    _apply_rotation_keys(delta)


# Physics movement: camera-relative WASD, gravity, jump, move_and_slide.
# Active in BOTH 1P and 3P whenever _fly_mode is false.
func _apply_keyboard_physics(delta: float) -> void:
    # 0) Crouch toggle (Ctrl)
    var want_crouch := Input.is_key_pressed(KEY_CTRL)
    if want_crouch and not _crouched:
        _set_capsule_height(capsule_crouch_h_m); _crouched = true
    elif not want_crouch and _crouched:
        var dh := capsule_stand_h_m - capsule_crouch_h_m
        if not test_move(global_transform, Vector3(0.0, dh, 0.0)):
            _set_capsule_height(capsule_stand_h_m); _crouched = false

    # 1) Horizontal movement — camera-relative via _move_yaw
    var yaw_basis := Basis(Vector3.UP, _move_yaw)
    var fwd: Vector3 = -yaw_basis.z; fwd.y = 0.0
    var right: Vector3 = yaw_basis.x; right.y = 0.0
    if fwd.length_squared() > 0.0001:   fwd   = fwd.normalized()
    if right.length_squared() > 0.0001: right = right.normalized()
    var horiz := Vector3.ZERO
    if Input.is_key_pressed(KEY_W):     horiz += fwd
    if Input.is_key_pressed(KEY_S):     horiz -= fwd
    if Input.is_key_pressed(KEY_A):     horiz -= right
    if Input.is_key_pressed(KEY_D):     horiz += right
    var spd := rig_speed
    if Input.is_key_pressed(KEY_SHIFT): spd *= fast_multiplier
    if horiz.length_squared() > 0.0001:
        horiz = horiz.normalized()
        velocity.x = horiz.x * spd
        velocity.z = horiz.z * spd
    else:
        velocity.x = 0.0
        velocity.z = 0.0

    # In 3P: rotate the rig to face the movement direction (character turns).
    if _third_person and horiz.length_squared() > 0.0001:
        var target_yaw := atan2(horiz.x, horiz.z)
        rotation.y = lerp_angle(rotation.y, target_yaw, clamp(delta * 10.0, 0.0, 1.0))

    # 2) Gravity / jump
    if is_on_floor():
        if velocity.y < 0.0:
            velocity.y = 0.0
        if Input.is_key_pressed(KEY_SPACE):
            velocity.y = jump_velocity_mps
    else:
        velocity.y -= gravity_mps2 * delta

    # 3) Sweep capsule against world geometry
    move_and_slide()

    # Step-up assist
    if is_on_floor() and (velocity.x * velocity.x + velocity.z * velocity.z) > 0.0001:
        var blocked := false
        for i in get_slide_collision_count():
            if absf(get_slide_collision(i).get_normal().y) < 0.3:
                blocked = true
                break
        if blocked:
            var hstep := Vector3(velocity.x, 0.0, velocity.z) * delta
            var up := Vector3(0.0, max_step_m, 0.0)
            if not test_move(global_transform, up) \
               and not test_move(Transform3D(global_transform.basis, global_transform.origin + up), hstep):
                global_position += up

    # 4) Look rotation (arrow keys) — only yaw+pitch, no roll
    var dyaw := 0.0
    var dpitch := 0.0
    if Input.is_key_pressed(KEY_LEFT):  dyaw += 1.0
    if Input.is_key_pressed(KEY_RIGHT): dyaw -= 1.0
    if Input.is_key_pressed(KEY_UP):    dpitch += 1.0
    if Input.is_key_pressed(KEY_DOWN):  dpitch -= 1.0
    if dpitch != 0.0:
        rotate_object_local(Vector3.RIGHT, dpitch * keyboard_look_speed * delta)
    if dyaw != 0.0:
        rotate(Vector3.UP, dyaw * keyboard_look_speed * delta)


func _apply_rotation_keys(delta: float) -> void:
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


func _set_capsule_height(h: float) -> void:
    var cap := _body_shape.shape as CapsuleShape3D
    var old_h: float = cap.height
    cap.height = h
    _body_shape.position.y = -(eye_height_m - h * 0.5)
    global_position.y += (h - old_h) * 0.5


func _sync_stereo() -> void:
    if _left_cam == null or _right_cam == null:
        return
    var xf := global_transform
    var offset := Vector3(stereo_baseline * 0.5, 0.0, 0.0)
    _left_cam.global_transform = xf * Transform3D(Basis.IDENTITY, -offset)
    _right_cam.global_transform = xf * Transform3D(Basis.IDENTITY, offset)
