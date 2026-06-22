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
# Physics-mode (1P) parameters
@export var gravity_mps2: float = 9.81
@export var jump_velocity_mps: float = 4.5
@export var eye_height_m: float = 1.6
@export var capsule_radius_m: float = 0.3
@export var capsule_stand_h_m: float = 1.7
@export var floor_max_angle_deg: float = 46.0
@export var max_step_m: float = 0.35
@export var capsule_crouch_h_m: float = 0.9

var _crouched: bool = false

var _left_cam: Camera3D = null
var _right_cam: Camera3D = null
var _visuals: Array[Node3D] = []
var _physics_mode: bool = false       # toggled by camera_controller on view_mode change
var _body_shape: CollisionShape3D = null

# Walk-cycle animation state. _walk_cycle advances when the rig is moving
# horizontally (any view mode); when it stops, it lerps back toward 0 so the
# limbs settle to neutral. The four pivots are cached from the humanoid
# after init_controller builds it.
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

# A2 sit/lay state. When _seated_entity_id != "" the rig is parked on top
# of an entity and the WASD / mouse-look paths skip movement.
var _seated_entity_id: String = ""
const _SIT_LIFT_ABOVE_ENTITY_CENTRE: float = 0.45  # rig waist sits above seat
const _SIT_LEG_PITCH: float = PI * 0.5             # legs out ~90°

# One-shot physics self-check counter (headless smoke test).
# Counts up to 90 physics frames in 1P mode, then prints settled state.
var _phys_check_frame: int = 0


const HumanoidVisualScript = preload("res://humanoid_visual.gd")


func init_controller(left_cam: Camera3D, right_cam: Camera3D) -> void:
    _left_cam = left_cam
    _right_cam = right_cam
    # Replace the red sensor box with a block-style humanoid. Hide the
    # original RedBox so we don't see both. Eye markers stay (they show
    # the stereo baseline).
    if has_node("RedBox"):
        get_node("RedBox").visible = false
    var humanoid: Node3D = HumanoidVisualScript.build()
    add_child(humanoid)
    _visuals = [humanoid]
    for n in ["LeftEyeMarker", "RightEyeMarker"]:
        if has_node(n):
            _visuals.append(get_node(n))
    # Cache pivots for the walk cycle. They were named in HumanoidVisual.build().
    _left_arm  = humanoid.get_node_or_null("LeftArm")
    _right_arm = humanoid.get_node_or_null("RightArm")
    _left_leg  = humanoid.get_node_or_null("LeftLeg")
    _right_leg = humanoid.get_node_or_null("RightLeg")

    # Build capsule collision shape. The rig origin sits at eye height,
    # so we shift the capsule down so its bottom aligns with the feet.
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


@export var spawn_y_m: float = 1.0   # rig origin (eye) height above world origin; safe
                                     # default so 1P physics doesn't dunk us
                                     # into a floor voxel at y=0.

func reset_pose() -> void:
    global_transform = Transform3D(Basis.IDENTITY, Vector3(0.0, spawn_y_m, 0.0))
    velocity = Vector3.ZERO


func set_pose(xf: Transform3D) -> void:
    global_transform = xf
    velocity = Vector3.ZERO


func get_pose() -> Transform3D:
    return global_transform


# ---------------------------------------------------------------------------
# A2 sit: park the rig on top of an entity. The entity supplies the chair's
# world-space centre + yaw; we lift the waist by _SIT_LIFT and bend the legs
# 90° forward so the humanoid reads as "sitting". WASD is gated on
# is_seated() in _apply_keyboard_*.
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


# Called by camera_controller when the view mode toggles. When ON, the rig
# obeys gravity and is blocked by voxel collision in WASD movement.
func set_physics_mode(enabled: bool) -> void:
    _physics_mode = enabled
    velocity = Vector3.ZERO
    _phys_check_frame = 0


func set_visuals_visible(v: bool) -> void:
    for n in _visuals:
        n.visible = v


func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed and event.keycode == KEY_R:
        reset_pose()


func _physics_process(delta: float) -> void:
    if _physics_mode and not is_seated():
        _apply_keyboard_physics(delta)
        _sync_stereo()

        # one-shot physics self-check (headless smoke)
        if _phys_check_frame >= 0:
            _phys_check_frame += 1
            if _phys_check_frame == 90:
                print("[rig] 1P self-check y=%.2f on_floor=%s vel_y=%.2f" % [global_position.y, str(is_on_floor()), velocity.y])
                _phys_check_frame = -1


func _process(delta: float) -> void:
    if is_seated():
        # Sit mode: rig stays glued to the entity; ignore movement keys.
        _sync_stereo()
        return
    if not _physics_mode:
        _apply_keyboard_freefly(delta)
        _sync_stereo()
    _animate_walk(delta)


# Advance the walk cycle when the rig is moving horizontally and apply
# sin-driven swing to the four limb pivots. Opposite limbs swing in
# opposition (left leg forward → right arm forward), and arms swing
# counter-phase to the legs for a natural gait.
func _animate_walk(delta: float) -> void:
    if _left_arm == null:
        return    # humanoid wasn't built yet
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
    # Cycle advances proportionally to speed (faster walk = faster swing)
    # but capped so sprint doesn't blur the limbs.
    var cycle_rate: float = min(target_speed, 4.0) * _WALK_FREQ_HZ
    _walk_cycle = fmod(_walk_cycle + cycle_rate * delta, TAU)

    var swing := sin(_walk_cycle) * _WALK_AMPL_RAD
    if not moving:
        # When standing, decay the visible swing toward 0 even though the
        # internal cycle keeps its phase — avoids snapping mid-step.
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


# Free-fly mode (3P): unchanged behaviour, ignores gravity & collision.
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


# Physics mode (1P): horizontal-only WASD on the rig's yaw plane,
# gravity pulls down, Space jumps when grounded, move_and_slide handles collision.
func _apply_keyboard_physics(delta: float) -> void:
    # 0) Crouch toggle (Ctrl) — gate stand-up on headroom clearance
    var want_crouch := Input.is_key_pressed(KEY_CTRL)
    if want_crouch and not _crouched:
        _set_capsule_height(capsule_crouch_h_m); _crouched = true
    elif not want_crouch and _crouched:
        var dh := capsule_stand_h_m - capsule_crouch_h_m
        if not test_move(global_transform, Vector3(0.0, dh, 0.0)):   # headroom?
            _set_capsule_height(capsule_stand_h_m); _crouched = false

    # 1) Horizontal movement (project rig basis onto Y=0 plane)
    var fwd: Vector3 = -transform.basis.z
    var right: Vector3 = transform.basis.x
    fwd.y = 0.0
    right.y = 0.0
    if fwd.length_squared() > 0.0001: fwd = fwd.normalized()
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

    # Step-up assist: if grounded and a near-vertical obstacle blocked horizontal
    # motion, and the same horizontal move is clear when raised by max_step_m, lift up.
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

    # 4) Look rotation: only yaw + pitch (no roll in 1P walking)
    var dyaw := 0.0
    var dpitch := 0.0
    if Input.is_key_pressed(KEY_LEFT):  dyaw += 1.0
    if Input.is_key_pressed(KEY_RIGHT): dyaw -= 1.0
    if Input.is_key_pressed(KEY_UP):    dpitch += 1.0
    if Input.is_key_pressed(KEY_DOWN):  dpitch -= 1.0
    if dpitch != 0.0:
        rotate_object_local(Vector3.RIGHT, dpitch * keyboard_look_speed * delta)
    if dyaw != 0.0:
        rotate(Vector3.UP, dyaw * keyboard_look_speed * delta)  # world-yaw stays upright


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


# Resize the body capsule keeping the FEET fixed; raise/lower the rig origin (eye)
# by half the height delta so the camera tracks the head.
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
