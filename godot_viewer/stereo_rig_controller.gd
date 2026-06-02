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
# Physics-mode (1P) parameters
@export var gravity_mps2: float = 9.81
@export var jump_velocity_mps: float = 4.5
@export var collision_clearance_m: float = 0.01   # tiny gap to avoid sticking to surfaces
@export var ground_probe_m: float = 0.10          # how far below to look for ground

var _left_cam: Camera3D = null
var _right_cam: Camera3D = null
var _visuals: Array[Node3D] = []
var _physics_mode: bool = false       # toggled by camera_controller on view_mode change
var _vertical_velocity: float = 0.0   # for gravity + jump in 1P

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
    for name in ["LeftEyeMarker", "RightEyeMarker"]:
        if has_node(name):
            _visuals.append(get_node(name))
    # Cache pivots for the walk cycle. They were named in HumanoidVisual.build().
    _left_arm  = humanoid.get_node_or_null("LeftArm")
    _right_arm = humanoid.get_node_or_null("RightArm")
    _left_leg  = humanoid.get_node_or_null("LeftLeg")
    _right_leg = humanoid.get_node_or_null("RightLeg")


@export var spawn_y_m: float = 1.0   # rig centre height above world origin; safe
                                     # default so 1P physics doesn't dunk us
                                     # into a floor voxel at y=0.

func reset_pose() -> void:
    global_transform = Transform3D(Basis.IDENTITY, Vector3(0.0, spawn_y_m, 0.0))
    _vertical_velocity = 0.0


func set_pose(xf: Transform3D) -> void:
    global_transform = xf
    _vertical_velocity = 0.0


func get_pose() -> Transform3D:
    return global_transform


# Called by camera_controller when the view mode toggles. When ON, the rig
# obeys gravity and is blocked by voxel collision in WASD movement.
func set_physics_mode(enabled: bool) -> void:
    _physics_mode = enabled
    _vertical_velocity = 0.0


func set_visuals_visible(v: bool) -> void:
    for n in _visuals:
        n.visible = v


func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed and event.keycode == KEY_R:
        reset_pose()


func _process(delta: float) -> void:
    if _physics_mode:
        _apply_keyboard_physics(delta)
    else:
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
# gravity pulls down, Space jumps when grounded, raycasts block wall collision.
func _apply_keyboard_physics(delta: float) -> void:
    # 1) Horizontal movement (project rig basis onto Y=0 plane)
    var fwd: Vector3 = -transform.basis.z
    var right: Vector3 = transform.basis.x
    fwd.y = 0.0
    right.y = 0.0
    if fwd.length_squared() > 0.0001: fwd = fwd.normalized()
    if right.length_squared() > 0.0001: right = right.normalized()
    var horizontal := Vector3.ZERO
    if Input.is_key_pressed(KEY_W):     horizontal += fwd
    if Input.is_key_pressed(KEY_S):     horizontal -= fwd
    if Input.is_key_pressed(KEY_A):     horizontal -= right
    if Input.is_key_pressed(KEY_D):     horizontal += right
    var spd := rig_speed
    if Input.is_key_pressed(KEY_SHIFT): spd *= fast_multiplier
    var horizontal_step: Vector3 = horizontal.normalized() * spd * delta if horizontal.length_squared() > 0.0001 else Vector3.ZERO

    # 2) Gravity / jump — gate on grounded state so velocity doesn't accumulate
    #    while we're resting on the floor (was the source of the post-landing jitter).
    var grounded := _is_grounded()
    if grounded and _vertical_velocity <= 0.0:
        _vertical_velocity = 0.0   # at rest on ground; don't add gravity
        if Input.is_key_pressed(KEY_SPACE):
            _vertical_velocity = jump_velocity_mps
    else:
        _vertical_velocity -= gravity_mps2 * delta
    var vertical_step := Vector3(0.0, _vertical_velocity * delta, 0.0)

    # 3) Move with collision resolution (one axis at a time so we slide along walls)
    var new_pos := global_position
    new_pos = _slide_axis(new_pos, Vector3(horizontal_step.x, 0.0, 0.0))
    new_pos = _slide_axis(new_pos, Vector3(0.0, 0.0, horizontal_step.z))
    var before_vertical := new_pos
    new_pos = _slide_axis(new_pos, vertical_step)
    # Ceiling bonk: if going up and the actual move is short of requested, reset velocity.
    if vertical_step.y > 0.0 and (new_pos.y - before_vertical.y) < vertical_step.y - 0.0001:
        _vertical_velocity = 0.0

    global_position = new_pos

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


# Raycast from current position toward (pos + offset). If hit, clamp the offset
# so we stop just before the geometry. Returns the new position.
func _slide_axis(from: Vector3, offset: Vector3) -> Vector3:
    if offset.length_squared() < 1e-12:
        return from
    var space := get_world_3d().direct_space_state
    var query := PhysicsRayQueryParameters3D.create(from, from + offset)
    var result := space.intersect_ray(query)
    if result.is_empty():
        return from + offset
    var hit_pos: Vector3 = result.position
    var back: Vector3 = -offset.normalized() * collision_clearance_m
    return hit_pos + back


func _is_grounded() -> bool:
    var space := get_world_3d().direct_space_state
    var query := PhysicsRayQueryParameters3D.create(
        global_position,
        global_position + Vector3(0.0, -(collision_clearance_m + ground_probe_m), 0.0)
    )
    var result := space.intersect_ray(query)
    return not result.is_empty()


func _sync_stereo() -> void:
    if _left_cam == null or _right_cam == null:
        return
    var xf := global_transform
    var offset := Vector3(stereo_baseline * 0.5, 0.0, 0.0)
    _left_cam.global_transform = xf * Transform3D(Basis.IDENTITY, -offset)
    _right_cam.global_transform = xf * Transform3D(Basis.IDENTITY, offset)
