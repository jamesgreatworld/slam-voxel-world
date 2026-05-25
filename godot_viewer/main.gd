# main.gd — entry point.
# Loads a .vxw world, renders voxels via MultiMeshInstance3D.
# Hosts two control targets:
#   - INSPECTOR camera (free-fly observer)
#   - RIG (red box) = the in-scene stereo camera body; PiPs render from its two "eyes"
# Tab toggles which one keyboard drives. R resets rig pose to map origin.

extends Node3D

const VxwLoader = preload("res://vxw_loader.gd")
const VxwLogger = preload("res://logger.gd")
const DEFAULT_WORLD_PATH := "../out/baseline.vxw"

var logger  # VxwLog instance, untyped to avoid class_name registration order issues

@export var stereo_baseline: float = 0.10        # meters between L/R eye centres
@export var rig_speed: float = 2.0               # m/s — keyboard always drives the rig
@export var fast_multiplier: float = 4.0
@export var keyboard_look_speed: float = 1.5     # rad/s yaw + pitch
@export var keyboard_roll_speed: float = 1.2

# Third-person orbit camera around the rig.
@export var orbit_distance_init: float = 8.0
@export var orbit_yaw_init: float = 0.0          # rad
@export var orbit_pitch_init: float = 0.35       # rad (~20° above horizon)
@export var mouse_sensitivity: float = 0.005
@export var zoom_step: float = 1.0

var orbit_distance: float
var orbit_yaw: float
var orbit_pitch: float

enum ViewMode { THIRD_PERSON, FIRST_PERSON }
var view_mode: ViewMode = ViewMode.THIRD_PERSON

# Snapshot mode (for AI / CI verification — no GUI needed).
#   pass `--snapshot=path.png` on the command line to enable.
#   the engine renders SNAPSHOT_WARMUP_FRAMES frames, then saves PNG and quits.
const SNAPSHOT_WARMUP_FRAMES := 20
var _snapshot_path: String = ""
var _snapshot_frames_left: int = -1

@onready var mmi: MultiMeshInstance3D = $Voxels
@onready var main_cam: Camera3D = $Camera3D
@onready var stereo_rig: Node3D = $StereoRig
@onready var rig_red_box: MeshInstance3D = $StereoRig/RedBox
@onready var rig_eye_l: MeshInstance3D = $StereoRig/LeftEyeMarker
@onready var rig_eye_r: MeshInstance3D = $StereoRig/RightEyeMarker
@onready var hud: Label = $HUD/StatusLabel
@onready var mode_label: Label = $HUD/ModeLabel
@onready var pose_label: Label = $HUD/PoseLabel
@onready var left_cam: Camera3D = $HUD/LeftStereoContainer/LeftStereoViewport/LeftStereoCamera
@onready var right_cam: Camera3D = $HUD/RightStereoContainer/RightStereoViewport/RightStereoCamera
@onready var left_vp: SubViewport = $HUD/LeftStereoContainer/LeftStereoViewport
@onready var right_vp: SubViewport = $HUD/RightStereoContainer/RightStereoViewport


func _ready() -> void:
    logger = VxwLogger.new()
    orbit_distance = orbit_distance_init
    orbit_yaw = orbit_yaw_init
    orbit_pitch = orbit_pitch_init
    _apply_mouse_mode()  # default = 3P → captured

    var world_path := DEFAULT_WORLD_PATH
    for arg in OS.get_cmdline_user_args():
        if arg.begins_with("--world="):
            world_path = arg.substr("--world=".length())
        elif arg.begins_with("--snapshot="):
            _snapshot_path = arg.substr("--snapshot=".length())
            _snapshot_frames_left = SNAPSHOT_WARMUP_FRAMES
        elif arg == "--view=1p":
            view_mode = ViewMode.FIRST_PERSON
        elif arg == "--view=3p":
            view_mode = ViewMode.THIRD_PERSON
        elif arg.begins_with("--rig-pose="):
            _parse_rig_pose(arg.substr("--rig-pose=".length()))

    logger.info("config", {
        "world_path": world_path,
        "view_mode": "3P" if view_mode == ViewMode.THIRD_PERSON else "1P",
        "stereo_baseline_m": stereo_baseline,
        "rig_speed_mps": rig_speed,
        "snapshot": _snapshot_path,
    })

    var world = VxwLoader.load_world(_resolve(world_path))
    if world.voxel_count() == 0:
        hud.text = "No voxels loaded.  Check console for errors.\nPath tried: " + world_path
        logger.error("world", {"reason": "empty load", "path": world_path})
        return

    logger.info("world_loaded", {
        "chunks": world.chunks.size() if "chunks" in world else -1,
        "voxels": world.voxel_count(),
        "voxel_size_m": world.voxel_size_meters,
        "path": world_path,
    })

    _build_multimesh(world)
    _add_reference_helpers(world)
    left_vp.world_3d = get_viewport().world_3d
    right_vp.world_3d = get_viewport().world_3d
    stereo_rig.global_transform = Transform3D.IDENTITY
    hud.text = "vxw_viewer | %d voxels | voxel %.2fm | Tab=1P/3P  R=reset rig  Esc=quit  |  KB always drives RIG  |  (3P: mouse=orbit  wheel=zoom)" % [
        world.voxel_count(),
        world.voxel_size_meters,
    ]


func _resolve(p: String) -> String:
    if p.is_absolute_path():
        return p
    var base := ProjectSettings.globalize_path("res://")
    return base.path_join(p)


func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed:
        if event.keycode == KEY_TAB:
            view_mode = (
                ViewMode.FIRST_PERSON if view_mode == ViewMode.THIRD_PERSON else ViewMode.THIRD_PERSON
            )
            _apply_mouse_mode()
            logger.info("view_mode", {"now": "1P" if view_mode == ViewMode.FIRST_PERSON else "3P"})
        elif event.keycode == KEY_R:
            stereo_rig.global_transform = Transform3D.IDENTITY
            logger.info("rig_reset", "")
        elif event.keycode == KEY_ESCAPE:
            logger.info("session_end", {"reason": "esc"})
            get_tree().quit()
        return

    # Mouse interactions only matter in 3rd-person (orbit camera around rig).
    if view_mode != ViewMode.THIRD_PERSON:
        return
    if event is InputEventMouseMotion:
        orbit_yaw -= event.relative.x * mouse_sensitivity
        orbit_pitch -= event.relative.y * mouse_sensitivity
        orbit_pitch = clamp(orbit_pitch, -PI / 2 + 0.1, PI / 2 - 0.1)
    elif event is InputEventMouseButton and event.pressed:
        if event.button_index == MOUSE_BUTTON_WHEEL_UP:
            orbit_distance = max(0.5, orbit_distance - zoom_step)
        elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
            orbit_distance += zoom_step


func _apply_mouse_mode() -> void:
    # 3P: capture mouse for orbit. 1P: hide (no mouse interaction).
    if view_mode == ViewMode.THIRD_PERSON:
        Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
    else:
        Input.mouse_mode = Input.MOUSE_MODE_HIDDEN


func _parse_rig_pose(spec: String) -> void:
    # Format: x,y,z,yaw_deg,pitch_deg,roll_deg
    var parts := spec.split(",")
    if parts.size() != 6:
        push_error("[snapshot] --rig-pose needs 6 comma-separated numbers, got %d" % parts.size())
        return
    var p := Vector3(float(parts[0]), float(parts[1]), float(parts[2]))
    var basis := Basis()
    basis = basis.rotated(Vector3.UP, deg_to_rad(float(parts[3])))      # yaw
    basis = basis.rotated(basis.x, deg_to_rad(float(parts[4])))         # pitch (local X)
    basis = basis.rotated(basis.z, deg_to_rad(float(parts[5])))         # roll (local Z)
    # Defer to after _ready() so stereo_rig is @onready'd:
    call_deferred("_set_rig_xform", Transform3D(basis, p))


func _set_rig_xform(xf: Transform3D) -> void:
    stereo_rig.global_transform = xf


func _take_snapshot() -> void:
    var image := get_viewport().get_texture().get_image()
    var dir := _snapshot_path.get_base_dir()
    if dir != "" and not DirAccess.dir_exists_absolute(dir):
        DirAccess.make_dir_recursive_absolute(dir)
    var err := image.save_png(_snapshot_path)
    var rig_xf := stereo_rig.global_transform
    var p := rig_xf.origin
    var e := rig_xf.basis.get_euler()
    logger.info("snapshot", {
        "path": _snapshot_path,
        "size": [image.get_width(), image.get_height()],
        "ok": err == OK,
        "view": "1P" if view_mode == ViewMode.FIRST_PERSON else "3P",
        "rig_pos_m": [p.x, p.y, p.z],
        "rig_rot_deg": [rad_to_deg(e.y), rad_to_deg(e.x), rad_to_deg(e.z)],
    })
    get_tree().quit()


func _process(delta: float) -> void:
    # 1) Keyboard always drives the RIG
    _apply_keyboard(stereo_rig, rig_speed, delta)

    # 2) Main camera follows RIG, layout depends on view mode
    var rig_xf: Transform3D = stereo_rig.global_transform
    if view_mode == ViewMode.FIRST_PERSON:
        # Camera sits at rig centre, sees what's in front of the rig
        main_cam.global_transform = rig_xf
        rig_red_box.visible = false
        rig_eye_l.visible = false
        rig_eye_r.visible = false
    else:
        # Orbit camera: spherical around rig position, controlled by mouse.
        var offset := Vector3(
            cos(orbit_pitch) * sin(orbit_yaw),
            sin(orbit_pitch),
            cos(orbit_pitch) * cos(orbit_yaw)
        ) * orbit_distance
        main_cam.global_position = rig_xf.origin + offset
        main_cam.look_at(rig_xf.origin, Vector3.UP)
        rig_red_box.visible = true
        rig_eye_l.visible = true
        rig_eye_r.visible = true

    # 3) Sync stereo SubViewport cams to RIG (independent of main view mode)
    var stereo_offset := Vector3(stereo_baseline * 0.5, 0.0, 0.0)
    left_cam.global_transform = rig_xf * Transform3D(Basis.IDENTITY, -stereo_offset)
    right_cam.global_transform = rig_xf * Transform3D(Basis.IDENTITY, stereo_offset)

    # 4) Update labels
    var p: Vector3 = rig_xf.origin
    var e: Vector3 = rig_xf.basis.get_euler()  # Godot default: YXZ Euler
    pose_label.text = "rig pose  pos (%+7.2f, %+7.2f, %+7.2f) m   yaw=%+6.1f° pitch=%+6.1f° roll=%+6.1f°   baseline=%.2fm" % [
        p.x, p.y, p.z,
        rad_to_deg(e.y), rad_to_deg(e.x), rad_to_deg(e.z),
        stereo_baseline,
    ]
    var mode_text: String
    if view_mode == ViewMode.FIRST_PERSON:
        mode_text = "FIRST-PERSON (camera on rig)"
    else:
        mode_text = "THIRD-PERSON (orbit dist=%.1fm  yaw=%+5.1f°  pitch=%+5.1f°)" % [
            orbit_distance, rad_to_deg(orbit_yaw), rad_to_deg(orbit_pitch),
        ]
    mode_label.text = "[Tab to switch]  view → %s" % mode_text

    # 5) Periodic rig pose log (throttled inside)
    logger.log_pose_throttled("rig_pose", rig_xf)

    # 6) Snapshot mode: wait warmup frames, then capture and quit
    if _snapshot_frames_left > 0:
        _snapshot_frames_left -= 1
    elif _snapshot_frames_left == 0:
        _snapshot_frames_left = -1
        _take_snapshot()


func _apply_keyboard(target: Node3D, spd: float, delta: float) -> void:
    var dir := Vector3.ZERO
    if Input.is_key_pressed(KEY_W):     dir -= target.transform.basis.z
    if Input.is_key_pressed(KEY_S):     dir += target.transform.basis.z
    if Input.is_key_pressed(KEY_A):     dir -= target.transform.basis.x
    if Input.is_key_pressed(KEY_D):     dir += target.transform.basis.x
    if Input.is_key_pressed(KEY_SPACE): dir += Vector3.UP
    if Input.is_key_pressed(KEY_CTRL):  dir -= Vector3.UP
    var v := spd
    if Input.is_key_pressed(KEY_SHIFT): v *= fast_multiplier
    if dir != Vector3.ZERO:
        target.translate(dir.normalized() * v * delta)

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
        target.rotate_object_local(Vector3.RIGHT, dpitch * keyboard_look_speed * delta)
    if dyaw != 0.0:
        target.rotate_object_local(Vector3.UP, dyaw * keyboard_look_speed * delta)
    if droll != 0.0:
        target.rotate_object_local(Vector3.FORWARD, droll * keyboard_roll_speed * delta)


func _build_multimesh(world) -> void:
    var box := BoxMesh.new()
    box.size = Vector3.ONE * world.voxel_size_meters

    var mm := MultiMesh.new()
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.use_colors = true
    mm.mesh = box
    mm.instance_count = world.voxel_count()

    var y_min: float = INF
    var y_max: float = -INF
    for i in world.voxel_count():
        var py: float = world.positions[i].y
        y_min = min(y_min, py)
        y_max = max(y_max, py)
    var y_span: float = max(0.01, y_max - y_min)

    for i in world.voxel_count():
        var p: Vector3 = world.positions[i]
        var t := Transform3D(Basis.IDENTITY, p)
        mm.set_instance_transform(i, t)
        var h: float = (p.y - y_min) / y_span
        var c: Color = Color(h, 1.0 - abs(h - 0.5) * 2.0, 1.0 - h, 1.0) * 0.9 + Color(0.1, 0.1, 0.1, 0)
        mm.set_instance_color(i, c)

    var mat := StandardMaterial3D.new()
    mat.vertex_color_use_as_albedo = true
    mat.roughness = 0.75
    mat.metallic = 0.0
    mmi.material_override = mat
    mmi.multimesh = mm


func _add_reference_helpers(world) -> void:
    var plane_y: float = INF
    for i in world.voxel_count():
        plane_y = min(plane_y, world.positions[i].y)
    var plane := MeshInstance3D.new()
    var pm := PlaneMesh.new()
    pm.size = Vector2(500, 500)
    plane.mesh = pm
    var pmat := StandardMaterial3D.new()
    pmat.albedo_color = Color(0.2, 0.22, 0.25, 1.0)
    pmat.roughness = 1.0
    plane.material_override = pmat
    plane.position = Vector3(0, plane_y - 0.5, 0)
    add_child(plane)

    var axis_dirs: Array[Vector3] = [Vector3.RIGHT, Vector3.UP, Vector3.FORWARD]
    var axis_cols: Array[Color] = [Color.RED, Color.GREEN, Color.BLUE]
    for axis_idx in 3:
        var im := ImmediateMesh.new()
        var mi := MeshInstance3D.new()
        mi.mesh = im
        var lmat := StandardMaterial3D.new()
        lmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
        lmat.albedo_color = axis_cols[axis_idx]
        im.surface_begin(Mesh.PRIMITIVE_LINES, lmat)
        im.surface_add_vertex(Vector3.ZERO)
        im.surface_add_vertex(axis_dirs[axis_idx] * 5.0)
        im.surface_end()
        add_child(mi)
