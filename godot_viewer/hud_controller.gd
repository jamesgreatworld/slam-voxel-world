# hud_controller.gd — Layer 6 (UI).
# Updates the HUD labels each frame from the current rig + camera state.
# Reads only — does not modify rig or camera. Pure projection of state to text.

extends Node

var _status_label: Label = null
var _mode_label: Label = null
var _pose_label: Label = null
var _rig_ctl: Node3D = null
var _cam_ctl: Node = null
var _world = null
var _logger = null


func init_controller(
    status_label: Label,
    mode_label: Label,
    pose_label: Label,
    world,
    rig_ctl: Node3D,
    cam_ctl: Node,
    logger = null
) -> void:
    _status_label = status_label
    _mode_label = mode_label
    _pose_label = pose_label
    _world = world
    _rig_ctl = rig_ctl
    _cam_ctl = cam_ctl
    _logger = logger
    if _status_label != null and world != null:
        _status_label.text = "vxw_viewer | %d voxels | voxel %.2fm | Tab=1P/3P  R=reset rig  Esc=quit  |  KB always drives RIG  |  (3P: mouse=orbit  wheel=zoom)" % [world.voxel_count(), world.voxel_size_meters]


func _process(_delta: float) -> void:
    if _rig_ctl == null or _cam_ctl == null:
        return
    var rig_xf: Transform3D = _rig_ctl.global_transform
    if _pose_label != null:
        var p: Vector3 = rig_xf.origin
        var e: Vector3 = rig_xf.basis.get_euler()
        _pose_label.text = "rig pose  pos (%+7.2f, %+7.2f, %+7.2f) m   yaw=%+6.1f° pitch=%+6.1f° roll=%+6.1f°   baseline=%.2fm" % [
            p.x, p.y, p.z,
            rad_to_deg(e.y), rad_to_deg(e.x), rad_to_deg(e.z),
            _rig_ctl.stereo_baseline,
        ]
    if _mode_label != null:
        var view_text: String
        # 1 = FIRST_PERSON (matches CameraController.ViewMode enum order)
        if _cam_ctl.get_view_mode() == 1:
            view_text = "FIRST-PERSON (camera on rig)"
        else:
            var ob: Dictionary = _cam_ctl.get_orbit_state()
            view_text = "THIRD-PERSON (orbit dist=%.1fm  yaw=%+5.1f°  pitch=%+5.1f°)" % [
                ob["distance"], rad_to_deg(ob["yaw"]), rad_to_deg(ob["pitch"]),
            ]
        _mode_label.text = "[Tab to switch]  view → %s" % view_text

    if _logger != null:
        _logger.log_pose_throttled("rig_pose", rig_xf)
