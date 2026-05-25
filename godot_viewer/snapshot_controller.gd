# snapshot_controller.gd — orchestration helper for AI / CI verification.
# When --snapshot=<path.png> is in cmdline-user-args, after SNAPSHOT_WARMUP_FRAMES
# frames the controller captures the main viewport and quits.
# Independent of rendering, camera, or UI logic — it only needs access to the
# rig + cam controllers for logging the captured pose.

extends Node

const SNAPSHOT_WARMUP_FRAMES := 20

var _snapshot_path: String = ""
var _frames_left: int = -1
var _rig_ctl: Node3D = null
var _cam_ctl: Node = null
var _logger = null


func init_controller(rig_ctl: Node3D, cam_ctl: Node, logger = null) -> void:
    _rig_ctl = rig_ctl
    _cam_ctl = cam_ctl
    _logger = logger


# Returns true if snapshot mode was enabled by cmdline.
func configure_from_cli(args: PackedStringArray) -> bool:
    for arg in args:
        if arg.begins_with("--snapshot="):
            _snapshot_path = arg.substr("--snapshot=".length())
            _frames_left = SNAPSHOT_WARMUP_FRAMES
            return true
    return false


func _process(_delta: float) -> void:
    if _frames_left > 0:
        _frames_left -= 1
    elif _frames_left == 0:
        _frames_left = -1
        _take_snapshot()


func _take_snapshot() -> void:
    var image := get_viewport().get_texture().get_image()
    var dir := _snapshot_path.get_base_dir()
    if dir != "" and not DirAccess.dir_exists_absolute(dir):
        DirAccess.make_dir_recursive_absolute(dir)
    var err := image.save_png(_snapshot_path)
    if _logger != null and _rig_ctl != null and _cam_ctl != null:
        var rig_xf: Transform3D = _rig_ctl.global_transform
        var p := rig_xf.origin
        var e := rig_xf.basis.get_euler()
        _logger.info("snapshot", {
            "path": _snapshot_path,
            "size": [image.get_width(), image.get_height()],
            "ok": err == OK,
            "view": "1P" if _cam_ctl.get_view_mode() == 1 else "3P",
            "rig_pos_m": [p.x, p.y, p.z],
            "rig_rot_deg": [rad_to_deg(e.y), rad_to_deg(e.x), rad_to_deg(e.z)],
        })
    get_tree().quit()
