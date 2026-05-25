# logger.gd — structured log for the Godot viewer.
# Writes one event per line to out/logs/godot_<timestamp>.log next to the project,
# and mirrors to stdout. Format: "[Ts] LEVEL CATEGORY  json-payload".

class_name VxwLog extends RefCounted

var _file: FileAccess
var _path: String
var _t0_ms: int
var _last_pose_log_ms: int = 0
@export var pose_log_interval_ms: int = 500   # throttle rig-pose lines to 2 Hz


func _init(out_dir_abs: String = "") -> void:
    var ts := Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
    if out_dir_abs.is_empty():
        out_dir_abs = ProjectSettings.globalize_path("res://../out/logs")
    if not DirAccess.dir_exists_absolute(out_dir_abs):
        DirAccess.make_dir_recursive_absolute(out_dir_abs)
    _path = out_dir_abs + "/godot_" + ts + ".log"
    _file = FileAccess.open(_path, FileAccess.WRITE)
    if _file == null:
        push_error("[logger] cannot open " + _path)
    _t0_ms = Time.get_ticks_msec()
    info("session", {"log_path": _path, "godot_version": Engine.get_version_info().string})


func path() -> String:
    return _path


func info(category: String, data) -> void:
    _write("INFO", category, data)


func warn(category: String, data) -> void:
    _write("WARN", category, data)


func error(category: String, data) -> void:
    _write("ERROR", category, data)


func log_pose_throttled(category: String, xform: Transform3D) -> void:
    var now := Time.get_ticks_msec()
    if now - _last_pose_log_ms < pose_log_interval_ms:
        return
    _last_pose_log_ms = now
    var p: Vector3 = xform.origin
    var e: Vector3 = xform.basis.get_euler()
    info(category, {
        "pos_m":   [snapped(p.x, 0.001), snapped(p.y, 0.001), snapped(p.z, 0.001)],
        "rot_deg": [
            snapped(rad_to_deg(e.y), 0.1),
            snapped(rad_to_deg(e.x), 0.1),
            snapped(rad_to_deg(e.z), 0.1),
        ],
    })


func _write(level: String, category: String, data) -> void:
    var elapsed: float = (Time.get_ticks_msec() - _t0_ms) / 1000.0
    var payload: String
    if typeof(data) == TYPE_DICTIONARY or typeof(data) == TYPE_ARRAY:
        payload = JSON.stringify(data)
    else:
        payload = str(data)
    var line := "[%8.3fs] %-5s %-12s %s\n" % [elapsed, level, category, payload]
    if _file != null:
        _file.store_string(line)
        _file.flush()
    print(line.strip_edges())
