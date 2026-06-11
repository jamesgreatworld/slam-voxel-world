# boot_config.gd — parses CLI user args into a plain config object.
# Pure data + parsing: no side effects, no scene access. main.gd builds one
# at boot; test_hooks_controller.gd consumes the test_* fields.
extends RefCounted

var raw_args: PackedStringArray = PackedStringArray()

var world_path: String = "../out/baseline.vxw"
var view_override: String = ""        # "" | "1p" | "3p"
var rig_pose_spec: String = ""        # "x,y,z,yaw,pitch,roll"
var open_item_picker: bool = false
var spawn_items: Array = []

var test_delete_first: bool = false
var test_rotate_first_deg: float = 0.0
var test_grab_first_to: Vector3 = Vector3.ZERO
var test_grab_first_set: bool = false
var test_undo_times: int = 0
var test_duplicate_first: bool = false
var test_snapshot_world: bool = false
var test_toggle_behavior_on_first: bool = false
var test_hide_voxel_set: bool = false
var test_hide_voxel_vi: Vector3i = Vector3i.ZERO
var test_toggle_day_night: bool = false


func parse_args(args: PackedStringArray) -> void:
    raw_args = args
    for arg in args:
        if arg.begins_with("--world="):
            world_path = arg.substr("--world=".length())
        elif arg == "--view=1p":
            view_override = "1p"
        elif arg == "--view=3p":
            view_override = "3p"
        elif arg.begins_with("--rig-pose="):
            rig_pose_spec = arg.substr("--rig-pose=".length())
        elif arg == "--open-item-picker":
            open_item_picker = true
        elif arg.begins_with("--spawn-items="):
            spawn_items = arg.substr("--spawn-items=".length()).split(",")
        elif arg == "--test-delete-first":
            test_delete_first = true
        elif arg.begins_with("--test-rotate-first="):
            test_rotate_first_deg = float(arg.substr("--test-rotate-first=".length()))
        elif arg.begins_with("--test-grab-first-to="):
            var parts := arg.substr("--test-grab-first-to=".length()).split(",")
            if parts.size() == 3:
                test_grab_first_to = Vector3(
                    float(parts[0]), float(parts[1]), float(parts[2])
                )
                test_grab_first_set = true
        elif arg.begins_with("--test-undo-times="):
            test_undo_times = int(arg.substr("--test-undo-times=".length()))
        elif arg == "--test-duplicate-first":
            test_duplicate_first = true
        elif arg == "--test-snapshot-world":
            test_snapshot_world = true
        elif arg == "--test-toggle-behavior-on-first":
            test_toggle_behavior_on_first = true
        elif arg.begins_with("--test-hide-voxel="):
            var hv_parts := arg.substr("--test-hide-voxel=".length()).split(",")
            if hv_parts.size() == 3:
                test_hide_voxel_vi = Vector3i(
                    int(hv_parts[0]), int(hv_parts[1]), int(hv_parts[2])
                )
                test_hide_voxel_set = true
        elif arg == "--test-toggle-day-night":
            test_toggle_day_night = true


static func parse_rig_pose(spec: String) -> Transform3D:
    var parts := spec.split(",")
    if parts.size() != 6:
        push_error("[boot_config] --rig-pose needs 6 comma-separated numbers, got %d" % parts.size())
        return Transform3D.IDENTITY
    var p := Vector3(float(parts[0]), float(parts[1]), float(parts[2]))
    var basis := Basis()
    basis = basis.rotated(Vector3.UP, deg_to_rad(float(parts[3])))     # yaw
    basis = basis.rotated(basis.x, deg_to_rad(float(parts[4])))        # pitch (local X)
    basis = basis.rotated(basis.z, deg_to_rad(float(parts[5])))        # roll (local Z)
    return Transform3D(basis, p)
