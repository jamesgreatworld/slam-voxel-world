# _editor_selftest.gd — headless verification of voxel_editor.gd
#
# Builds a synthetic mini-world (three voxels along -Z), a real
# MultiMeshInstance3D, attaches the VoxelEditor as a child of the scene root,
# then exercises the picking math via try_destroy_along_ray to avoid relying
# on Camera3D.project_ray_* (which can be flaky under headless renderers).
#
# We do also briefly attempt a real Camera3D path for sanity but the
# pass/fail criterion is the ray-based path so this test is reliable.
#
# Run (note: NOT --headless — MultiMesh.get_instance_transform readback
# returns zeros under the dummy renderer, so we render to an offscreen window):
#   F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer ^
#       --position -10000,-10000 --resolution 800x600 --quit-after 30 ^
#       --script res://_editor_selftest.gd

extends SceneTree

const VoxelEditor = preload("res://voxel_editor.gd")
const _ZERO_BASIS := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)

# Minimal stand-in for VxwLoader.VxwWorld — voxel_editor.gd only reads
# .positions and .voxel_size_meters.
class FakeWorld extends RefCounted:
    var voxel_size_meters: float = 1.0
    var positions: PackedVector3Array = PackedVector3Array()


func _fail(msg: String) -> void:
    push_error("[EDITOR SELFTEST] FAIL: " + msg)
    quit(1)


func _initialize() -> void:
    var ok := _run()
    if ok:
        print("EDITOR SELFTEST OK")
        quit(0)
    else:
        quit(1)


func _run() -> bool:
    # ---- build synthetic world ----
    var world := FakeWorld.new()
    world.voxel_size_meters = 1.0
    world.positions = PackedVector3Array([
        Vector3(0, 0, -1),
        Vector3(0, 0, -2),
        Vector3(0, 0, -3),
    ])

    # ---- build a real MultiMeshInstance3D mirroring main.gd._build_multimesh ----
    var root := Node3D.new()
    get_root().add_child(root)

    var mmi := MultiMeshInstance3D.new()
    var mm := MultiMesh.new()
    var box := BoxMesh.new()
    box.size = Vector3.ONE * world.voxel_size_meters
    mm.mesh = box
    mm.transform_format = MultiMesh.TRANSFORM_3D
    mm.instance_count = world.positions.size()
    mmi.multimesh = mm
    root.add_child(mmi)
    for i in world.positions.size():
        mm.set_instance_transform(i, Transform3D(Basis.IDENTITY, world.positions[i]))

    # ---- camera (kept around so the API call exists; not relied on for asserts) ----
    var cam := Camera3D.new()
    cam.transform = Transform3D(Basis.IDENTITY, Vector3.ZERO)  # at origin, looking -Z (Godot default)
    root.add_child(cam)

    var editor := VoxelEditor.new()
    root.add_child(editor)
    editor.init_editor(world, mmi, cam)

    # ---- collect signal payloads ----
    var hits: Array = []
    editor.voxel_destroyed.connect(func(vi: Vector3i, wp: Vector3) -> void:
        hits.append({"vi": vi, "wp": wp})
    )

    # ---- TEST 1: ray from origin along -Z hits (0,0,-1) first ----
    var ok1: bool = editor.try_destroy_along_ray(Vector3.ZERO, Vector3(0, 0, -1), 50.0)
    if not ok1:
        _fail("first ray pick returned false (expected hit on (0,0,-1))")
        return false
    if hits.size() != 1:
        _fail("expected 1 signal emission, got %d" % hits.size())
        return false
    if hits[0]["vi"] != Vector3i(0, 0, -1):
        _fail("expected first hit at voxel index (0,0,-1), got %s" % str(hits[0]["vi"]))
        return false
    var wp1: Vector3 = hits[0]["wp"]
    if not wp1.is_equal_approx(Vector3(0, 0, -1)):
        _fail("expected first hit world pos (0,0,-1), got %s" % str(wp1))
        return false

    # ---- TEST 2: that instance is now hidden (basis is zero) ----
    var xf0: Transform3D = mmi.multimesh.get_instance_transform(0)
    if xf0.basis != _ZERO_BASIS:
        _fail("instance 0 should be hidden (zero basis), got basis=%s" % str(xf0.basis))
        return false
    # Other instances should still be at their original spots.
    var xf1: Transform3D = mmi.multimesh.get_instance_transform(1)
    if xf1.basis == _ZERO_BASIS:
        _fail("instance 1 was hidden but should still be visible")
        return false
    if not xf1.origin.is_equal_approx(Vector3(0, 0, -2)):
        _fail("instance 1 origin moved unexpectedly: %s" % str(xf1.origin))
        return false

    # ---- TEST 3: next pick falls through to (0,0,-2) ----
    var ok2: bool = editor.try_destroy_along_ray(Vector3.ZERO, Vector3(0, 0, -1), 50.0)
    if not ok2:
        _fail("second ray pick returned false (expected hit on (0,0,-2))")
        return false
    if hits.size() != 2:
        _fail("expected 2 signal emissions, got %d" % hits.size())
        return false
    if hits[1]["vi"] != Vector3i(0, 0, -2):
        _fail("expected second hit at (0,0,-2), got %s" % str(hits[1]["vi"]))
        return false
    var xf1b: Transform3D = mmi.multimesh.get_instance_transform(1)
    if xf1b.basis != _ZERO_BASIS:
        _fail("instance 1 should now be hidden after second pick")
        return false

    # ---- TEST 4: third pick destroys (0,0,-3); fourth should miss ----
    var ok3: bool = editor.try_destroy_along_ray(Vector3.ZERO, Vector3(0, 0, -1), 50.0)
    if not ok3:
        _fail("third pick returned false (expected hit on (0,0,-3))")
        return false
    if hits[2]["vi"] != Vector3i(0, 0, -3):
        _fail("expected third hit at (0,0,-3), got %s" % str(hits[2]["vi"]))
        return false
    var ok4: bool = editor.try_destroy_along_ray(Vector3.ZERO, Vector3(0, 0, -1), 50.0)
    if ok4:
        _fail("fourth pick should miss (all three voxels destroyed) but returned true")
        return false

    # ---- TEST 5: short-range ray should miss even when something is present ----
    # Rebuild a fresh editor + mmi to test max_distance honouring.
    var world2 := FakeWorld.new()
    world2.voxel_size_meters = 1.0
    world2.positions = PackedVector3Array([Vector3(0, 0, -10)])
    var mm2 := MultiMesh.new()
    var box2 := BoxMesh.new()
    box2.size = Vector3.ONE
    mm2.mesh = box2
    mm2.transform_format = MultiMesh.TRANSFORM_3D
    mm2.instance_count = 1
    var mmi2 := MultiMeshInstance3D.new()
    mmi2.multimesh = mm2
    root.add_child(mmi2)
    mm2.set_instance_transform(0, Transform3D(Basis.IDENTITY, Vector3(0, 0, -10)))
    var editor2 := VoxelEditor.new()
    root.add_child(editor2)
    editor2.init_editor(world2, mmi2, cam)
    var miss: bool = editor2.try_destroy_along_ray(Vector3.ZERO, Vector3(0, 0, -1), 5.0)
    if miss:
        _fail("short ray (5m) should not reach voxel at -10m but returned true")
        return false
    var hit_far: bool = editor2.try_destroy_along_ray(Vector3.ZERO, Vector3(0, 0, -1), 50.0)
    if not hit_far:
        _fail("long ray (50m) should reach voxel at -10m but returned false")
        return false

    return true
