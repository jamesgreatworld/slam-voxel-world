# humanoid_visual.gd — build a simple block-style humanoid Node3D tree.
#
# Phase 1: static figure, no animation. Replaces the red box on the stereo
# rig so 3P views see a person walking around instead of a wireframe-style
# sensor box. Pure BoxMesh; no external GLB dependency, so the project
# remains self-contained and matches the MC-pixel aesthetic.
#
# Coordinate convention: the humanoid is built so the rig's local origin
# sits at the figure's waist (consistent with the rig's default behaviour —
# spawn_y_m=1.0 puts the waist 1 m above the floor, head ~1.7 m, feet ~0.1 m).
#
# Returned root has named children for the limbs (LeftArm / RightArm /
# LeftLeg / RightLeg pivots) so phase 2 can attach a walk-cycle animation.

extends RefCounted

const _SKIN  := Color(0.94, 0.78, 0.62)
const _SHIRT := Color(0.25, 0.35, 0.7)
const _PANT  := Color(0.20, 0.18, 0.15)
const _SHOE  := Color(0.08, 0.08, 0.10)
const _HAIR  := Color(0.18, 0.12, 0.08)


static func build() -> Node3D:
    var root := Node3D.new()
    root.name = "Humanoid"

    # ---- Head (offset above waist) ----
    _add_part(root, "Head", _SKIN, Vector3(0.22, 0.22, 0.22),
              Vector3(0, 0.78, 0))
    _add_part(root, "Hair", _HAIR, Vector3(0.24, 0.07, 0.24),
              Vector3(0, 0.92, 0))

    # ---- Torso ----
    _add_part(root, "Torso", _SHIRT, Vector3(0.4, 0.55, 0.22),
              Vector3(0, 0.4, 0))

    # ---- Arms (pivoted at the shoulder so phase-2 swing rotates around it) ----
    # Hands hang from each arm pivot so they swing along with the arm.
    var left_arm := _add_pivoted_limb(root, "LeftArm",
        _SHIRT, Vector3(0.12, 0.45, 0.12), Vector3(-0.28, 0.6, 0))
    _add_part(left_arm, "Hand", _SKIN, Vector3(0.13, 0.13, 0.13),
              Vector3(0, -0.515, 0))   # wrist sits just below the sleeve end
    var right_arm := _add_pivoted_limb(root, "RightArm",
        _SHIRT, Vector3(0.12, 0.45, 0.12), Vector3(0.28, 0.6, 0))
    _add_part(right_arm, "Hand", _SKIN, Vector3(0.13, 0.13, 0.13),
              Vector3(0, -0.515, 0))

    # ---- Legs (pivoted at the hip) — shoes ride on the legs ----
    var left_leg := _add_pivoted_limb(root, "LeftLeg",
        _PANT, Vector3(0.16, 0.55, 0.16), Vector3(-0.1, 0.05, 0))
    _add_part(left_leg, "Shoe", _SHOE, Vector3(0.18, 0.08, 0.26),
              Vector3(0, -0.59, 0.04))
    var right_leg := _add_pivoted_limb(root, "RightLeg",
        _PANT, Vector3(0.16, 0.55, 0.16), Vector3(0.1, 0.05, 0))
    _add_part(right_leg, "Shoe", _SHOE, Vector3(0.18, 0.08, 0.26),
              Vector3(0, -0.59, 0.04))

    return root


# Static MeshInstance3D for body parts that won't be rotated independently.
static func _add_part(parent: Node3D, n: String, col: Color,
                      size: Vector3, pos: Vector3) -> MeshInstance3D:
    var box := BoxMesh.new()
    box.size = size
    var mat := StandardMaterial3D.new()
    mat.albedo_color = col
    mat.metallic = 0.05
    mat.roughness = 0.78
    var mi := MeshInstance3D.new()
    mi.name = n
    mi.mesh = box
    mi.material_override = mat
    mi.position = pos
    parent.add_child(mi)
    return mi


# Returns a Node3D pivot whose origin sits at the shoulder/hip joint. The
# limb mesh hangs below the pivot, so rotating the pivot around the local
# X axis swings the limb forward / back. Phase 2 walk animation will reach
# in via root.get_node("LeftArm") etc. and animate .rotation.x.
static func _add_pivoted_limb(parent: Node3D, n: String, col: Color,
                              size: Vector3, joint_pos: Vector3) -> Node3D:
    var pivot := Node3D.new()
    pivot.name = n
    pivot.position = joint_pos
    parent.add_child(pivot)

    var box := BoxMesh.new()
    box.size = size
    var mat := StandardMaterial3D.new()
    mat.albedo_color = col
    mat.metallic = 0.05
    mat.roughness = 0.78
    var mi := MeshInstance3D.new()
    mi.name = "Mesh"
    mi.mesh = box
    mi.material_override = mat
    # Centre the mesh half its length below the pivot so the rotation point
    # is at the top of the limb (shoulder / hip).
    mi.position = Vector3(0, -size.y * 0.5, 0)
    pivot.add_child(mi)
    return pivot
