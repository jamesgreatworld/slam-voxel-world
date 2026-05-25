# _writer_selftest.gd
# Headless round-trip test for vxw_writer.gd.
# Run: F:\Godot\Godot_v4.6.3-stable_win64_console.exe --path F:\slam-voxel-world\godot_viewer \
#         --headless --script res://_writer_selftest.gd

extends SceneTree

# Use preload (not class_name) so we work in --script mode without depending on
# global class registration (mirrors main.gd's pattern).
const VxwWriter = preload("res://vxw_writer.gd")
const VxwLoader = preload("res://vxw_loader.gd")


const EXTENT := 32
const MARKER := Vector3i(5, 0, 7)
const MARKER_MATERIAL := 0x2A   # 42
const MARKER_SEMANTIC := 0x10   # 16
const MARKER_STATE := 0x03      # 3
const MARKER_COLOR := 0x07      # 7


func _init() -> void:
    var ok := _run()
    if ok:
        print("SELFTEST OK")
        quit(0)
    else:
        push_error("SELFTEST FAILED")
        quit(1)


func _run() -> bool:
    # Build E^3 voxel buffer, all-air except marker.
    var voxels := PackedByteArray()
    voxels.resize(EXTENT * EXTENT * EXTENT * 4)
    var ee := EXTENT * EXTENT
    var lin: int = MARKER.x * ee + MARKER.y * EXTENT + MARKER.z
    var b := lin * 4
    voxels[b + 0] = MARKER_MATERIAL
    voxels[b + 1] = MARKER_SEMANTIC
    voxels[b + 2] = MARKER_STATE
    voxels[b + 3] = MARKER_COLOR

    # Verify CRC implementation against a known vector: zlib.crc32(b"") == 0,
    # zlib.crc32(b"123456789") == 0xCBF43926 (the well-known check value).
    var empty_crc := VxwWriter.crc32(PackedByteArray())
    if empty_crc != 0:
        push_error("CRC of empty != 0: got 0x%08x" % empty_crc)
        return false
    var test_data := "123456789".to_utf8_buffer()
    var test_crc := VxwWriter.crc32(test_data)
    if test_crc != 0xCBF43926:
        push_error("CRC of '123456789' != 0xCBF43926: got 0x%08x" % test_crc)
        return false
    print("[selftest] CRC32 check vector OK")

    # Common output directory.
    var out_dir := ProjectSettings.globalize_path("res://../out")
    DirAccess.make_dir_recursive_absolute(out_dir)

    # ---- Case A: RLE + GZIP (the realistic patch_voxel default) ----
    var path_rle_gz := out_dir + "/_writer_test.chunk"
    var err := VxwWriter.write_chunk_file(
        path_rle_gz,
        1, 2, 3,
        voxels, EXTENT,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.GZIP
    )
    if err != OK:
        push_error("write_chunk_file (RLE/GZIP) failed: %d" % err)
        return false
    if not _roundtrip_via_loader(path_rle_gz, "RLE/GZIP"):
        return false

    # ---- Case B: DENSE + RAW ----
    var path_dense_raw := out_dir + "/_writer_test_dense.chunk"
    err = VxwWriter.write_chunk_file(
        path_dense_raw,
        0, 0, 0,
        voxels, EXTENT,
        VxwWriter.Encoding.DENSE,
        VxwWriter.Compression.RAW
    )
    if err != OK:
        push_error("write_chunk_file (DENSE/RAW) failed: %d" % err)
        return false
    if not _roundtrip_via_loader(path_dense_raw, "DENSE/RAW"):
        return false

    # ---- Case C: DENSE + GZIP ----
    var path_dense_gz := out_dir + "/_writer_test_dense_gz.chunk"
    err = VxwWriter.write_chunk_file(
        path_dense_gz,
        -1, -2, -3,
        voxels, EXTENT,
        VxwWriter.Encoding.DENSE,
        VxwWriter.Compression.GZIP
    )
    if err != OK:
        push_error("write_chunk_file (DENSE/GZIP) failed: %d" % err)
        return false
    if not _roundtrip_via_loader(path_dense_gz, "DENSE/GZIP"):
        return false

    # ---- Case D: RLE + RAW ----
    var path_rle_raw := out_dir + "/_writer_test_rle_raw.chunk"
    err = VxwWriter.write_chunk_file(
        path_rle_raw,
        7, 8, 9,
        voxels, EXTENT,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.RAW
    )
    if err != OK:
        push_error("write_chunk_file (RLE/RAW) failed: %d" % err)
        return false
    if not _roundtrip_via_loader(path_rle_raw, "RLE/RAW"):
        return false

    # ---- Case E: patch_voxel on existing file ----
    # First, create a chunk with one voxel via write_chunk_file, then patch a
    # second voxel via patch_voxel, then verify both are present.
    var path_patch := out_dir + "/_writer_test_patch.chunk"
    var base := PackedByteArray()
    base.resize(EXTENT * EXTENT * EXTENT * 4)
    var lin_a: int = 1 * ee + 2 * EXTENT + 3
    base[lin_a * 4 + 0] = 99
    err = VxwWriter.write_chunk_file(
        path_patch,
        5, 5, 5,
        base, EXTENT,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.GZIP
    )
    if err != OK:
        push_error("patch base write failed: %d" % err)
        return false
    var cell := PackedByteArray([77, 0, 0, 0])
    err = VxwWriter.patch_voxel(
        path_patch, Vector3i(5, 5, 5),
        Vector3i(10, 11, 12), cell,
        EXTENT,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.GZIP
    )
    if err != OK:
        push_error("patch_voxel failed: %d" % err)
        return false
    # Verify via internal reader and loader.
    var patched := VxwWriter._read_voxels_flat(path_patch, EXTENT)
    if patched.is_empty():
        push_error("could not re-read patched chunk")
        return false
    var idx_a := lin_a * 4
    var idx_b: int = (10 * ee + 11 * EXTENT + 12) * 4
    if patched[idx_a] != 99:
        push_error("patch lost original voxel: got %d expected 99" % patched[idx_a])
        return false
    if patched[idx_b] != 77:
        push_error("patch failed to set voxel: got %d expected 77" % patched[idx_b])
        return false
    print("[selftest] patch_voxel OK (preserved old, set new)")

    # ---- Case F: patch_voxel on non-existent file ----
    var path_patch2 := out_dir + "/_writer_test_patch_new.chunk"
    if FileAccess.file_exists(path_patch2):
        DirAccess.remove_absolute(path_patch2)
    err = VxwWriter.patch_voxel(
        path_patch2, Vector3i(0, 0, 0),
        Vector3i(1, 2, 3), PackedByteArray([55, 0, 0, 0]),
        EXTENT,
        VxwWriter.Encoding.RLE,
        VxwWriter.Compression.GZIP
    )
    if err != OK:
        push_error("patch_voxel on fresh file failed: %d" % err)
        return false
    var fresh := VxwWriter._read_voxels_flat(path_patch2, EXTENT)
    var fresh_idx: int = (1 * ee + 2 * EXTENT + 3) * 4
    if fresh.is_empty() or fresh[fresh_idx] != 55:
        push_error("fresh patch_voxel failed")
        return false
    print("[selftest] patch_voxel-on-missing OK")

    return true


# Validate a written chunk by parsing it via the VxwLoader code path. We
# reproduce the relevant parts of VxwLoader._load_chunk so we can assert on the
# decoded voxels directly (loader's public API only exposes positions/colors,
# which loses semantic/state info).
func _roundtrip_via_loader(path: String, label: String) -> bool:
    var flat := VxwWriter._read_voxels_flat(path, EXTENT)
    if flat.is_empty():
        push_error("[%s] _read_voxels_flat returned empty" % label)
        return false
    var ee := EXTENT * EXTENT
    var b: int = (MARKER.x * ee + MARKER.y * EXTENT + MARKER.z) * 4
    if flat[b + 0] != MARKER_MATERIAL \
            or flat[b + 1] != MARKER_SEMANTIC \
            or flat[b + 2] != MARKER_STATE \
            or flat[b + 3] != MARKER_COLOR:
        push_error("[%s] marker voxel not recovered: got (%d,%d,%d,%d)"
            % [label, flat[b], flat[b+1], flat[b+2], flat[b+3]])
        return false
    # Check that all other voxels are zero.
    var nonzero := 0
    for i in flat.size():
        if i >= b and i < b + 4:
            continue
        if flat[i] != 0:
            nonzero += 1
    if nonzero != 0:
        push_error("[%s] %d unexpected non-zero bytes" % [label, nonzero])
        return false

    # Also exercise the full VxwLoader._decode_dense / _decode_rle path by
    # creating a tiny synthetic VxwWorld and calling _load_chunk. This proves
    # the file is consumable by the public loader API.
    var world := VxwLoader.VxwWorld.new()
    world.voxel_size_meters = 0.10
    world.chunk_extent = EXTENT
    world.palette_rgb = PackedColorArray()
    world.palette_rgb.resize(256)
    for k in 256:
        world.palette_rgb[k] = Color(1, 1, 1)
    VxwLoader._load_chunk(path, world)
    if world.positions.size() != 1:
        push_error("[%s] loader recovered %d positions, expected 1"
            % [label, world.positions.size()])
        return false
    # Marker world-pos: chunk_origin + Vector3(MARKER)*vs.
    # Each case uses different chunk coord; we just check the offset modulo
    # chunk origin by re-reading the header.
    var f := FileAccess.open(path, FileAccess.READ)
    var raw := f.get_buffer(f.get_length())
    f.close()
    var cx := raw.decode_s32(8)
    var cy := raw.decode_s32(12)
    var cz := raw.decode_s32(16)
    var origin := Vector3(cx * EXTENT, cy * EXTENT, cz * EXTENT) * 0.10
    var expected := origin + Vector3(MARKER.x, MARKER.y, MARKER.z) * 0.10
    if not world.positions[0].is_equal_approx(expected):
        push_error("[%s] loader pos %s != expected %s"
            % [label, world.positions[0], expected])
        return false

    print("[selftest] %s round-trip OK (file=%s, size=%d bytes)"
        % [label, path.get_file(), raw.size()])
    return true
