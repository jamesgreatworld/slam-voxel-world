# vxw_loader.gd
# GDScript port of vxw_format.py — read-only.
# Mirrors §4 of docs/superpowers/plans/2026-05-22-slam-voxel-world.md.
# Supports: manifest.json, palette.json, chunks/*.chunk (DENSE + RLE, RAW + ZSTD).
# Not implemented: SVO encoding, LZ4 compression, CRC32 verification.

class_name VxwLoader extends RefCounted

const FORMAT_VERSION := 0x0100
const HEADER_SIZE := 28

enum Encoding { DENSE = 0, RLE = 1, SVO = 2 }
enum Compression { RAW = 0, LZ4 = 1, ZSTD = 2, GZIP = 3 }


class VxwWorld extends RefCounted:
    var voxel_size_meters: float = 0.10
    var chunk_extent: int = 32
    var palette_rgb: PackedColorArray  # indexed by material_id
    # Parallel arrays: positions[i] is voxel world-pos in meters, colors[i] is its RGB.
    var positions: PackedVector3Array
    var colors: PackedColorArray

    func voxel_count() -> int:
        return positions.size()


static func load_world(world_path: String) -> VxwWorld:
    var world := VxwWorld.new()

    # ---- manifest.json ----
    var mtxt := FileAccess.get_file_as_string(world_path + "/manifest.json")
    if mtxt.is_empty():
        push_error("Cannot read manifest.json from " + world_path)
        return world
    var manifest = JSON.parse_string(mtxt)
    world.voxel_size_meters = float(manifest.voxel.size_meters)
    world.chunk_extent = int(manifest.voxel.chunk_extent_voxels[0])

    # ---- palette.json ----
    var ptxt := FileAccess.get_file_as_string(world_path + "/palette.json")
    var palette = JSON.parse_string(ptxt)
    world.palette_rgb = PackedColorArray()
    # Build a 256-slot lookup so material_id index is direct.
    world.palette_rgb.resize(256)
    for m in palette.materials:
        var rgb = m.color_rgb
        world.palette_rgb[int(m.id)] = Color(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0)

    # ---- chunks/ ----
    world.positions = PackedVector3Array()
    world.colors = PackedColorArray()
    var dir := DirAccess.open(world_path + "/chunks")
    if dir == null:
        push_error("Cannot open chunks/ dir in " + world_path)
        return world
    dir.list_dir_begin()
    var name := dir.get_next()
    var chunks_loaded := 0
    while name != "":
        if name.ends_with(".chunk"):
            _load_chunk(world_path + "/chunks/" + name, world)
            chunks_loaded += 1
        name = dir.get_next()
    print("[vxw] loaded %d chunks, %d voxels, voxel_size=%.3fm" %
        [chunks_loaded, world.positions.size(), world.voxel_size_meters])
    return world


static func _load_chunk(path: String, world: VxwWorld) -> void:
    var f := FileAccess.open(path, FileAccess.READ)
    if f == null:
        push_error("Cannot open chunk " + path)
        return
    var raw := f.get_buffer(f.get_length())
    f.close()

    if raw.size() < HEADER_SIZE + 4:
        push_error("Chunk too small: " + path)
        return
    if raw[0] != 0x43 or raw[1] != 0x48 or raw[2] != 0x4E or raw[3] != 0x4B:  # "CHNK"
        push_error("Bad magic in " + path)
        return

    var format_version := raw.decode_u16(4)
    if format_version != FORMAT_VERSION:
        push_error("Unsupported format version 0x%04x in %s" % [format_version, path])
        return

    var compression := raw.decode_u16(6)
    var cx := raw.decode_s32(8)
    var cy := raw.decode_s32(12)
    var cz := raw.decode_s32(16)
    var encoding := int(raw[20])
    var extent_log2 := int(raw[21])
    var payload_bytes := raw.decode_u32(24)
    var extent := 1 << extent_log2

    var payload := raw.slice(HEADER_SIZE, HEADER_SIZE + payload_bytes)

    var data: PackedByteArray
    var max_size := extent * extent * extent * 8 + 1024
    if compression == Compression.RAW:
        data = payload
    elif compression == Compression.GZIP:
        data = payload.decompress_dynamic(max_size, FileAccess.COMPRESSION_GZIP)
        if data.is_empty():
            push_error("GZIP decompression failed in " + path)
            return
    elif compression == Compression.ZSTD:
        # Godot 4.6's decompress_dynamic does NOT support ZSTD. Use --compression gzip
        # when generating the .vxw if you need to load it in Godot.
        push_error("ZSTD compression not supported in Godot loader (regenerate with --compression gzip)")
        return
    else:
        push_error("Unsupported compression %d in %s" % [compression, path])
        return

    var voxel_size := world.voxel_size_meters
    var chunk_origin := Vector3(cx * extent, cy * extent, cz * extent) * voxel_size

    if encoding == Encoding.DENSE:
        _decode_dense(data, extent, chunk_origin, voxel_size, world)
    elif encoding == Encoding.RLE:
        _decode_rle(data, extent, chunk_origin, voxel_size, world)
    else:
        push_error("Unsupported encoding %d in %s" % [encoding, path])


static func _decode_dense(
    data: PackedByteArray, extent: int, origin: Vector3, vs: float, world: VxwWorld
) -> void:
    # C-order: x slowest, z fastest. Matches numpy.tobytes(order='C') on shape (E,E,E).
    var idx := 0
    for x in extent:
        for y in extent:
            for z in extent:
                var mid := int(data[idx])
                if mid != 0:
                    world.positions.append(origin + Vector3(x, y, z) * vs)
                    world.colors.append(world.palette_rgb[mid])
                idx += 4


static func _decode_rle(
    data: PackedByteArray, extent: int, origin: Vector3, vs: float, world: VxwWorld
) -> void:
    var run_count := data.decode_u32(0)
    var pos := 4
    var lin_idx := 0
    var ee := extent * extent
    for _r in run_count:
        var cell_u32 := data.decode_u32(pos)
        pos += 4
        var run_len := int(data[pos])
        pos += 1
        var mid := cell_u32 & 0xFF
        if mid != 0:
            var color := world.palette_rgb[mid]
            for i in run_len:
                var li := lin_idx + i
                # Inverse of C-order flattening for shape (E,E,E) → x slow, z fast
                var z := li % extent
                var y := (li / extent) % extent
                var x := li / ee
                world.positions.append(origin + Vector3(x, y, z) * vs)
                world.colors.append(color)
        lin_idx += run_len
