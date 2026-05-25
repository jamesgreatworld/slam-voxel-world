# vxw_loader.gd
# GDScript port of vxw_format.py — read-only.
# Mirrors §4 of docs/spec.md.
# Supports: manifest.json, palette.json, chunks/*.chunk (DENSE + RLE, RAW + GZIP).
# Not implemented in this loader: SVO encoding, LZ4/ZSTD compression, CRC32 verification.

class_name VxwLoader extends RefCounted

const FORMAT_VERSION := 0x0100
const HEADER_SIZE := 28

enum Encoding { DENSE = 0, RLE = 1, SVO = 2 }
enum Compression { RAW = 0, LZ4 = 1, ZSTD = 2, GZIP = 3 }


class VxwWorld extends RefCounted:
    var voxel_size_meters: float = 0.10
    var chunk_extent: int = 32
    # palette_rgb[material_id] = Color (legacy convenience accessor)
    var palette_rgb: PackedColorArray
    # palette_materials[material_id] = Dictionary with full Material data
    # (color_rgb, flags, transparent, emission_rgb, emission_energy, metallic, roughness)
    var palette_materials: Dictionary
    # Parallel arrays — index i is the same voxel across positions / colors / material_ids.
    var positions: PackedVector3Array
    var colors: PackedColorArray
    var material_ids: PackedByteArray   # 1 byte per voxel (parallel to positions)

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
    world.palette_rgb.resize(256)
    world.palette_materials = {}
    for m in palette.materials:
        var rgb = m.color_rgb
        var col := Color(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0)
        var mid: int = int(m.id)
        world.palette_rgb[mid] = col
        # Full material data with sensible defaults for old palette.json files
        var em_rgb = m.emission_rgb if m.has("emission_rgb") else [0, 0, 0]
        world.palette_materials[mid] = {
            "id": mid,
            "name": String(m.name) if m.has("name") else "",
            "color_rgb": [int(rgb[0]), int(rgb[1]), int(rgb[2])],
            "flags": m.flags if m.has("flags") else [],
            "transparent": bool(m.transparent) if m.has("transparent") else false,
            "emission_rgb": [int(em_rgb[0]), int(em_rgb[1]), int(em_rgb[2])],
            "emission_energy": float(m.emission_energy) if m.has("emission_energy") else 0.0,
            "metallic": float(m.metallic) if m.has("metallic") else 0.0,
            "roughness": float(m.roughness) if m.has("roughness") else 0.75,
        }

    # ---- chunks/ ----
    world.positions = PackedVector3Array()
    world.colors = PackedColorArray()
    world.material_ids = PackedByteArray()
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
        push_error("ZSTD not supported in Godot loader; regenerate with --compression gzip")
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
    var idx := 0
    for x in extent:
        for y in extent:
            for z in extent:
                var mid := int(data[idx])
                if mid != 0:
                    world.positions.append(origin + Vector3(x, y, z) * vs)
                    world.colors.append(world.palette_rgb[mid])
                    world.material_ids.append(mid)
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
                var z := li % extent
                var y := (li / extent) % extent
                var x := li / ee
                world.positions.append(origin + Vector3(x, y, z) * vs)
                world.colors.append(color)
                world.material_ids.append(mid)
        lin_idx += run_len
