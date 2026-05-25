# vxw_writer.gd
# GDScript writer mirroring vxw_format.py::write_chunk byte-for-byte.
# Supports DENSE + RLE encodings, RAW + GZIP compression.
# Includes a table-based zlib-compatible CRC32 and a patch_voxel helper.

class_name VxwWriter extends RefCounted

const FORMAT_VERSION := 0x0100
const HEADER_SIZE := 28

enum Encoding { DENSE = 0, RLE = 1, SVO = 2 }
enum Compression { RAW = 0, LZ4 = 1, ZSTD = 2, GZIP = 3 }

# --- CRC32 table (zlib polynomial), built lazily on first crc32() call. ---
static var _crc_table: PackedInt64Array


static func _ensure_crc_table() -> void:
    if _crc_table.size() == 256:
        return
    var t := PackedInt64Array()
    t.resize(256)
    for n in 256:
        var c: int = n
        for _k in 8:
            if (c & 1) != 0:
                c = (c >> 1) ^ 0xEDB88320
            else:
                c = c >> 1
        t[n] = c & 0xFFFFFFFF
    _crc_table = t


static func crc32(data: PackedByteArray) -> int:
    _ensure_crc_table()
    var crc: int = 0xFFFFFFFF
    var n := data.size()
    for i in n:
        var idx: int = (crc ^ data[i]) & 0xFF
        crc = (crc >> 8) ^ int(_crc_table[idx])
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF


# --- Public API ---

# Returns OK (0) on success, or a non-zero error code.
static func write_chunk_file(
    path: String,
    chunk_x: int, chunk_y: int, chunk_z: int,
    voxels_flat: PackedByteArray,    # (E*E*E*4) bytes, C-order x slowest z fastest
    extent: int,
    encoding: int,
    compression: int
) -> int:
    if extent <= 0 or (extent & (extent - 1)) != 0:
        push_error("[vxw_writer] extent must be power of two, got %d" % extent)
        return ERR_INVALID_PARAMETER

    var expected_bytes: int = extent * extent * extent * 4
    if voxels_flat.size() != expected_bytes:
        push_error("[vxw_writer] voxels_flat size %d != expected %d (extent=%d)"
            % [voxels_flat.size(), expected_bytes, extent])
        return ERR_INVALID_PARAMETER

    var payload: PackedByteArray
    if encoding == Encoding.DENSE:
        payload = _encode_dense(voxels_flat)
    elif encoding == Encoding.RLE:
        payload = _encode_rle(voxels_flat)
    else:
        push_error("[vxw_writer] unsupported encoding %d (only DENSE=0, RLE=1)" % encoding)
        return ERR_INVALID_PARAMETER

    var compressed: PackedByteArray
    if compression == Compression.RAW:
        compressed = payload
    elif compression == Compression.GZIP:
        compressed = payload.compress(FileAccess.COMPRESSION_GZIP)
        if compressed.is_empty() and not payload.is_empty():
            push_error("[vxw_writer] GZIP compression failed")
            return FAILED
    else:
        push_error("[vxw_writer] unsupported compression %d (only RAW=0, GZIP=3)" % compression)
        return ERR_INVALID_PARAMETER

    var crc: int = crc32(compressed)
    var extent_log2: int = _int_log2(extent)

    var header := PackedByteArray()
    header.resize(HEADER_SIZE)
    # Magic "CHNK"
    header[0] = 0x43
    header[1] = 0x48
    header[2] = 0x4E
    header[3] = 0x4B
    header.encode_u16(4, FORMAT_VERSION)
    header.encode_u16(6, compression)
    header.encode_s32(8, chunk_x)
    header.encode_s32(12, chunk_y)
    header.encode_s32(16, chunk_z)
    header[20] = encoding & 0xFF
    header[21] = extent_log2 & 0xFF
    header.encode_u16(22, 0)  # reserved
    header.encode_u32(24, compressed.size())

    var crc_tail := PackedByteArray()
    crc_tail.resize(4)
    crc_tail.encode_u32(0, crc)

    var dir_path := path.get_base_dir()
    if not dir_path.is_empty() and not DirAccess.dir_exists_absolute(dir_path):
        var mk_err := DirAccess.make_dir_recursive_absolute(dir_path)
        if mk_err != OK:
            push_error("[vxw_writer] cannot create dir %s (err=%d)" % [dir_path, mk_err])
            return mk_err

    var f := FileAccess.open(path, FileAccess.WRITE)
    if f == null:
        var open_err := FileAccess.get_open_error()
        push_error("[vxw_writer] cannot open %s for writing (err=%d)" % [path, open_err])
        return open_err if open_err != OK else FAILED

    f.store_buffer(header)
    f.store_buffer(compressed)
    f.store_buffer(crc_tail)
    f.close()
    return OK


# Convenience: writes a single voxel into an existing on-disk chunk by rewriting it.
# Reads the current chunk if present, sets the voxel, re-writes. If the chunk
# file doesn't exist, creates an all-air chunk with the voxel.
static func patch_voxel(
    chunk_path: String,
    chunk_coord: Vector3i,
    local_voxel: Vector3i,
    cell_bytes: PackedByteArray,
    extent: int,
    encoding: int,
    compression: int
) -> int:
    if cell_bytes.size() != 4:
        push_error("[vxw_writer] cell_bytes must be 4 bytes, got %d" % cell_bytes.size())
        return ERR_INVALID_PARAMETER
    if extent <= 0 or (extent & (extent - 1)) != 0:
        push_error("[vxw_writer] extent must be power of two, got %d" % extent)
        return ERR_INVALID_PARAMETER
    if local_voxel.x < 0 or local_voxel.x >= extent \
            or local_voxel.y < 0 or local_voxel.y >= extent \
            or local_voxel.z < 0 or local_voxel.z >= extent:
        push_error("[vxw_writer] local_voxel %s out of range [0,%d)" % [local_voxel, extent])
        return ERR_INVALID_PARAMETER

    var voxels_flat: PackedByteArray
    if FileAccess.file_exists(chunk_path):
        var loaded := _read_voxels_flat(chunk_path, extent)
        if loaded.is_empty():
            # _read_voxels_flat already pushed an error; fall back to fresh air buffer
            voxels_flat = _make_air_buffer(extent)
        else:
            voxels_flat = loaded
    else:
        voxels_flat = _make_air_buffer(extent)

    var ee := extent * extent
    var lin_idx: int = local_voxel.x * ee + local_voxel.y * extent + local_voxel.z
    var byte_idx := lin_idx * 4
    voxels_flat[byte_idx + 0] = cell_bytes[0]
    voxels_flat[byte_idx + 1] = cell_bytes[1]
    voxels_flat[byte_idx + 2] = cell_bytes[2]
    voxels_flat[byte_idx + 3] = cell_bytes[3]

    return write_chunk_file(
        chunk_path,
        chunk_coord.x, chunk_coord.y, chunk_coord.z,
        voxels_flat, extent, encoding, compression
    )


# --- Internal encoders ---

static func _encode_dense(voxels_flat: PackedByteArray) -> PackedByteArray:
    return voxels_flat


static func _encode_rle(voxels_flat: PackedByteArray) -> PackedByteArray:
    # RLE on flattened C-order array. cell_bits is the little-endian u32
    # interpretation of the 4-byte VoxelCell. Run length capped at 255.
    var total_bytes := voxels_flat.size()
    var total_cells: int = total_bytes / 4

    # First pass: collect runs.
    var runs := PackedInt64Array()  # alternating [cell_u32, length, cell_u32, length, ...]
    var i := 0
    while i < total_cells:
        var byte_i := i * 4
        var cell: int = voxels_flat.decode_u32(byte_i)
        var j := i + 1
        var remaining_cap: int = 255
        var max_j: int = i + min(remaining_cap, total_cells - i)
        while j < max_j and voxels_flat.decode_u32(j * 4) == cell:
            j += 1
        runs.append(cell)
        runs.append(j - i)
        i = j

    var run_count: int = runs.size() / 2
    var out := PackedByteArray()
    out.resize(4 + run_count * 5)
    out.encode_u32(0, run_count)
    var pos := 4
    for r in run_count:
        var cell_u32: int = int(runs[r * 2])
        var length: int = int(runs[r * 2 + 1])
        out.encode_u32(pos, cell_u32)
        pos += 4
        out[pos] = length & 0xFF
        pos += 1
    return out


# --- Internal helpers ---

static func _int_log2(n: int) -> int:
    var r := 0
    var v := n
    while v > 1:
        v >>= 1
        r += 1
    return r


static func _make_air_buffer(extent: int) -> PackedByteArray:
    var b := PackedByteArray()
    b.resize(extent * extent * extent * 4)
    # PackedByteArray.resize zero-initializes.
    return b


# Internal: parses a chunk file and returns its voxels as a flat
# (E*E*E*4) byte buffer in C-order (x slow, z fast). Returns an empty
# PackedByteArray on error (after push_error).
static func _read_voxels_flat(path: String, expected_extent: int) -> PackedByteArray:
    var f := FileAccess.open(path, FileAccess.READ)
    if f == null:
        push_error("[vxw_writer] cannot open chunk %s for read" % path)
        return PackedByteArray()
    var raw := f.get_buffer(f.get_length())
    f.close()

    if raw.size() < HEADER_SIZE + 4:
        push_error("[vxw_writer] chunk too small: %s" % path)
        return PackedByteArray()
    if raw[0] != 0x43 or raw[1] != 0x48 or raw[2] != 0x4E or raw[3] != 0x4B:
        push_error("[vxw_writer] bad magic in %s" % path)
        return PackedByteArray()

    var format_version := raw.decode_u16(4)
    if format_version != FORMAT_VERSION:
        push_error("[vxw_writer] unsupported format version 0x%04x in %s"
            % [format_version, path])
        return PackedByteArray()

    var compression := raw.decode_u16(6)
    var encoding := int(raw[20])
    var extent_log2 := int(raw[21])
    var payload_bytes := raw.decode_u32(24)
    var extent := 1 << extent_log2

    if extent != expected_extent:
        push_error("[vxw_writer] extent mismatch in %s: file=%d expected=%d"
            % [path, extent, expected_extent])
        return PackedByteArray()

    if raw.size() < HEADER_SIZE + payload_bytes + 4:
        push_error("[vxw_writer] truncated chunk %s" % path)
        return PackedByteArray()
    var compressed := raw.slice(HEADER_SIZE, HEADER_SIZE + payload_bytes)
    var crc_stored := raw.decode_u32(HEADER_SIZE + payload_bytes)
    var crc_computed := crc32(compressed)
    if crc_stored != crc_computed:
        push_error("[vxw_writer] CRC mismatch in %s: stored=0x%08x computed=0x%08x"
            % [path, crc_stored, crc_computed])
        return PackedByteArray()

    var payload: PackedByteArray
    if compression == Compression.RAW:
        payload = compressed
    elif compression == Compression.GZIP:
        var max_size := extent * extent * extent * 4 + 1024
        payload = compressed.decompress_dynamic(max_size, FileAccess.COMPRESSION_GZIP)
        if payload.is_empty():
            push_error("[vxw_writer] GZIP decompress failed in %s" % path)
            return PackedByteArray()
    else:
        push_error("[vxw_writer] unsupported compression %d in %s" % [compression, path])
        return PackedByteArray()

    var expected_bytes := extent * extent * extent * 4
    if encoding == Encoding.DENSE:
        if payload.size() != expected_bytes:
            push_error("[vxw_writer] dense payload size %d != expected %d in %s"
                % [payload.size(), expected_bytes, path])
            return PackedByteArray()
        return payload
    elif encoding == Encoding.RLE:
        return _decode_rle_to_flat(payload, extent)
    else:
        push_error("[vxw_writer] unsupported encoding %d in %s" % [encoding, path])
        return PackedByteArray()


static func _decode_rle_to_flat(buf: PackedByteArray, extent: int) -> PackedByteArray:
    var total_cells: int = extent * extent * extent
    var out := PackedByteArray()
    out.resize(total_cells * 4)
    var run_count := buf.decode_u32(0)
    var pos := 4
    var lin_idx := 0
    for _r in run_count:
        var cell_u32: int = buf.decode_u32(pos)
        pos += 4
        var run_len: int = int(buf[pos])
        pos += 1
        for k in run_len:
            out.encode_u32((lin_idx + k) * 4, cell_u32)
        lin_idx += run_len
    if lin_idx != total_cells:
        push_error("[vxw_writer] RLE decoded %d cells, expected %d" % [lin_idx, total_cells])
        return PackedByteArray()
    return out
