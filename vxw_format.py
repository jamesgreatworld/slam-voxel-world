"""vxw — VoxelWorld file format library.

Implements §4 of docs/superpowers/plans/2026-05-22-slam-voxel-world.md.
This is the only module SLAM (M2) and the game engine (M4/M5) share.

P0 scope:
- manifest.json, palette.json read/write with contract validation
- .chunk binary read/write (DENSE + RLE encodings, RAW + ZSTD compression)
- chunks.idx read/write
- World directory read/write (the public API)

Out of P0 scope (raise NotImplementedError):
- SVO encoding
- LZ4 compression
- semantics/instances.json
- trajectory.json
"""

from __future__ import annotations

import gzip
import json
import struct
import zlib
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Optional

import numpy as np
import zstandard as zstd


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------


class VxwError(Exception):
    """Base class for all vxw errors."""


class VxwFormatError(VxwError):
    """The file violates the .vxw format contract."""


class VxwCorruptError(VxwError):
    """The file passed format checks but its data is corrupted (CRC mismatch)."""


class VxwVersionError(VxwError):
    """The file is a newer/older format version we cannot handle."""


# ---------------------------------------------------------------------------
# Format constants
# ---------------------------------------------------------------------------


CHUNK_MAGIC = b"CHNK"
INDEX_MAGIC = b"CIDX"
FORMAT_VERSION = 0x0100  # major=1, minor=0
CHUNK_HEADER_SIZE = 28
INDEX_ENTRY_SIZE = 24  # 3×int32 + uint64 + uint32


class Encoding(IntEnum):
    DENSE = 0
    RLE = 1
    SVO = 2


class Compression(IntEnum):
    RAW = 0
    LZ4 = 1
    ZSTD = 2
    GZIP = 3


# 4 bytes per voxel — see Spec §4.4
VOXEL_DTYPE = np.dtype(
    [
        ("material_id", "u1"),
        ("semantic_id", "u1"),
        ("state", "u1"),
        ("color_palette_idx", "u1"),
    ]
)


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Material:
    id: int
    name: str
    color_rgb: tuple[int, int, int]
    flags: tuple[str, ...]
    density: Optional[float] = None
    hardness: Optional[float] = None

    def to_dict(self) -> dict:
        d: dict = {
            "id": self.id,
            "name": self.name,
            "color_rgb": list(self.color_rgb),
            "flags": list(self.flags),
        }
        if self.density is not None:
            d["density"] = self.density
        if self.hardness is not None:
            d["hardness"] = self.hardness
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Material":
        return cls(
            id=int(d["id"]),
            name=str(d["name"]),
            color_rgb=tuple(d["color_rgb"]),
            flags=tuple(d["flags"]),
            density=d.get("density"),
            hardness=d.get("hardness"),
        )


@dataclass(frozen=True)
class SemanticClass:
    id: int
    name: str
    default_material: int

    def to_dict(self) -> dict:
        return {"id": self.id, "name": self.name, "default_material": self.default_material}

    @classmethod
    def from_dict(cls, d: dict) -> "SemanticClass":
        return cls(
            id=int(d["id"]),
            name=str(d["name"]),
            default_material=int(d["default_material"]),
        )


# Accept tuple OR list for flags/colors; normalize on construction.
def _normalize_material(m: Material) -> Material:
    return Material(
        id=m.id,
        name=m.name,
        color_rgb=tuple(m.color_rgb),
        flags=tuple(m.flags),
        density=m.density,
        hardness=m.hardness,
    )


@dataclass
class Palette:
    materials: list
    semantic_classes: list
    color_lut: list

    def __post_init__(self) -> None:
        self.materials = [_normalize_material(m) for m in self.materials]
        self.color_lut = [tuple(c) for c in self.color_lut]
        if len(self.materials) > 256:
            raise VxwFormatError(f"materials cannot exceed 256, got {len(self.materials)}")
        if len(self.color_lut) > 256:
            raise VxwFormatError(f"color_lut cannot exceed 256, got {len(self.color_lut)}")

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Palette):
            return NotImplemented
        return (
            self.materials == other.materials
            and self.semantic_classes == other.semantic_classes
            and self.color_lut == other.color_lut
        )


@dataclass
class Manifest:
    world_id: str
    voxel_size_meters: float
    chunk_extent: int
    bounds_chunks_min: tuple
    bounds_chunks_max: tuple
    coord_convention: str = "right_handed_y_up"
    world_up_axis: tuple = (0, 1, 0)
    scene_scale_meters_per_unit: float = 1.0
    world_origin_in_slam_frame: tuple = (0.0, 0.0, 0.0)
    lod_levels: int = 1
    created_at: str = ""
    source_slam_system: str = ""
    source_sensor: str = ""
    raw_data_hash: str = ""
    format_version: str = "1.0"

    def __post_init__(self) -> None:
        self.bounds_chunks_min = tuple(self.bounds_chunks_min)
        self.bounds_chunks_max = tuple(self.bounds_chunks_max)
        self.world_up_axis = tuple(self.world_up_axis)
        self.world_origin_in_slam_frame = tuple(self.world_origin_in_slam_frame)


@dataclass
class Chunk:
    coord: tuple
    voxels: np.ndarray
    encoding: Encoding = Encoding.RLE
    compression: Compression = Compression.RAW

    def __post_init__(self) -> None:
        self.coord = tuple(self.coord)
        if self.voxels.dtype != VOXEL_DTYPE:
            raise VxwFormatError(f"voxels dtype must be VOXEL_DTYPE, got {self.voxels.dtype}")
        s = self.voxels.shape
        if len(s) != 3 or s[0] != s[1] or s[1] != s[2]:
            raise VxwFormatError(f"voxels must be cubic, got shape {s}")


@dataclass
class ChunkIndexEntry:
    coord: tuple
    file_offset: int
    size_bytes: int

    def __post_init__(self) -> None:
        self.coord = tuple(self.coord)


@dataclass
class World:
    manifest: Manifest
    palette: Palette
    chunks: dict

    def get_chunk_or_air(self, coord: tuple) -> np.ndarray:
        """Return voxel array for a coord; if absent, return all-air (Contract C9)."""
        coord = tuple(coord)
        if coord in self.chunks:
            return self.chunks[coord].voxels
        e = self.manifest.chunk_extent
        return np.zeros((e, e, e), dtype=VOXEL_DTYPE)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _is_pow2(n: int) -> bool:
    return n > 0 and (n & (n - 1)) == 0


def _int_log2(n: int) -> int:
    return n.bit_length() - 1


# ---------------------------------------------------------------------------
# Manifest I/O
# ---------------------------------------------------------------------------


def write_manifest(path: Path, m: Manifest) -> None:
    data = {
        "format_version": m.format_version,
        "world_id": m.world_id,
        "created_at": m.created_at,
        "source": {
            "slam_system": m.source_slam_system,
            "sensor": m.source_sensor,
            "raw_data_hash": m.raw_data_hash,
        },
        "coord_system": {
            "convention": m.coord_convention,
            "world_origin_in_slam_frame": list(m.world_origin_in_slam_frame),
            "world_up_axis": list(m.world_up_axis),
            "scene_scale_meters_per_unit": m.scene_scale_meters_per_unit,
        },
        "voxel": {
            "size_meters": m.voxel_size_meters,
            "chunk_extent_voxels": [m.chunk_extent, m.chunk_extent, m.chunk_extent],
        },
        "bounds_chunks": {
            "min": list(m.bounds_chunks_min),
            "max": list(m.bounds_chunks_max),
        },
        "lod_levels": m.lod_levels,
    }
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def read_manifest(path: Path) -> Manifest:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    cs = data["coord_system"]
    if cs["convention"] != "right_handed_y_up":
        raise VxwFormatError(
            f"unsupported coord_system.convention: {cs['convention']!r} "
            "(only right_handed_y_up is supported)"
        )
    vox = data["voxel"]
    ext = vox["chunk_extent_voxels"]
    if not (ext[0] == ext[1] == ext[2]):
        raise VxwFormatError(f"chunk_extent_voxels must be cubic, got {ext}")
    if not _is_pow2(ext[0]):
        raise VxwFormatError(f"chunk_extent_voxels must be power of two, got {ext[0]}")
    src = data.get("source", {}) or {}
    return Manifest(
        world_id=data["world_id"],
        voxel_size_meters=float(vox["size_meters"]),
        chunk_extent=int(ext[0]),
        bounds_chunks_min=tuple(data["bounds_chunks"]["min"]),
        bounds_chunks_max=tuple(data["bounds_chunks"]["max"]),
        coord_convention=cs["convention"],
        world_up_axis=tuple(cs["world_up_axis"]),
        scene_scale_meters_per_unit=float(cs["scene_scale_meters_per_unit"]),
        world_origin_in_slam_frame=tuple(cs["world_origin_in_slam_frame"]),
        lod_levels=int(data.get("lod_levels", 1)),
        created_at=data.get("created_at", ""),
        source_slam_system=src.get("slam_system", ""),
        source_sensor=src.get("sensor", ""),
        raw_data_hash=src.get("raw_data_hash", ""),
        format_version=data.get("format_version", "1.0"),
    )


# ---------------------------------------------------------------------------
# Palette I/O
# ---------------------------------------------------------------------------


def _validate_palette_air(p: Palette) -> None:
    if not p.materials:
        raise VxwFormatError("palette must contain at least material id=0 (air)")
    air = p.materials[0]
    if air.id != 0 or air.name != "air":
        raise VxwFormatError(
            f"palette.materials[0] must be material_id=0 named 'air', got id={air.id} name={air.name!r}"
        )


def write_palette(path: Path, p: Palette) -> None:
    _validate_palette_air(p)
    data = {
        "materials": [m.to_dict() for m in p.materials],
        "semantic_classes": [c.to_dict() for c in p.semantic_classes],
        "color_lut": [list(c) for c in p.color_lut],
    }
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def read_palette(path: Path) -> Palette:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return Palette(
        materials=[Material.from_dict(m) for m in data["materials"]],
        semantic_classes=[SemanticClass.from_dict(c) for c in data["semantic_classes"]],
        color_lut=[tuple(c) for c in data["color_lut"]],
    )


# ---------------------------------------------------------------------------
# Chunk binary I/O
# ---------------------------------------------------------------------------


def _encode_dense(voxels: np.ndarray) -> bytes:
    return voxels.tobytes(order="C")


def _decode_dense(buf: bytes, extent: int) -> np.ndarray:
    arr = np.frombuffer(buf, dtype=VOXEL_DTYPE).reshape(extent, extent, extent)
    return arr.copy()


def _encode_rle(voxels: np.ndarray) -> bytes:
    """RLE on a flattened (C-order) view. Run length capped at 255."""
    flat = voxels.ravel(order="C")
    n = len(flat)
    raw = flat.view(np.uint32)  # treat each 4-byte VoxelCell as a uint32 for fast compare

    runs: list[tuple[int, int]] = []  # (cell_uint32, length)
    i = 0
    while i < n:
        cell = int(raw[i])
        # find run end
        j = i + 1
        max_run = min(255, n - i)
        while j - i < max_run and int(raw[j]) == cell:
            j += 1
        runs.append((cell, j - i))
        i = j

    out = bytearray()
    out += struct.pack("<I", len(runs))
    for cell, length in runs:
        out += struct.pack("<IB", cell, length)
    return bytes(out)


def _decode_rle(buf: bytes, extent: int) -> np.ndarray:
    run_count = struct.unpack_from("<I", buf, 0)[0]
    total = extent * extent * extent
    flat = np.empty(total, dtype=VOXEL_DTYPE)
    flat_u32 = flat.view(np.uint32)
    pos = 4
    idx = 0
    for _ in range(run_count):
        cell, length = struct.unpack_from("<IB", buf, pos)
        pos += 5
        flat_u32[idx : idx + length] = cell
        idx += length
    if idx != total:
        raise VxwFormatError(
            f"RLE decoded {idx} voxels but expected {total} (extent={extent})"
        )
    return flat.reshape(extent, extent, extent).copy()


def write_chunk(path: Path, chunk: Chunk) -> None:
    extent = chunk.voxels.shape[0]
    if not _is_pow2(extent):
        raise VxwFormatError(f"chunk extent must be power of two, got {extent}")

    encoding = chunk.encoding
    if encoding == Encoding.DENSE:
        payload = _encode_dense(chunk.voxels)
    elif encoding == Encoding.RLE:
        payload = _encode_rle(chunk.voxels)
    elif encoding == Encoding.SVO:
        raise NotImplementedError("Encoding.SVO not implemented in P0")
    else:
        raise VxwFormatError(f"unknown encoding: {encoding!r}")

    compression = chunk.compression
    if compression == Compression.RAW:
        compressed = payload
    elif compression == Compression.ZSTD:
        compressed = zstd.ZstdCompressor().compress(payload)
    elif compression == Compression.GZIP:
        compressed = gzip.compress(payload, compresslevel=6)
    elif compression == Compression.LZ4:
        raise NotImplementedError("Compression.LZ4 not implemented in P0")
    else:
        raise VxwFormatError(f"unknown compression: {compression!r}")

    crc = zlib.crc32(compressed) & 0xFFFFFFFF
    cx, cy, cz = chunk.coord
    extent_log2 = _int_log2(extent)

    # 28-byte header. reserved[3] is used: byte 21 = extent_log2, bytes 22-23 = 0
    header = struct.pack(
        "<4sHHiiiBBHI",
        CHUNK_MAGIC,
        FORMAT_VERSION,
        int(compression),
        cx,
        cy,
        cz,
        int(encoding),
        extent_log2,
        0,  # reserved uint16
        len(compressed),
    )
    Path(path).write_bytes(header + compressed + struct.pack("<I", crc))


def read_chunk(path: Path) -> Chunk:
    raw = Path(path).read_bytes()
    if len(raw) < CHUNK_HEADER_SIZE + 4:
        raise VxwFormatError(f"chunk file too small ({len(raw)} bytes)")
    if raw[0:4] != CHUNK_MAGIC:
        raise VxwFormatError(f"bad magic: {raw[0:4]!r}, expected {CHUNK_MAGIC!r}")

    (
        format_version,
        compression_int,
        cx,
        cy,
        cz,
        encoding_int,
        extent_log2,
        _reserved,
        payload_bytes,
    ) = struct.unpack_from("<HHiiiBBHI", raw, 4)

    if format_version != FORMAT_VERSION:
        raise VxwVersionError(
            f"format_version {format_version:#06x} not supported (this lib supports {FORMAT_VERSION:#06x})"
        )

    payload_start = CHUNK_HEADER_SIZE
    payload_end = payload_start + payload_bytes
    if len(raw) < payload_end + 4:
        raise VxwFormatError(
            f"truncated chunk: expected {payload_end + 4} bytes, got {len(raw)}"
        )
    compressed = raw[payload_start:payload_end]
    crc_stored = struct.unpack_from("<I", raw, payload_end)[0]
    crc_computed = zlib.crc32(compressed) & 0xFFFFFFFF
    if crc_stored != crc_computed:
        raise VxwCorruptError(
            f"CRC mismatch in {path}: stored={crc_stored:#010x} computed={crc_computed:#010x}"
        )

    if compression_int == Compression.RAW:
        payload = compressed
    elif compression_int == Compression.ZSTD:
        payload = zstd.ZstdDecompressor().decompress(compressed)
    elif compression_int == Compression.GZIP:
        payload = gzip.decompress(compressed)
    elif compression_int == Compression.LZ4:
        raise NotImplementedError("LZ4 decompression not implemented in P0")
    else:
        raise VxwFormatError(f"unknown compression: {compression_int}")

    extent = 1 << extent_log2

    if encoding_int == Encoding.DENSE:
        voxels = _decode_dense(payload, extent)
    elif encoding_int == Encoding.RLE:
        voxels = _decode_rle(payload, extent)
    elif encoding_int == Encoding.SVO:
        raise NotImplementedError("SVO decoding not implemented in P0")
    else:
        raise VxwFormatError(f"unknown encoding: {encoding_int}")

    return Chunk(
        coord=(cx, cy, cz),
        voxels=voxels,
        encoding=Encoding(encoding_int),
        compression=Compression(compression_int),
    )


# ---------------------------------------------------------------------------
# chunks.idx
# ---------------------------------------------------------------------------


def _morton_key(coord: tuple) -> int:
    """Bit-interleave 3 ints (signed → offset to unsigned) for stable ordering."""
    x, y, z = (c + (1 << 31) for c in coord)  # shift to unsigned
    result = 0
    for i in range(32):
        result |= ((x >> i) & 1) << (3 * i)
        result |= ((y >> i) & 1) << (3 * i + 1)
        result |= ((z >> i) & 1) << (3 * i + 2)
    return result


def write_chunks_index(path: Path, entries: list) -> None:
    sorted_entries = sorted(entries, key=lambda e: _morton_key(e.coord))
    out = bytearray()
    out += INDEX_MAGIC
    out += struct.pack("<I", len(sorted_entries))
    for e in sorted_entries:
        x, y, z = e.coord
        out += struct.pack("<iiiQI", x, y, z, e.file_offset, e.size_bytes)
    Path(path).write_bytes(bytes(out))


def read_chunks_index(path: Path) -> list:
    raw = Path(path).read_bytes()
    if raw[0:4] != INDEX_MAGIC:
        raise VxwFormatError(f"bad index magic: {raw[0:4]!r}, expected {INDEX_MAGIC!r}")
    count = struct.unpack_from("<I", raw, 4)[0]
    expected_size = 8 + count * INDEX_ENTRY_SIZE
    if len(raw) < expected_size:
        raise VxwFormatError(
            f"truncated index: expected {expected_size} bytes, got {len(raw)}"
        )
    entries = []
    for i in range(count):
        off = 8 + i * INDEX_ENTRY_SIZE
        x, y, z, file_offset, size_bytes = struct.unpack_from("<iiiQI", raw, off)
        entries.append(
            ChunkIndexEntry(
                coord=(x, y, z), file_offset=file_offset, size_bytes=size_bytes
            )
        )
    return entries


# ---------------------------------------------------------------------------
# World directory (the public API SLAM/engine actually use)
# ---------------------------------------------------------------------------


def write_world(path: Path, world: World) -> None:
    out = Path(path)
    out.mkdir(parents=True, exist_ok=True)
    (out / "chunks").mkdir(exist_ok=True)
    write_manifest(out / "manifest.json", world.manifest)
    write_palette(out / "palette.json", world.palette)

    entries = []
    for coord, chunk in world.chunks.items():
        x, y, z = coord
        cpath = out / "chunks" / f"{x}_{y}_{z}.chunk"
        write_chunk(cpath, chunk)
        entries.append(
            ChunkIndexEntry(coord=coord, file_offset=0, size_bytes=cpath.stat().st_size)
        )
    write_chunks_index(out / "chunks.idx", entries)


def read_world(path: Path) -> World:
    root = Path(path)
    manifest = read_manifest(root / "manifest.json")
    palette = read_palette(root / "palette.json")
    chunks: dict = {}
    chunks_dir = root / "chunks"
    if chunks_dir.is_dir():
        for cpath in chunks_dir.glob("*.chunk"):
            chunk = read_chunk(cpath)
            chunks[chunk.coord] = chunk
    return World(manifest=manifest, palette=palette, chunks=chunks)
