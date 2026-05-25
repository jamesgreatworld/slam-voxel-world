"""TDD red-phase tests for vxw_format.

These tests define the public surface of the .vxw format library.
Run with: pixi run pytest tests/ -v
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np
import pytest

import vxw_format as vxw


# ---------------------------------------------------------------------------
# Manifest
# ---------------------------------------------------------------------------


def test_manifest_roundtrip(tmp_path: Path) -> None:
    m = vxw.Manifest(
        world_id="11111111-2222-3333-4444-555555555555",
        voxel_size_meters=0.05,
        chunk_extent=32,
        bounds_chunks_min=(-10, -2, -10),
        bounds_chunks_max=(10, 4, 10),
        source_slam_system="Hydra",
        source_sensor="RealSense D455",
    )
    path = tmp_path / "manifest.json"
    vxw.write_manifest(path, m)
    loaded = vxw.read_manifest(path)
    assert loaded == m


def test_manifest_requires_y_up_convention(tmp_path: Path) -> None:
    """Contract: coord_convention must be right_handed_y_up (Spec §4.2)."""
    path = tmp_path / "manifest.json"
    path.write_text(
        '{"format_version":"1.0","world_id":"x","coord_system":'
        '{"convention":"left_handed_z_up","world_up_axis":[0,0,1],'
        '"scene_scale_meters_per_unit":1.0,"world_origin_in_slam_frame":[0,0,0]},'
        '"voxel":{"size_meters":0.05,"chunk_extent_voxels":[32,32,32]},'
        '"bounds_chunks":{"min":[0,0,0],"max":[1,1,1]},"lod_levels":1,'
        '"created_at":"2026-05-23T00:00:00Z","source":{}}'
    )
    with pytest.raises(vxw.VxwFormatError, match="convention"):
        vxw.read_manifest(path)


def test_manifest_rejects_non_cubic_chunk(tmp_path: Path) -> None:
    """Contract: chunk_extent_voxels must be three equal values (Spec §4.2)."""
    path = tmp_path / "manifest.json"
    path.write_text(
        '{"format_version":"1.0","world_id":"x","coord_system":'
        '{"convention":"right_handed_y_up","world_up_axis":[0,1,0],'
        '"scene_scale_meters_per_unit":1.0,"world_origin_in_slam_frame":[0,0,0]},'
        '"voxel":{"size_meters":0.05,"chunk_extent_voxels":[32,16,32]},'
        '"bounds_chunks":{"min":[0,0,0],"max":[1,1,1]},"lod_levels":1,'
        '"created_at":"2026-05-23T00:00:00Z","source":{}}'
    )
    with pytest.raises(vxw.VxwFormatError, match="cubic"):
        vxw.read_manifest(path)


def test_manifest_rejects_non_power_of_two_chunk(tmp_path: Path) -> None:
    path = tmp_path / "manifest.json"
    path.write_text(
        '{"format_version":"1.0","world_id":"x","coord_system":'
        '{"convention":"right_handed_y_up","world_up_axis":[0,1,0],'
        '"scene_scale_meters_per_unit":1.0,"world_origin_in_slam_frame":[0,0,0]},'
        '"voxel":{"size_meters":0.05,"chunk_extent_voxels":[30,30,30]},'
        '"bounds_chunks":{"min":[0,0,0],"max":[1,1,1]},"lod_levels":1,'
        '"created_at":"2026-05-23T00:00:00Z","source":{}}'
    )
    with pytest.raises(vxw.VxwFormatError, match="power of two"):
        vxw.read_manifest(path)


# ---------------------------------------------------------------------------
# Palette
# ---------------------------------------------------------------------------


def test_palette_roundtrip(tmp_path: Path) -> None:
    p = vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=["empty"]),
            vxw.Material(
                id=1,
                name="concrete",
                color_rgb=(180, 180, 180),
                flags=["solid", "destructible"],
            ),
            vxw.Material(
                id=2,
                name="wood",
                color_rgb=(139, 90, 43),
                flags=["solid", "destructible", "flammable"],
            ),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0, name="unknown", default_material=1),
            vxw.SemanticClass(id=1, name="wall", default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 178, 175), (139, 90, 43)],
    )
    path = tmp_path / "palette.json"
    vxw.write_palette(path, p)
    loaded = vxw.read_palette(path)
    assert loaded == p


def test_palette_id_zero_must_be_air(tmp_path: Path) -> None:
    """Contract: material_id=0 is reserved for air (Spec §4.3)."""
    bad = vxw.Palette(
        materials=[vxw.Material(id=0, name="stone", color_rgb=(1, 1, 1), flags=["solid"])],
        semantic_classes=[],
        color_lut=[],
    )
    path = tmp_path / "palette.json"
    with pytest.raises(vxw.VxwFormatError, match="material_id=0.*air"):
        vxw.write_palette(path, bad)


def test_palette_max_256_materials() -> None:
    too_many = [
        vxw.Material(id=i, name=f"m{i}", color_rgb=(0, 0, 0), flags=[])
        for i in range(257)
    ]
    too_many[0] = vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=["empty"])
    with pytest.raises(vxw.VxwFormatError, match="256"):
        vxw.Palette(materials=too_many, semantic_classes=[], color_lut=[])


# ---------------------------------------------------------------------------
# Chunk binary format
# ---------------------------------------------------------------------------


def _empty_chunk_voxels(extent: int = 32) -> np.ndarray:
    return np.zeros((extent, extent, extent), dtype=vxw.VOXEL_DTYPE)


def test_chunk_dense_roundtrip(tmp_path: Path) -> None:
    voxels = _empty_chunk_voxels()
    voxels[0, 0, 0] = (1, 1, 0, 0)  # material_id=1, semantic_id=1
    voxels[5, 3, 7] = (2, 3, 1, 42)  # material=2, semantic=3, state=1, color_idx=42
    chunk = vxw.Chunk(
        coord=(0, 0, 0),
        voxels=voxels,
        encoding=vxw.Encoding.DENSE,
        compression=vxw.Compression.RAW,
    )
    path = tmp_path / "0_0_0.chunk"
    vxw.write_chunk(path, chunk)
    loaded = vxw.read_chunk(path)
    assert loaded.coord == chunk.coord
    assert loaded.encoding == vxw.Encoding.DENSE
    assert np.array_equal(loaded.voxels, voxels)


def test_chunk_rle_roundtrip(tmp_path: Path) -> None:
    voxels = _empty_chunk_voxels()
    # A wall: 4×4×1 of concrete at z=0
    voxels[0:4, 0:4, 0] = (1, 1, 0, 0)
    chunk = vxw.Chunk(
        coord=(2, 0, -3),
        voxels=voxels,
        encoding=vxw.Encoding.RLE,
        compression=vxw.Compression.RAW,
    )
    path = tmp_path / "2_0_-3.chunk"
    vxw.write_chunk(path, chunk)
    loaded = vxw.read_chunk(path)
    assert loaded.coord == (2, 0, -3)
    assert loaded.encoding == vxw.Encoding.RLE
    assert np.array_equal(loaded.voxels, voxels)


def test_chunk_rle_is_smaller_than_dense_for_sparse_chunk(tmp_path: Path) -> None:
    """RLE should win on sparse chunks (sanity check on the encoding choice)."""
    voxels = _empty_chunk_voxels()
    voxels[0, 0, 0] = (1, 1, 0, 0)
    dense = vxw.Chunk(
        coord=(0, 0, 0),
        voxels=voxels,
        encoding=vxw.Encoding.DENSE,
        compression=vxw.Compression.RAW,
    )
    rle = vxw.Chunk(
        coord=(0, 0, 0),
        voxels=voxels,
        encoding=vxw.Encoding.RLE,
        compression=vxw.Compression.RAW,
    )
    dense_path = tmp_path / "d.chunk"
    rle_path = tmp_path / "r.chunk"
    vxw.write_chunk(dense_path, dense)
    vxw.write_chunk(rle_path, rle)
    assert rle_path.stat().st_size < dense_path.stat().st_size


def test_chunk_zstd_compression_roundtrip(tmp_path: Path) -> None:
    voxels = _empty_chunk_voxels()
    voxels[10:20, 10:20, 10:20] = (1, 1, 0, 0)
    chunk = vxw.Chunk(
        coord=(0, 0, 0),
        voxels=voxels,
        encoding=vxw.Encoding.DENSE,
        compression=vxw.Compression.ZSTD,
    )
    path = tmp_path / "0_0_0.chunk"
    vxw.write_chunk(path, chunk)
    loaded = vxw.read_chunk(path)
    assert loaded.compression == vxw.Compression.ZSTD
    assert np.array_equal(loaded.voxels, voxels)


def test_chunk_header_magic_and_version(tmp_path: Path) -> None:
    """Spec §4.4: magic=CHNK, format_version=0x0100."""
    voxels = _empty_chunk_voxels()
    chunk = vxw.Chunk(coord=(0, 0, 0), voxels=voxels)
    path = tmp_path / "0_0_0.chunk"
    vxw.write_chunk(path, chunk)
    raw = path.read_bytes()
    assert raw[0:4] == b"CHNK"
    fmt_version = struct.unpack("<H", raw[4:6])[0]
    assert fmt_version == 0x0100


def test_chunk_crc_corruption_raises(tmp_path: Path) -> None:
    """Contract C7: CRC32 mismatch must raise on read (Spec §4.7)."""
    voxels = _empty_chunk_voxels()
    voxels[0, 0, 0] = (1, 1, 0, 0)
    chunk = vxw.Chunk(coord=(0, 0, 0), voxels=voxels)
    path = tmp_path / "0_0_0.chunk"
    vxw.write_chunk(path, chunk)
    raw = bytearray(path.read_bytes())
    # Flip a byte in the payload (after header at offset 28)
    raw[30] ^= 0xFF
    path.write_bytes(raw)
    with pytest.raises(vxw.VxwCorruptError, match="CRC"):
        vxw.read_chunk(path)


def test_chunk_unknown_encoding_raises(tmp_path: Path) -> None:
    """Contract C8: unknown encoding must error, not be skipped (Spec §4.7)."""
    voxels = _empty_chunk_voxels()
    chunk = vxw.Chunk(coord=(0, 0, 0), voxels=voxels)
    path = tmp_path / "0_0_0.chunk"
    vxw.write_chunk(path, chunk)
    raw = bytearray(path.read_bytes())
    raw[20] = 99  # encoding byte → unknown
    # We must rewrite CRC because the test checks encoding-rejection, not CRC
    payload_bytes = struct.unpack("<I", raw[24:28])[0]
    payload = raw[28 : 28 + payload_bytes]
    new_crc = zlib.crc32(payload) & 0xFFFFFFFF
    raw[28 + payload_bytes : 28 + payload_bytes + 4] = struct.pack("<I", new_crc)
    path.write_bytes(raw)
    with pytest.raises(vxw.VxwFormatError, match="encoding"):
        vxw.read_chunk(path)


def test_chunk_rle_run_length_byte_caps_at_255(tmp_path: Path) -> None:
    """RleRun.length is uint8 (1-255); writer must split longer runs."""
    voxels = _empty_chunk_voxels(extent=16)  # 4096 voxels, one material → must split
    voxels[:, :, :] = (1, 1, 0, 0)
    chunk = vxw.Chunk(
        coord=(0, 0, 0),
        voxels=voxels,
        encoding=vxw.Encoding.RLE,
        compression=vxw.Compression.RAW,
    )
    path = tmp_path / "0_0_0.chunk"
    vxw.write_chunk(path, chunk)
    loaded = vxw.read_chunk(path)
    assert np.array_equal(loaded.voxels, voxels)


# ---------------------------------------------------------------------------
# Index file
# ---------------------------------------------------------------------------


def test_chunks_index_roundtrip(tmp_path: Path) -> None:
    entries = [
        vxw.ChunkIndexEntry(coord=(0, 0, 0), file_offset=0, size_bytes=128),
        vxw.ChunkIndexEntry(coord=(1, 0, 0), file_offset=0, size_bytes=256),
        vxw.ChunkIndexEntry(coord=(0, 0, 1), file_offset=0, size_bytes=512),
    ]
    path = tmp_path / "chunks.idx"
    vxw.write_chunks_index(path, entries)
    loaded = vxw.read_chunks_index(path)
    # Entries should round-trip as a set (order is morton-sorted internally)
    assert set((e.coord, e.size_bytes) for e in loaded) == set(
        (e.coord, e.size_bytes) for e in entries
    )


def test_chunks_index_magic(tmp_path: Path) -> None:
    path = tmp_path / "chunks.idx"
    vxw.write_chunks_index(path, [])
    raw = path.read_bytes()
    assert raw[0:4] == b"CIDX"


# ---------------------------------------------------------------------------
# World directory (the public API)
# ---------------------------------------------------------------------------


def _make_minimal_world() -> vxw.World:
    manifest = vxw.Manifest(
        world_id="aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee",
        voxel_size_meters=0.05,
        chunk_extent=32,
        bounds_chunks_min=(0, 0, 0),
        bounds_chunks_max=(1, 1, 1),
    )
    palette = vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=["empty"]),
            vxw.Material(
                id=1,
                name="concrete",
                color_rgb=(180, 180, 180),
                flags=["solid", "destructible"],
            ),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0, name="unknown", default_material=1),
            vxw.SemanticClass(id=1, name="wall", default_material=1),
        ],
        color_lut=[(0, 0, 0)],
    )
    voxels = _empty_chunk_voxels()
    voxels[0:5, 0:5, 0] = (1, 1, 0, 0)
    chunk = vxw.Chunk(coord=(0, 0, 0), voxels=voxels)
    return vxw.World(manifest=manifest, palette=palette, chunks={(0, 0, 0): chunk})


def test_world_roundtrip(tmp_path: Path) -> None:
    world = _make_minimal_world()
    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)
    loaded = vxw.read_world(out)
    assert loaded.manifest == world.manifest
    assert loaded.palette == world.palette
    assert set(loaded.chunks.keys()) == {(0, 0, 0)}
    assert np.array_equal(
        loaded.chunks[(0, 0, 0)].voxels, world.chunks[(0, 0, 0)].voxels
    )


def test_world_creates_expected_layout(tmp_path: Path) -> None:
    """Spec §4.1: world.vxw/ contains manifest.json, palette.json, chunks/, chunks.idx."""
    world = _make_minimal_world()
    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)
    assert (out / "manifest.json").is_file()
    assert (out / "palette.json").is_file()
    assert (out / "chunks.idx").is_file()
    assert (out / "chunks" / "0_0_0.chunk").is_file()


def test_world_palette_must_have_air(tmp_path: Path) -> None:
    manifest = vxw.Manifest(
        world_id="x",
        voxel_size_meters=0.05,
        chunk_extent=32,
        bounds_chunks_min=(0, 0, 0),
        bounds_chunks_max=(1, 1, 1),
    )
    palette = vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=["empty"]),
        ],
        semantic_classes=[vxw.SemanticClass(id=0, name="unknown", default_material=0)],
        color_lut=[],
    )
    world = vxw.World(manifest=manifest, palette=palette, chunks={})
    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)
    loaded = vxw.read_world(out)
    assert loaded.palette.materials[0].name == "air"


def test_missing_chunk_means_all_air(tmp_path: Path) -> None:
    """Contract C9: a chunk that isn't on disk == that region is all-air (Spec §4.7)."""
    world = _make_minimal_world()
    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)
    loaded = vxw.read_world(out)
    # We didn't write chunk (1,0,0); accessor should return all-air, not raise
    air_voxels = loaded.get_chunk_or_air((1, 0, 0))
    assert air_voxels.shape == (32, 32, 32)
    assert (air_voxels["material_id"] == 0).all()
