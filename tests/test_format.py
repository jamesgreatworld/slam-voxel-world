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


def test_material_visual_defaults() -> None:
    """New visual fields must default to non-renderering values."""
    m = vxw.Material(id=1, name="concrete", color_rgb=(180, 180, 180), flags=("solid",))
    assert m.transparent is False
    assert m.emission_rgb == (0, 0, 0)
    assert m.emission_energy == 0.0
    assert m.metallic == 0.0
    assert m.roughness == 0.75


def test_material_visual_roundtrip(tmp_path: Path) -> None:
    """Glass + lamp + metal materials with full visual properties round-trip via JSON."""
    p = vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=("empty",)),
            vxw.Material(
                id=1, name="glass", color_rgb=(200, 230, 255), flags=("solid", "transparent"),
                transparent=True, roughness=0.1,
            ),
            vxw.Material(
                id=2, name="lamp", color_rgb=(255, 230, 150), flags=("solid", "emissive"),
                emission_rgb=(255, 220, 140), emission_energy=4.0, roughness=0.4,
            ),
            vxw.Material(
                id=3, name="steel", color_rgb=(140, 140, 150), flags=("solid", "metallic"),
                metallic=0.9, roughness=0.3,
            ),
        ],
        semantic_classes=[vxw.SemanticClass(id=0, name="unknown", default_material=1)],
        color_lut=[],
    )
    path = tmp_path / "palette.json"
    vxw.write_palette(path, p)
    loaded = vxw.read_palette(path)
    assert loaded == p
    assert loaded.materials[1].transparent is True
    assert loaded.materials[2].emission_energy == 4.0
    assert loaded.materials[3].metallic == 0.9


def test_material_legacy_palette_loads(tmp_path: Path) -> None:
    """A palette.json written without the new visual fields must load with defaults."""
    path = tmp_path / "palette.json"
    path.write_text(
        '{"materials":['
        '{"id":0,"name":"air","color_rgb":[0,0,0],"flags":["empty"]},'
        '{"id":1,"name":"concrete","color_rgb":[180,180,180],"flags":["solid"]}'
        '],"semantic_classes":[],"color_lut":[]}'
    )
    p = vxw.read_palette(path)
    assert p.materials[1].transparent is False
    assert p.materials[1].emission_energy == 0.0
    assert p.materials[1].roughness == 0.75


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


def test_world_overwrite_clears_stale_chunks(tmp_path: Path) -> None:
    """Writing world A then world B to the same path must produce only B's chunks.

    Regression: previously orphan chunk files from A were re-read as part of B,
    inflating the voxel count by a real-world factor of 1.5×.
    """
    out = tmp_path / "world.vxw"

    # World A: chunks at (0,0,0) and (5,0,0)
    voxels_a1 = _empty_chunk_voxels()
    voxels_a1[0, 0, 0] = (1, 1, 0, 0)
    voxels_a2 = _empty_chunk_voxels()
    voxels_a2[1, 2, 3] = (1, 1, 0, 0)
    world_a = vxw.World(
        manifest=vxw.Manifest(
            world_id="A", voxel_size_meters=0.05, chunk_extent=32,
            bounds_chunks_min=(0, 0, 0), bounds_chunks_max=(6, 1, 1),
        ),
        palette=vxw.Palette(
            materials=[
                vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=("empty",)),
                vxw.Material(id=1, name="x", color_rgb=(1, 1, 1), flags=("solid",)),
            ],
            semantic_classes=[vxw.SemanticClass(id=1, name="x", default_material=1)],
            color_lut=[],
        ),
        chunks={
            (0, 0, 0): vxw.Chunk(coord=(0, 0, 0), voxels=voxels_a1),
            (5, 0, 0): vxw.Chunk(coord=(5, 0, 0), voxels=voxels_a2),
        },
    )
    vxw.write_world(out, world_a)
    assert {p.name for p in (out / "chunks").iterdir()} == {
        "0_0_0.chunk", "5_0_0.chunk",
    }

    # World B: only chunk at (10,0,0) — completely different geometry
    voxels_b = _empty_chunk_voxels()
    voxels_b[2, 2, 2] = (1, 1, 0, 0)
    world_b = vxw.World(
        manifest=world_a.manifest,
        palette=world_a.palette,
        chunks={(10, 0, 0): vxw.Chunk(coord=(10, 0, 0), voxels=voxels_b)},
    )
    vxw.write_world(out, world_b)
    # Only world_b's chunk should remain on disk
    assert {p.name for p in (out / "chunks").iterdir()} == {"10_0_0.chunk"}

    loaded = vxw.read_world(out)
    assert set(loaded.chunks.keys()) == {(10, 0, 0)}


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


# ---------------------------------------------------------------------------
# Incremental edits: set_voxel + save_dirty
# ---------------------------------------------------------------------------


def test_set_voxel_in_existing_chunk() -> None:
    """set_voxel writes into the owning chunk and the value reads back."""
    world = _make_minimal_world()
    cell = np.array((1, 1, 2, 7), dtype=vxw.VOXEL_DTYPE)
    # World voxel (3, 4, 5) → chunk (0, 0, 0) local (3, 4, 5) at extent=32.
    world.set_voxel((3, 4, 5), cell)
    voxels = world.get_chunk_or_air((0, 0, 0))
    assert int(voxels[3, 4, 5]["material_id"]) == 1
    assert int(voxels[3, 4, 5]["semantic_id"]) == 1
    assert int(voxels[3, 4, 5]["state"]) == 2
    assert int(voxels[3, 4, 5]["color_palette_idx"]) == 7


def test_set_voxel_creates_missing_chunk() -> None:
    """Writing into a non-existent chunk creates an air-filled chunk first."""
    world = _make_minimal_world()
    assert (2, 0, 0) not in world.chunks
    # Extent=32 → world voxel (70, 1, 1) lives in chunk (2, 0, 0) local (6, 1, 1)
    world.set_voxel((70, 1, 1), (1, 1, 0, 0))
    assert (2, 0, 0) in world.chunks
    new_chunk = world.chunks[(2, 0, 0)]
    assert int(new_chunk.voxels[6, 1, 1]["material_id"]) == 1
    # All other voxels remain air.
    assert int(new_chunk.voxels[0, 0, 0]["material_id"]) == 0


def test_set_voxel_extends_bounds() -> None:
    """Creating a chunk outside the current bounds grows bounds_chunks_max/min."""
    world = _make_minimal_world()
    assert world.manifest.bounds_chunks_max == (1, 1, 1)
    # Voxel (320, -1, 0) → chunk (10, -1, 0) at extent=32.
    world.set_voxel((320, -1, 0), (1, 0, 0, 0))
    assert world.manifest.bounds_chunks_max[0] >= 10
    assert world.manifest.bounds_chunks_min[1] <= -1


def test_set_voxel_accepts_tuple_form() -> None:
    """Caller can pass a 4-tuple without constructing a structured array."""
    world = _make_minimal_world()
    world.set_voxel((10, 0, 0), (1, 1, 0, 0))
    voxels = world.get_chunk_or_air((0, 0, 0))
    assert int(voxels[10, 0, 0]["material_id"]) == 1
    assert int(voxels[10, 0, 0]["semantic_id"]) == 1


def test_save_dirty_writes_only_dirty_chunks(tmp_path: Path) -> None:
    """Non-dirty chunk files must be untouched by save_dirty (byte-identical)."""
    # Two chunks in the world.
    voxels_a = _empty_chunk_voxels()
    voxels_a[0, 0, 0] = (1, 1, 0, 0)
    voxels_b = _empty_chunk_voxels()
    voxels_b[1, 1, 1] = (1, 1, 0, 0)
    world = vxw.World(
        manifest=vxw.Manifest(
            world_id="dirty-test", voxel_size_meters=0.05, chunk_extent=32,
            bounds_chunks_min=(0, 0, 0), bounds_chunks_max=(1, 0, 0),
        ),
        palette=vxw.Palette(
            materials=[
                vxw.Material(id=0, name="air", color_rgb=(0, 0, 0), flags=("empty",)),
                vxw.Material(id=1, name="x", color_rgb=(1, 1, 1), flags=("solid",)),
            ],
            semantic_classes=[vxw.SemanticClass(id=1, name="x", default_material=1)],
            color_lut=[],
        ),
        chunks={
            (0, 0, 0): vxw.Chunk(coord=(0, 0, 0), voxels=voxels_a),
            (1, 0, 0): vxw.Chunk(coord=(1, 0, 0), voxels=voxels_b),
        },
    )
    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)

    loaded = vxw.read_world(out)
    untouched_bytes = (out / "chunks" / "1_0_0.chunk").read_bytes()

    # Mutate only chunk (0,0,0) and save.
    loaded.set_voxel((2, 2, 2), (1, 1, 0, 0))
    written = loaded.save_dirty(out)

    assert written == {(0, 0, 0)}
    # The non-dirty chunk's bytes are unchanged.
    assert (out / "chunks" / "1_0_0.chunk").read_bytes() == untouched_bytes


def test_save_dirty_updates_index(tmp_path: Path) -> None:
    """A new chunk created via set_voxel must round-trip via chunks.idx + disk."""
    world = _make_minimal_world()
    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)

    loaded = vxw.read_world(out)
    # Force creation of a new chunk at (3, 0, 0): voxel (96, 0, 0) at extent 32.
    loaded.set_voxel((96, 0, 0), (1, 1, 0, 0))
    written = loaded.save_dirty(out)
    assert (3, 0, 0) in written

    # Index should now mention the new chunk coord.
    idx_entries = vxw.read_chunks_index(out / "chunks.idx")
    assert (3, 0, 0) in {e.coord for e in idx_entries}

    # And re-reading the world picks up the new chunk with the right value.
    reloaded = vxw.read_world(out)
    assert (3, 0, 0) in reloaded.chunks
    assert int(reloaded.chunks[(3, 0, 0)].voxels[0, 0, 0]["material_id"]) == 1


def test_save_dirty_rejects_uninitialized_path(tmp_path: Path) -> None:
    """Calling save_dirty on a directory without manifest.json must raise."""
    world = _make_minimal_world()
    world.set_voxel((0, 0, 0), (1, 1, 0, 0))
    empty_dir = tmp_path / "no_manifest_here"
    empty_dir.mkdir()
    with pytest.raises(vxw.VxwFormatError, match="manifest"):
        world.save_dirty(empty_dir)


def test_save_dirty_returns_written_set(tmp_path: Path) -> None:
    """The return value must equal the set of coords actually flushed."""
    world = _make_minimal_world()
    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)

    loaded = vxw.read_world(out)
    # Touch three distinct chunks: (0,0,0) existing, (2,0,0) new, (0,3,0) new.
    loaded.set_voxel((1, 1, 1), (1, 1, 0, 0))
    loaded.set_voxel((64, 0, 0), (1, 1, 0, 0))
    loaded.set_voxel((0, 96, 0), (1, 1, 0, 0))

    written = loaded.save_dirty(out)
    assert written == {(0, 0, 0), (2, 0, 0), (0, 3, 0)}
    # Dirty set is cleared after save.
    second = loaded.save_dirty(out)
    assert second == set()
