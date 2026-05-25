"""Tests for m3_adapter/pcd_to_vxw.py — the PCL .pcd → .vxw converter."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))
sys.path.insert(0, str(_PROJECT_ROOT / "m3_adapter"))

import vxw_format as vxw  # noqa: E402
from pcd_to_vxw import (  # noqa: E402
    build_palette,
    load_pcd_xyz,
    ros_zup_to_vxw_yup,
    voxelize_and_group,
)


def _write_ascii_pcd(path: Path, xyz: np.ndarray) -> None:
    """Write a minimal ASCII .pcd file (XYZ only). float32 fields."""
    n = len(xyz)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )
    body = "".join(f"{p[0]} {p[1]} {p[2]}\n" for p in xyz)
    path.write_text(header + body)


# ---------------------------------------------------------------------------
# load_pcd_xyz
# ---------------------------------------------------------------------------


def test_load_pcd_xyz_returns_correct_shape_and_values(tmp_path: Path) -> None:
    xyz_in = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    pcd = tmp_path / "three.pcd"
    _write_ascii_pcd(pcd, xyz_in)
    xyz_out = load_pcd_xyz(pcd)
    assert xyz_out.shape == (3, 3)
    np.testing.assert_allclose(xyz_out, xyz_in, atol=1e-5)


def test_load_pcd_xyz_returns_float64() -> None:
    # The adapter promises float64 downstream so numpy.floor/divide work cleanly.
    # (Guard against pypcd4 silently returning float32.)
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        pcd = Path(td) / "one.pcd"
        _write_ascii_pcd(pcd, np.array([[0.5, 0.5, 0.5]]))
        out = load_pcd_xyz(pcd)
    assert out.dtype == np.float64


# ---------------------------------------------------------------------------
# ros_zup_to_vxw_yup — coordinate convention transform (Contract §4.2)
# ---------------------------------------------------------------------------


def test_coord_swap_axes_correct() -> None:
    # ROS (X-forward, Y-left, Z-up) → vxw (X-right, Y-up, Z-back; Godot convention)
    # Transform:
    #   vxw.X = -ROS.Y  (right = -left)
    #   vxw.Y =  ROS.Z  (up   = up)
    #   vxw.Z = -ROS.X  (back = -forward)
    ros_pt = np.array([[1.0, 2.0, 3.0]])  # ROS: forward=1, left=2, up=3
    vxw_pt = ros_zup_to_vxw_yup(ros_pt)
    np.testing.assert_allclose(vxw_pt, [[-2.0, 3.0, -1.0]])


def test_coord_swap_preserves_handedness() -> None:
    # Three basis vectors in ROS that form a right-handed frame should still be
    # right-handed in vxw (det of basis matrix = +1, not -1).
    ros_basis = np.eye(3)
    vxw_basis = ros_zup_to_vxw_yup(ros_basis)
    assert np.linalg.det(vxw_basis) == pytest.approx(1.0, abs=1e-9)


# ---------------------------------------------------------------------------
# voxelize_and_group
# ---------------------------------------------------------------------------


def test_voxelize_single_point_makes_one_chunk_one_voxel() -> None:
    pts = np.array([[0.5, 0.5, 0.5]])
    chunks, bmin, bmax = voxelize_and_group(pts, voxel_size=0.10, chunk_extent=32)
    assert len(chunks) == 1
    assert (0, 0, 0) in chunks
    occ = int((chunks[(0, 0, 0)].voxels["material_id"] != 0).sum())
    assert occ == 1
    assert bmin == (0, 0, 0)
    assert bmax == (1, 1, 1)


def test_voxelize_dedupes_overlapping_points_within_one_voxel() -> None:
    # 100 points all inside the same 10cm voxel → 1 occupied voxel
    pts = np.tile(np.array([[0.01, 0.02, 0.03]]), (100, 1))
    chunks, _, _ = voxelize_and_group(pts, voxel_size=0.10, chunk_extent=32)
    total = sum(int((c.voxels["material_id"] != 0).sum()) for c in chunks.values())
    assert total == 1


def test_voxelize_spans_multiple_chunks() -> None:
    # Two points far apart should land in two different chunks
    # voxel_size=0.10m, chunk_extent=32 → chunk side = 3.2m
    pts = np.array([
        [0.0, 0.0, 0.0],     # chunk (0,0,0)
        [10.0, 0.0, 0.0],    # voxel x=100, chunk x = 100//32 = 3
    ])
    chunks, _, _ = voxelize_and_group(pts, voxel_size=0.10, chunk_extent=32)
    assert len(chunks) == 2
    assert (0, 0, 0) in chunks
    assert (3, 0, 0) in chunks


def test_voxelize_negative_coords_work() -> None:
    # Negative ROS coordinates should produce negative chunk coords
    pts = np.array([[-5.0, -5.0, -5.0]])
    chunks, bmin, _ = voxelize_and_group(pts, voxel_size=0.10, chunk_extent=32)
    assert len(chunks) == 1
    assert bmin[0] < 0 and bmin[1] < 0 and bmin[2] < 0


def test_voxelize_marks_wall_semantic() -> None:
    # Adapter currently assigns material=1 (concrete) and semantic=1 (wall)
    pts = np.array([[0.0, 0.0, 0.0]])
    chunks, _, _ = voxelize_and_group(pts, voxel_size=0.10, chunk_extent=32)
    arr = chunks[(0, 0, 0)].voxels
    mat = arr["material_id"][arr["material_id"] != 0]
    sem = arr["semantic_id"][arr["material_id"] != 0]
    assert (mat == 1).all()
    assert (sem == 1).all()


# ---------------------------------------------------------------------------
# End-to-end: synthetic .pcd → .vxw → read back → assert equal occupancy
# ---------------------------------------------------------------------------


def test_end_to_end_synthetic_roundtrip(tmp_path: Path) -> None:
    rng = np.random.default_rng(42)
    pts = rng.uniform(-5.0, 5.0, (500, 3))
    pcd = tmp_path / "synthetic.pcd"
    _write_ascii_pcd(pcd, pts)

    xyz = load_pcd_xyz(pcd)
    chunks, bmin, bmax = voxelize_and_group(
        xyz, voxel_size=0.20, chunk_extent=32,
        compression=vxw.Compression.GZIP,
    )
    palette = build_palette()
    manifest = vxw.Manifest(
        world_id="test-roundtrip",
        voxel_size_meters=0.20,
        chunk_extent=32,
        bounds_chunks_min=bmin,
        bounds_chunks_max=bmax,
    )
    world = vxw.World(manifest=manifest, palette=palette, chunks=chunks)

    out = tmp_path / "world.vxw"
    vxw.write_world(out, world)

    loaded = vxw.read_world(out)
    assert len(loaded.chunks) == len(chunks)
    in_occ = sum(int((c.voxels["material_id"] != 0).sum()) for c in chunks.values())
    out_occ = sum(int((c.voxels["material_id"] != 0).sum()) for c in loaded.chunks.values())
    assert in_occ == out_occ


def test_end_to_end_with_swap_preserves_point_count(tmp_path: Path) -> None:
    # 30 random points → after swap_yz → voxelize → distinct-voxel count <= 30
    rng = np.random.default_rng(7)
    pts = rng.uniform(0.0, 2.0, (30, 3))
    pcd = tmp_path / "rosframe.pcd"
    _write_ascii_pcd(pcd, pts)

    xyz = load_pcd_xyz(pcd)
    xyz_swapped = ros_zup_to_vxw_yup(xyz)
    chunks, _, _ = voxelize_and_group(xyz_swapped, voxel_size=0.05, chunk_extent=32)
    occ = sum(int((c.voxels["material_id"] != 0).sum()) for c in chunks.values())
    # With 5cm voxels and points spread in 2m³ cube, very unlikely to collide
    assert occ <= 30
    assert occ >= 28  # allow a couple of coincidental collisions
