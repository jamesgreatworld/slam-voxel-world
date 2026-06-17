"""test_obsmap_export_semantic.py — verify that occupancy_to_vxw writes per-cell
semantic_id when a semantic_grid and palette are provided.

The hydra cfg YAML/CSV are NOT bundled in this repo (they live in the
E:/aros_slam_ws/hydra_ws workspace on the developer machine).  Rather than
skip the test entirely, we build a minimal inline palette that covers the two
super_ids exercised by the test (5=chair, 19=wall) plus air (0).  This fully
exercises the semantic export path without needing the external config files.
"""
from __future__ import annotations

import numpy as np
import pytest
import vxw_format as vxw

from m3_adapter.obsmap_export import occupancy_to_vxw


def _make_minimal_palette(super_ids: list[int]) -> vxw.Palette:
    """Build a vxw.Palette containing air (0) plus the given super_ids."""
    ids = sorted({0} | set(super_ids))
    materials = []
    semclasses = []
    color_lut = []
    for sid in ids:
        if sid == 0:
            name, flags, color = "air", ("empty",), (0, 0, 0)
        elif sid == 5:
            name, flags, color = "chair", ("solid", "destructible", "object"), (220, 50, 50)
        elif sid == 19:
            name, flags, color = "wall", ("solid", "destructible"), (210, 200, 180)
        else:
            name, flags, color = f"id{sid}", ("solid", "destructible"), (128, 128, 128)
        materials.append(vxw.Material(id=sid, name=name, color_rgb=color, flags=flags))
        semclasses.append(vxw.SemanticClass(id=sid, name=name, default_material=sid))
        color_lut.append(color)
    return vxw.Palette(materials=materials, semantic_classes=semclasses, color_lut=color_lut)


def test_semantic_export_writes_per_cell_label(tmp_path):
    """semantic_grid values are preserved as semantic_id in the written .vxw."""
    occ = np.zeros((40, 40, 40), dtype=bool)
    occ[10, 10, 10] = True
    occ[12, 12, 12] = True
    sem = np.zeros((40, 40, 40), dtype=np.uint8)
    sem[10, 10, 10] = 5    # chair
    sem[12, 12, 12] = 19   # wall

    pal = _make_minimal_palette([5, 19])
    out = tmp_path / "g.vxw"
    occupancy_to_vxw(occ, np.zeros(3, np.int64), 0.1, out,
                     semantic_grid=sem, palette=pal)

    w = vxw.read_world(out)
    # Collect world-voxel-coord -> semantic_id for all occupied cells
    found: dict[tuple[int, int, int], int] = {}
    for coord, ch in w.chunks.items():
        v = ch.voxels
        nz = np.argwhere(v["material_id"] != 0)
        for (x, y, z) in nz:
            wv = (
                np.array(coord) * w.manifest.chunk_extent + np.array([x, y, z])
            )
            found[tuple(int(t) for t in wv)] = int(v["semantic_id"][x, y, z])

    assert found.get((10, 10, 10)) == 5,  f"expected 5 (chair) at (10,10,10), got {found}"
    assert found.get((12, 12, 12)) == 19, f"expected 19 (wall) at (12,12,12), got {found}"


def test_semantic_export_unknown_cell_gets_material_fallback(tmp_path):
    """A cell occupied but with semantic label 0 still renders (material_id=1)."""
    occ = np.zeros((20, 20, 20), dtype=bool)
    occ[5, 5, 5] = True
    sem = np.zeros((20, 20, 20), dtype=np.uint8)
    # sem[5,5,5] = 0 (unknown) — default

    pal = _make_minimal_palette([5])
    out = tmp_path / "g.vxw"
    occupancy_to_vxw(occ, np.zeros(3, np.int64), 0.1, out,
                     semantic_grid=sem, palette=pal)

    w = vxw.read_world(out)
    found_mat = {}
    for coord, ch in w.chunks.items():
        v = ch.voxels
        nz = np.argwhere(v["material_id"] != 0)
        for (x, y, z) in nz:
            wv = np.array(coord) * w.manifest.chunk_extent + np.array([x, y, z])
            found_mat[tuple(int(t) for t in wv)] = int(v["material_id"][x, y, z])

    # Cell (5,5,5) must still be present with material_id=1 (fallback)
    assert found_mat.get((5, 5, 5)) == 1, f"expected fallback material_id=1, got {found_mat}"


def test_non_semantic_path_unchanged(tmp_path):
    """Without semantic_grid, occupancy_to_vxw uses concrete palette (material_id=1)."""
    occ = np.zeros((40, 40, 40), dtype=bool)
    occ[10:15, 10:15, 10:15] = True  # 125 cells
    out = tmp_path / "g.vxw"
    occupancy_to_vxw(occ, np.array([-5, -5, -5], np.int64), 0.1, out)
    w = vxw.read_world(out)
    total = sum(int((c.voxels["material_id"] != 0).sum()) for c in w.chunks.values())
    assert total == 125
    assert w.manifest.voxel_size_meters == 0.1
