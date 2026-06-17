"""Tests for obsmap_export.obsmap_to_world — the full ObsMap → vxw exporter.

The ObsMap is the single source of truth; this exporter derives semantic
voxels + furniture entities + spawn_hint without touching the bag or CSV files.
Stub inject extract_entities_fn / find_spawn_fn so the test has zero external
dependencies.
"""

import numpy as np
import vxw_format as vxw
from m3_adapter.obsmap import ObsMap
from m3_adapter.obsmap_export import obsmap_to_world


def _make_map():
    """Return a 40x40x40 ObsMap with 3 occupied semantic cells."""
    m = ObsMap.new((40, 40, 40), np.array([0, 0, 0], np.int64), 0.1)
    # Two wall voxels (super_id=19) and one floor voxel (super_id=3).
    for (x, y, z, L) in [(10, 10, 10, 19), (11, 10, 10, 19), (10, 5, 10, 3)]:
        m.logodds[x, y, z] = 3.0   # force occupied (above occ_thr=0.85)
        m.sem_label[x, y, z] = L
    return m


def _make_palette():
    """Minimal palette covering air / floor / wall."""
    return vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air",  color_rgb=(0, 0, 0),       flags=("empty",)),
            vxw.Material(id=1, name="m1",   color_rgb=(180, 180, 180), flags=("solid",)),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0,  name="unknown", default_material=1),
            vxw.SemanticClass(id=3,  name="floor",   default_material=1),
            vxw.SemanticClass(id=19, name="wall",    default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 180, 180)],
    )


def _no_entities(vc, lbl, vs, ln, eps, ms):
    """Stub: no entity extraction — keep all voxels as structure."""
    return [], np.ones(len(vc), dtype=bool)


def _spawn(vc, lbl, vs, floor_label=3):
    """Stub: return a fixed spawn point."""
    return [1.0, 0.5, 1.0, 0.0]


def test_obsmap_to_world_builds_semantic_chunks_and_entities(tmp_path):
    m = _make_map()
    pal = _make_palette()
    names = {0: "unknown", 3: "floor", 19: "wall"}

    world, ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_no_entities,
        find_spawn_fn=_spawn,
    )

    # All 3 semantic voxels must be written as occupied (material_id != 0).
    total = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in world.chunks.values()
    )
    assert total == 3, f"expected 3 occupied voxels, got {total}"

    # Spawn hint must be set.
    assert world.manifest.spawn_hint == [1.0, 0.5, 1.0, 0.0]

    # Both semantic ids (3=floor, 19=wall) must appear in the chunk data.
    sems = set()
    for c in world.chunks.values():
        sems.update(int(s) for s in np.unique(c.voxels["semantic_id"]) if s != 0)
    assert sems == {3, 19}, f"expected semantic ids {{3,19}}, got {sems}"

    # No entities returned when stub is in use.
    assert ents == []


def test_obsmap_to_world_no_spawn_fn():
    """When find_spawn_fn is None, spawn_hint is None."""
    m = _make_map()
    pal = _make_palette()
    names = {0: "unknown", 3: "floor", 19: "wall"}

    world, ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_no_entities,
        find_spawn_fn=None,
    )
    assert world.manifest.spawn_hint is None


def test_obsmap_to_world_label0_voxels_dropped():
    """Occupied cells with semantic label 0 (unknown) are excluded."""
    m = ObsMap.new((10, 10, 10), np.array([0, 0, 0], np.int64), 0.1)
    m.logodds[5, 5, 5] = 3.0   # occupied but label stays 0 (unknown)
    pal = _make_palette()
    names = {0: "unknown"}

    # All occupied voxels have label=0 so they should be dropped.
    try:
        world, ents = obsmap_to_world(
            m, names, pal,
            extract_entities_fn=_no_entities,
            find_spawn_fn=None,
        )
        total = sum(
            int((c.voxels["material_id"] != 0).sum()) for c in world.chunks.values()
        )
        assert total == 0
    except ValueError as exc:
        # obsmap_to_world raises ValueError when no semantic voxels remain
        assert "no occupied voxels" in str(exc).lower()


def test_obsmap_to_world_roundtrip(tmp_path):
    """World written with write_world can be read back and matches voxel count."""
    m = _make_map()
    pal = _make_palette()
    names = {0: "unknown", 3: "floor", 19: "wall"}

    world, _ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_no_entities,
        find_spawn_fn=_spawn,
    )

    out = tmp_path / "test_obs.vxw"
    vxw.write_world(out, world)
    loaded = vxw.read_world(out)

    total = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in loaded.chunks.values()
    )
    assert total == 3
    assert loaded.manifest.spawn_hint == [1.0, 0.5, 1.0, 0.0]


def test_obsmap_to_world_vmin_offset():
    """Non-zero vmin is correctly applied to world voxel coords (chunk keys shift)."""
    m = ObsMap.new((10, 10, 10), np.array([100, 200, 300], np.int64), 0.1)
    m.logodds[5, 5, 5] = 3.0
    m.sem_label[5, 5, 5] = 3
    pal = _make_palette()
    names = {3: "floor"}

    world, _ = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_no_entities,
        find_spawn_fn=None,
    )

    # world voxel coord = (5+100, 5+200, 5+300) = (105, 205, 305)
    # chunk coord at extent=32 = (3, 6, 9)
    assert (3, 6, 9) in world.chunks, \
        f"expected chunk (3,6,9) for vmin-shifted voxel; got keys {list(world.chunks.keys())}"
