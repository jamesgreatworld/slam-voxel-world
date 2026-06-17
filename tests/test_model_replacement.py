"""Tests for the object-class → mc_item pre-built model replacement.

Verifies that obsmap_to_world sets custom_meta["mc_item"] on entities whose
super_id is in SUPER_ID_TO_MC_ITEM, so entity_renderer.gd renders a complete
pre-built model (chair/table/lamp) instead of a generic OBB box.
"""

import numpy as np
import vxw_format as vxw
from m3_adapter.obsmap import ObsMap
from m3_adapter.obsmap_export import obsmap_to_world, SUPER_ID_TO_MC_ITEM


def test_chair_entity_gets_mc_item_preset(tmp_path):
    m = ObsMap.new((40, 40, 40), np.zeros(3, np.int64), 0.1)
    for (x, y, z) in [(10, 10, 10), (11, 10, 10)]:
        m.logodds[x, y, z] = 3.0
        m.sem_label[x, y, z] = 5   # chair
    m.logodds[10, 5, 10] = 3.0
    m.sem_label[10, 5, 10] = 3     # floor
    pal = vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air",  color_rgb=(0, 0, 0),         flags=("empty",)),
            vxw.Material(id=1, name="m1",   color_rgb=(180, 180, 180),   flags=("solid",)),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0, name="unknown", default_material=1),
            vxw.SemanticClass(id=3, name="floor",   default_material=1),
            vxw.SemanticClass(id=5, name="chair",   default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 180, 180)],
    )
    names = {0: "unknown", 3: "floor", 5: "chair"}

    # Stub: return one chair entity (label=5) with empty custom_meta; keep_mask all True.
    def _entities(vc, lbl, vs, ln, eps, ms):
        e = vxw.Entity(
            id="e1",
            label=5,
            label_name="chair",
            position=(1.0, 1.0, 1.0),
            rotation=(0, 0, 0, 1),
            bbox_dims=(0.5, 0.9, 0.5),
            voxel_count=2,
            custom_meta={},
        )
        return [e], np.ones(len(vc), dtype=bool)

    def _spawn(vc, lbl, vs, floor_label=3):
        return [1.0, 0.5, 1.0, 0.0]

    world, ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_entities,
        find_spawn_fn=_spawn,
    )

    assert ents[0].custom_meta.get("mc_item") == "chair"
    # An unmapped label would NOT get mc_item.
    assert SUPER_ID_TO_MC_ITEM[5] == "chair" and 99 not in SUPER_ID_TO_MC_ITEM


def test_table_entity_gets_mc_item_preset():
    """super_id=16 (table) also maps to the 'table' preset."""
    m = ObsMap.new((40, 40, 40), np.zeros(3, np.int64), 0.1)
    m.logodds[5, 5, 5] = 3.0
    m.sem_label[5, 5, 5] = 16   # table
    m.logodds[5, 2, 5] = 3.0
    m.sem_label[5, 2, 5] = 3    # floor so voxels remain
    pal = vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0),       flags=("empty",)),
            vxw.Material(id=1, name="m1",  color_rgb=(180, 180, 180), flags=("solid",)),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0,  name="unknown", default_material=1),
            vxw.SemanticClass(id=3,  name="floor",   default_material=1),
            vxw.SemanticClass(id=16, name="table",   default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 180, 180)],
    )
    names = {0: "unknown", 3: "floor", 16: "table"}

    def _entities(vc, lbl, vs, ln, eps, ms):
        e = vxw.Entity(
            id="e2", label=16, label_name="table",
            position=(0.5, 0.5, 0.5), rotation=(0, 0, 0, 1),
            bbox_dims=(1.0, 0.8, 0.6), voxel_count=1,
            custom_meta={},
        )
        return [e], np.ones(len(vc), dtype=bool)

    def _spawn(vc, lbl, vs, floor_label=3):
        return [0.5, 0.1, 0.5, 0.0]

    world, ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_entities,
        find_spawn_fn=_spawn,
    )

    assert ents[0].custom_meta.get("mc_item") == "table"


def test_lamp_entity_gets_mc_item_preset():
    """super_id=11 (lamp) maps to the 'lamp' preset."""
    m = ObsMap.new((40, 40, 40), np.zeros(3, np.int64), 0.1)
    m.logodds[3, 3, 3] = 3.0
    m.sem_label[3, 3, 3] = 11   # lamp
    m.logodds[3, 1, 3] = 3.0
    m.sem_label[3, 1, 3] = 3    # floor
    pal = vxw.Palette(
        materials=[
            vxw.Material(id=0,  name="air",  color_rgb=(0, 0, 0),       flags=("empty",)),
            vxw.Material(id=1,  name="m1",   color_rgb=(180, 180, 180), flags=("solid",)),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0,  name="unknown", default_material=1),
            vxw.SemanticClass(id=3,  name="floor",   default_material=1),
            vxw.SemanticClass(id=11, name="lamp",    default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 180, 180)],
    )
    names = {0: "unknown", 3: "floor", 11: "lamp"}

    def _entities(vc, lbl, vs, ln, eps, ms):
        e = vxw.Entity(
            id="e3", label=11, label_name="lamp",
            position=(0.3, 0.3, 0.3), rotation=(0, 0, 0, 1),
            bbox_dims=(0.3, 1.5, 0.3), voxel_count=1,
            custom_meta={},
        )
        return [e], np.ones(len(vc), dtype=bool)

    def _spawn(vc, lbl, vs, floor_label=3):
        return [0.3, 0.1, 0.3, 0.0]

    world, ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_entities,
        find_spawn_fn=_spawn,
    )

    assert ents[0].custom_meta.get("mc_item") == "lamp"


def test_unmapped_entity_gets_no_mc_item():
    """An entity whose super_id is not in the map keeps generic OBB rendering."""
    m = ObsMap.new((40, 40, 40), np.zeros(3, np.int64), 0.1)
    m.logodds[6, 6, 6] = 3.0
    m.sem_label[6, 6, 6] = 9   # furniture (ambiguous, not mapped)
    m.logodds[6, 2, 6] = 3.0
    m.sem_label[6, 2, 6] = 3   # floor
    pal = vxw.Palette(
        materials=[
            vxw.Material(id=0, name="air", color_rgb=(0, 0, 0),       flags=("empty",)),
            vxw.Material(id=1, name="m1",  color_rgb=(180, 180, 180), flags=("solid",)),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0, name="unknown",   default_material=1),
            vxw.SemanticClass(id=3, name="floor",     default_material=1),
            vxw.SemanticClass(id=9, name="furniture", default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 180, 180)],
    )
    names = {0: "unknown", 3: "floor", 9: "furniture"}

    def _entities(vc, lbl, vs, ln, eps, ms):
        e = vxw.Entity(
            id="e4", label=9, label_name="furniture",
            position=(0.6, 0.6, 0.6), rotation=(0, 0, 0, 1),
            bbox_dims=(1.0, 1.0, 1.0), voxel_count=1,
            custom_meta={},
        )
        return [e], np.ones(len(vc), dtype=bool)

    def _spawn(vc, lbl, vs, floor_label=3):
        return [0.6, 0.1, 0.6, 0.0]

    world, ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_entities,
        find_spawn_fn=_spawn,
    )

    assert "mc_item" not in ents[0].custom_meta


def test_custom_mc_item_map_override():
    """mc_item_map param overrides the default SUPER_ID_TO_MC_ITEM."""
    m = ObsMap.new((40, 40, 40), np.zeros(3, np.int64), 0.1)
    m.logodds[7, 7, 7] = 3.0
    m.sem_label[7, 7, 7] = 42   # hypothetical new label
    m.logodds[7, 2, 7] = 3.0
    m.sem_label[7, 2, 7] = 3
    pal = vxw.Palette(
        materials=[
            vxw.Material(id=0,  name="air",  color_rgb=(0, 0, 0),       flags=("empty",)),
            vxw.Material(id=1,  name="m1",   color_rgb=(180, 180, 180), flags=("solid",)),
        ],
        semantic_classes=[
            vxw.SemanticClass(id=0,  name="unknown", default_material=1),
            vxw.SemanticClass(id=3,  name="floor",   default_material=1),
            vxw.SemanticClass(id=42, name="widget",  default_material=1),
        ],
        color_lut=[(0, 0, 0), (180, 180, 180)],
    )
    names = {0: "unknown", 3: "floor", 42: "widget"}

    def _entities(vc, lbl, vs, ln, eps, ms):
        e = vxw.Entity(
            id="e5", label=42, label_name="widget",
            position=(0.7, 0.7, 0.7), rotation=(0, 0, 0, 1),
            bbox_dims=(0.5, 0.5, 0.5), voxel_count=1,
            custom_meta={},
        )
        return [e], np.ones(len(vc), dtype=bool)

    def _spawn(vc, lbl, vs, floor_label=3):
        return [0.7, 0.1, 0.7, 0.0]

    custom_map = {42: "bed"}   # map the new label to "bed" preset
    world, ents = obsmap_to_world(
        m, names, pal,
        extract_entities_fn=_entities,
        find_spawn_fn=_spawn,
        mc_item_map=custom_map,
    )

    assert ents[0].custom_meta.get("mc_item") == "bed"
