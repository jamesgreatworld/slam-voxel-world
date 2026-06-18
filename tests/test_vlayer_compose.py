import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.overlay import Overlay, compose_structure


def _obs():
    m = ObsMap.new((6, 6, 6), np.zeros(3, np.int64), 0.1)
    m.logodds[2, 1, 2] = 5.0
    m.sem_label[2, 1, 2] = 3
    return m


def test_completed_add_fills_only_unknown():
    m = _obs()
    ov = Overlay()
    ov.add_voxel((3, 1, 2), sem=3, generator="floor_fill", binding="persistent")  # empty -> fill
    ov.add_voxel((2, 1, 2), sem=9, generator="floor_fill", binding="persistent")  # occupied -> NOT overridden
    occ, sem = compose_structure(m, ov)
    assert occ[3, 1, 2] and sem[3, 1, 2] == 3
    assert occ[2, 1, 2] and sem[2, 1, 2] == 3        # observation kept
    assert not m.occupancy_mask()[3, 1, 2]           # L0 untouched


def test_authored_add_and_remove_override():
    m = _obs()
    ov = Overlay()
    ov.add_voxel((2, 1, 2), op="remove", generator="manual", binding="independent")
    ov.add_voxel((4, 4, 4), sem=7, generator="manual", binding="independent")
    occ, sem = compose_structure(m, ov)
    assert not occ[2, 1, 2]
    assert occ[4, 4, 4] and sem[4, 4, 4] == 7
    assert m.occupancy_mask()[2, 1, 2]               # L0 untouched
