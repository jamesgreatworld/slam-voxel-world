import numpy as np
import vxw_format as vxw
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.generators.floor import FloorFill
from m3_adapter.vlayer.export import obsmap_to_completed_vxw

FLOOR = 3


def _obs_holey_floor():
    m = ObsMap.new((10, 6, 10), np.zeros(3, np.int64), 0.1)
    for x in range(2, 8):
        for z in range(2, 8):
            if (x, z) in [(4, 4), (5, 5)]:
                continue
            m.logodds[x, 1, z] = 5.0
            m.sem_label[x, 1, z] = FLOOR
            m.logodds[x, 3, z] = -5.0
    m.logodds[4, 3, 4] = -5.0
    m.logodds[5, 3, 5] = -5.0
    return m


def test_export_writes_solid_floor(tmp_path):
    m = _obs_holey_floor()
    out = tmp_path / "completed.vxw"
    base_occ = int(m.occupancy_mask().sum())
    overlay = obsmap_to_completed_vxw(m, out, generators=[FloorFill(close_radius=1)])
    assert len(overlay.voxels) >= 2
    world = vxw.read_world(out)
    assert world is not None
    assert (out / "overlay.npz").exists() and (out / "overlay.json").exists()
    assert int(m.occupancy_mask().sum()) == base_occ      # L0 untouched
