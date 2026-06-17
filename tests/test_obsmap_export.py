import numpy as np
import vxw_format as vxw
from m3_adapter.obsmap_export import occupancy_to_vxw


def test_occupancy_to_vxw_roundtrip(tmp_path):
    occ = np.zeros((40, 40, 40), dtype=bool)
    occ[10:15, 10:15, 10:15] = True   # 125 cells
    out = tmp_path / "g.vxw"
    occupancy_to_vxw(occ, np.array([-5, -5, -5], np.int64), 0.1, out)
    w = vxw.read_world(out)
    total = sum(int((c.voxels["material_id"] != 0).sum()) for c in w.chunks.values())
    assert total == 125
    assert w.manifest.voxel_size_meters == 0.1
