import numpy as np
from m3_adapter.vlayer.overlay import VoxelDelta, Overlay


def test_add_and_iter():
    ov = Overlay()
    ov.add_voxel((1, 2, 3), sem=3, generator="floor_fill", binding="persistent")
    assert len(ov.voxels) == 1
    d = ov.voxels[0]
    assert d.idx == (1, 2, 3) and d.op == "add" and d.sem == 3
    assert d.generator == "floor_fill" and d.binding == "persistent"


def test_save_load_roundtrip(tmp_path):
    ov = Overlay()
    ov.add_voxel((1, 2, 3), sem=3, generator="floor_fill", binding="persistent")
    ov.add_voxel((4, 5, 6), sem=0, generator="manual", binding="independent", op="remove")
    npz = tmp_path / "overlay.npz"
    js = tmp_path / "overlay.json"
    ov.save(npz, js)
    ov2 = Overlay.load(npz, js)
    assert len(ov2.voxels) == 2
    assert ov2.voxels[0].idx == (1, 2, 3) and ov2.voxels[0].generator == "floor_fill"
    assert ov2.voxels[1].op == "remove" and ov2.voxels[1].binding == "independent"
