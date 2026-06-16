import json
import numpy as np
import vxw_format as vxw
from m3_adapter.common import build_concrete_palette
from m3_adapter.gvd.pipeline import GvdConfig, run


def _make_box_vxw(tmp_path, extent=32, vsize=0.5):
    pal = build_concrete_palette()
    arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
    occ = np.zeros((21, 21, 21), dtype=bool)
    occ[0] = occ[20] = True
    occ[:, 0] = occ[:, 20] = True
    occ[:, :, 0] = occ[:, :, 20] = True
    ii = np.argwhere(occ)
    arr["material_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    arr["semantic_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    man = vxw.Manifest(world_id="t", voxel_size_meters=vsize, chunk_extent=extent,
                       bounds_chunks_min=(0, 0, 0), bounds_chunks_max=(1, 1, 1))
    w = vxw.World(manifest=man, palette=pal,
                  chunks={(0, 0, 0): vxw.Chunk(coord=(0, 0, 0), voxels=arr)})
    p = tmp_path / "box.vxw"
    vxw.write_world(p, w)
    return p


def test_pipeline_field_only(tmp_path):
    src = _make_box_vxw(tmp_path)
    out = tmp_path / "o.vxw"
    stats = run(GvdConfig(str(src), str(out), band_max=1.0, thin=True))
    assert stats["gvd_voxels"] > 0
    w = vxw.read_world(out)
    assert "gvd_skeleton" in [m.name for m in w.palette.materials]


def test_pipeline_graph_rooms_cleanup(tmp_path):
    src = _make_box_vxw(tmp_path)
    out = tmp_path / "o.vxw"
    stats = run(GvdConfig(str(src), str(out), band_max=1.0, thin=True,
                          rooms=True, prune_spurs_m=0.3, merge_close_m=0.2,
                          room_resolution=1.0))
    assert stats["graph_nodes"] >= 1
    assert stats["num_rooms"] >= 1
    assert stats["graph_nodes"] <= stats["graph_nodes_raw"]  # cleanup didn't grow it
    g = json.loads((tmp_path / "o.graph.json").read_text())
    assert "num_rooms" in g and all("room" in n for n in g["nodes"])
    w = vxw.read_world(out)
    assert any(m.name.startswith("room_") for m in w.palette.materials)
