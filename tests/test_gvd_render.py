import json
import numpy as np
import vxw_format as vxw
from m3_adapter.common import build_concrete_palette
from m3_adapter.gvd.graph import PlaceNode, PlacesGraph
from m3_adapter.gvd import render


def _world():
    man = vxw.Manifest(world_id="t", voxel_size_meters=0.5, chunk_extent=32,
                       bounds_chunks_min=(0,0,0), bounds_chunks_max=(1,1,1))
    arr = np.zeros((32,32,32), dtype=vxw.VOXEL_DTYPE)
    arr["material_id"][0,0,0] = 1
    return vxw.World(manifest=man, palette=build_concrete_palette(),
                     chunks={(0,0,0): vxw.Chunk(coord=(0,0,0), voxels=arr)})


def test_stamp_skeleton_adds_material_and_voxels():
    w = _world()
    gvd = np.zeros((5,5,5), dtype=bool); gvd[2,2,2] = True
    n0 = len(w.palette.materials)
    mid = render.stamp_skeleton(w, gvd, np.zeros(3, np.int64))
    assert len(w.palette.materials) == n0 + 1
    assert any((c.voxels["material_id"] == mid).any() for c in w.chunks.values())


def test_stamp_room_nodes_and_json(tmp_path):
    w = _world()
    nodes = [PlaceNode(idx=(3,3,3), clearance_m=0.5, degree=1, type="endpoint", room=0),
             PlaceNode(idx=(6,6,6), clearance_m=0.5, degree=1, type="endpoint", room=1)]
    g = PlacesGraph(nodes, [(0,1,2.0)], 0.5, np.zeros(3, np.int64))
    render.stamp_room_nodes(w, g)
    assert any(m.name.startswith("room_") for m in w.palette.materials)
    p = tmp_path / "g.json"
    render.write_graph_json(p, g)
    doc = json.loads(p.read_text())
    assert doc["num_rooms"] == 2
    assert all("room" in n and "pos_m" in n for n in doc["nodes"])
