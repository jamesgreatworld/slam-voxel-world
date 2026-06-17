import numpy as np
from m3_adapter.gvd.objects import extract_objects, link_to_places, ObjectNode
from m3_adapter.gvd.graph import PlaceNode, PlacesGraph


def test_extract_two_chair_clusters():
    occ = np.zeros((40, 40, 40), dtype=bool)
    sem = np.zeros((40, 40, 40), dtype=np.uint8)
    # two separate chair(5) blobs + some wall(19) structure (ignored)
    occ[5:8, 5:8, 5:8] = True; sem[5:8, 5:8, 5:8] = 5
    occ[30:33, 30:33, 30:33] = True; sem[30:33, 30:33, 30:33] = 5
    occ[0, :, :] = True; sem[0, :, :] = 19      # wall structure
    objs = extract_objects(occ, sem, 0.1, np.zeros(3, np.int64),
                           label_names={5: "chair", 19: "wall"}, min_voxels=10)
    assert len(objs) == 2
    assert all(o.label == 5 and o.label_name == "chair" for o in objs)


def test_link_to_nearest_place():
    o = ObjectNode(idx=(10, 10, 10), label=5, label_name="chair", voxel_count=27,
                   bbox_min=(9,9,9), bbox_max=(11,11,11))
    nodes = [PlaceNode(idx=(10, 10, 10), clearance_m=1, degree=1, type="x"),
             PlaceNode(idx=(35, 35, 35), clearance_m=1, degree=1, type="x")]
    g = PlacesGraph(nodes, [], 0.1, np.zeros(3))
    link_to_places([o], g)
    assert o.place_id == 0   # nearest place is node 0
