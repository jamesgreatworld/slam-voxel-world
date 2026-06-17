import numpy as np
from m3_adapter.gvd.graph import PlaceNode, PlacesGraph
from m3_adapter.gvd.objects import ObjectNode
from m3_adapter.gvd.scene_graph import build_scene_graph, consolidate_fragments


def _pg():
    return PlacesGraph([PlaceNode(idx=(10,2,10), clearance_m=1, degree=1, type="x", room=0)],
                       [], 0.1, np.zeros(3, np.int64))


def _obj(cx, label, name):
    return ObjectNode(idx=(cx,5,10), label=label, label_name=name, voxel_count=100,
                      bbox_min=(cx-1,3,8), bbox_max=(cx+1,7,12), place_id=0)


def test_persistent_close_same_class_fragments_merge():
    pg = _pg()
    # two table fragments 1 voxel (0.1 m) apart -> within gap_m=0.15
    a = _obj(10, 16, "table"); b = _obj(13, 16, "table")  # bbox_max_a x=11 -> 1.1m, bbox_min_b x=12 -> 1.2m, gap 0.1m
    sg = build_scene_graph(pg, [a, b])
    # mark both persistent
    for n in sg.nodes_by_layer("object"):
        n.attrs["seen_count"] = 3
    before = len(sg.nodes_by_layer("object"))
    merges = consolidate_fragments(sg, gap_m=0.15, min_persist=2)
    after = len(sg.nodes_by_layer("object"))
    assert merges == 1 and after == before - 1
    # merged object voxel_count summed
    assert sg.nodes_by_layer("object")[0].attrs["voxel_count"] == 200


def test_transient_fragments_not_merged():
    pg = _pg()
    a = _obj(10, 16, "table"); b = _obj(13, 16, "table")
    sg = build_scene_graph(pg, [a, b])
    # only seen once -> below min_persist -> NOT merged
    merges = consolidate_fragments(sg, gap_m=0.15, min_persist=2)
    assert merges == 0


def test_far_fragments_not_merged():
    pg = _pg()
    a = _obj(10, 16, "table"); b = _obj(40, 16, "table")  # 3 m apart
    sg = build_scene_graph(pg, [a, b])
    for n in sg.nodes_by_layer("object"): n.attrs["seen_count"] = 5
    assert consolidate_fragments(sg, gap_m=0.15, min_persist=2) == 0
