import numpy as np
from m3_adapter.gvd.objects import extract_objects, ObjectNode
from m3_adapter.gvd.graph import PlaceNode, PlacesGraph
from m3_adapter.gvd.scene_graph import build_scene_graph, merge_observation


def test_shape_sig_distinguishes_tall_vs_flat():
    occ = np.zeros((40, 40, 40), dtype=bool); sem = np.zeros((40, 40, 40), np.uint8)
    # a TALL object (along y) and a FLAT object (along x), both class 9 (furniture)
    occ[10, 5:25, 10] = True; sem[10, 5:25, 10] = 9           # tall (y-extent big)
    occ[25:39, 30, 30] = True; sem[25:39, 30, 30] = 9         # flat-x
    objs = extract_objects(occ, sem, 0.1, np.zeros(3, np.int64),
                           label_names={9: "furniture"}, min_voxels=5)
    assert len(objs) == 2
    # each shape_sig has a dominant axis (largest principal extent) > the others
    for o in objs:
        s = o.shape_sig
        assert len(s) == 3 and s[0] >= s[1] >= s[2] and s[0] > 0.2


def _pg():
    return PlacesGraph([PlaceNode(idx=(10,2,10), clearance_m=1, degree=1, type="x", room=0)],
                       [], 0.1, np.zeros(3, np.int64))


def _obj(cx, label, name, sig, pid=0):
    o = ObjectNode(idx=(cx,5,10), label=label, label_name=name, voxel_count=100,
                   bbox_min=(cx-2,3,8), bbox_max=(cx+2,7,12), place_id=pid)
    o.shape_sig = sig
    return o


def test_shape_disambiguates_same_class_assignment():
    pg = _pg()
    # two chairs (class 5): SMALL at x=10, BIG at x=14
    small = _obj(10, 5, "chair", (0.2, 0.15, 0.1))
    big   = _obj(14, 5, "chair", (0.6, 0.5, 0.4))
    sg1 = build_scene_graph(pg, [small, big])
    small_id = sg1.get([n.id for n in sg1.nodes_by_layer("object")
                        if tuple(n.attrs["shape"]) == [0.2,0.15,0.1] or n.attrs["shape"]==[0.2,0.15,0.1]][0]).id if False else None
    # grab ids by shape
    def id_by_shape(sg, sig):
        for n in sg.nodes_by_layer("object"):
            if [round(x,3) for x in n.attrs["shape"]] == [round(x,3) for x in sig]:
                return n.id
        raise AssertionError("not found")
    small_id = id_by_shape(sg1, (0.2,0.15,0.1))
    big_id   = id_by_shape(sg1, (0.6,0.5,0.4))
    # observation 2: BIG moved NEAR where small was (x=11), small moved to x=13.
    # distance-only would mis-assign small->big@11. shape must fix it.
    fresh = [_obj(11, 5, "chair", (0.6,0.5,0.4)),   # the BIG one, now near x=11
             _obj(13, 5, "chair", (0.2,0.15,0.1))]  # the SMALL one, now x=13
    sg2, stats = merge_observation(sg1, pg, fresh, match_radius_m=5.0,
                                   max_misses=3, shape_weight=3.0)
    # the small chair identity should follow its SHAPE to fresh x=13, not the closer x=11
    sid = id_by_shape(sg2, (0.2,0.15,0.1))
    assert sid == small_id                  # identity preserved by shape, not proximity
    bid = id_by_shape(sg2, (0.6,0.5,0.4))
    assert bid == big_id
    # and the small one is now at x=13 (1.3 m)
    assert sg2.get(sid).pos_m[0] > 1.2
