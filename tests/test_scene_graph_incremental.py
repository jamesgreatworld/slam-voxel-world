import numpy as np
from m3_adapter.gvd.graph import PlaceNode, PlacesGraph
from m3_adapter.gvd.objects import ObjectNode
from m3_adapter.gvd.scene_graph import SceneGraph, SceneNode, build_scene_graph, merge_observation


def _pg(rooms=(0,)):
    nodes = [PlaceNode(idx=(10*(i+1), 2, 10), clearance_m=1, degree=1, type="x", room=r)
             for i, r in enumerate(rooms)]
    return PlacesGraph(nodes, [], 0.1, np.zeros(3, np.int64))


def _obj(cx, label, name, pid=0):
    return ObjectNode(idx=(cx, 5, 10), label=label, label_name=name, voxel_count=100,
                      bbox_min=(cx-2, 3, 8), bbox_max=(cx+2, 7, 12), place_id=pid)


def test_save_load_restores_structure(tmp_path):
    g = SceneGraph()
    g.add_node(SceneNode(id="building:0", layer="building"))
    g.add_node(SceneNode(id="room:0", layer="room"), parent_id="building:0")
    g.add_node(SceneNode(id="o1", layer="object", label="table"), parent_id="room:0")
    p = tmp_path / "sg.json"
    g.save(p)
    g2 = SceneGraph.load(p)
    # query works on in-memory structure (pointers), not JSON scanning
    assert g2.parent("o1").id == "room:0"
    assert [c.id for c in g2.children("room:0")] == ["o1"]
    assert [a.id for a in g2.ancestors("o1")] == ["room:0", "building:0"]


def test_merge_moved_kept_added_removed():
    pg = _pg(rooms=(0,))
    # build #1: a chair at x=10 and a lamp at x=10 (different class) in room 0
    sg1 = build_scene_graph(pg, [_obj(10, 5, "chair"), _obj(10, 11, "lamp")])
    chair_id = [n.id for n in sg1.nodes_by_layer("object") if n.label == "chair"][0]
    # observation 2: chair MOVED to x=12 (within match radius), lamp GONE, a new table appears
    fresh_objs = [_obj(12, 5, "chair"), _obj(40, 16, "table", pid=0)]
    sg2, stats = merge_observation(sg1, pg, fresh_objs, match_radius_m=0.5, max_misses=0)
    obj_labels = sorted(n.label for n in sg2.nodes_by_layer("object"))
    # chair kept (same id, moved), table added, lamp removed (max_misses=0)
    chair_nodes = [n for n in sg2.nodes_by_layer("object") if n.label == "chair"]
    assert len(chair_nodes) == 1
    assert chair_nodes[0].id == chair_id          # identity persisted
    assert chair_nodes[0].pos_m[0] > 1.0          # moved (x=12 voxel * 0.1 = 1.2m)
    assert "table" in obj_labels                  # new added
    assert "lamp" not in obj_labels               # removed after misses>max


def test_merge_stats_counts():
    pg = _pg(rooms=(0,))
    sg1 = build_scene_graph(pg, [_obj(10, 5, "chair"), _obj(20, 11, "lamp")])
    # fresh: chair moved (matched), lamp gone, new table
    fresh_objs = [_obj(11, 5, "chair"), _obj(40, 16, "table", pid=0)]
    sg2, stats = merge_observation(sg1, pg, fresh_objs, match_radius_m=0.5, max_misses=0)
    assert stats["matched"] == 1
    assert stats["added"] == 1
    assert stats["removed"] == 1
    assert stats["carried"] == 0
    assert stats["objects_total"] == 2


def test_merge_carry_over_survives_max_misses():
    pg = _pg(rooms=(0,))
    sg1 = build_scene_graph(pg, [_obj(10, 5, "chair"), _obj(20, 11, "lamp")])
    lamp_id = [n.id for n in sg1.nodes_by_layer("object") if n.label == "lamp"][0]
    # fresh: chair present, lamp absent — with max_misses=2 lamp should survive 2 rounds
    fresh_objs1 = [_obj(10, 5, "chair")]
    sg2, stats2 = merge_observation(sg1, pg, fresh_objs1, match_radius_m=0.5, max_misses=2)
    # lamp should still be in sg2 (misses=1 <= 2)
    lamp_nodes = [n for n in sg2.nodes_by_layer("object") if n.label == "lamp"]
    assert len(lamp_nodes) == 1
    assert lamp_nodes[0].id == lamp_id
    assert lamp_nodes[0].attrs["misses"] == 1
    assert stats2["carried"] == 1
    # second round: lamp still absent → misses becomes 2, still kept
    sg3, stats3 = merge_observation(sg2, pg, fresh_objs1, match_radius_m=0.5, max_misses=2)
    lamp3 = [n for n in sg3.nodes_by_layer("object") if n.label == "lamp"]
    assert len(lamp3) == 1
    assert lamp3[0].attrs["misses"] == 2
    # third round: misses would be 3 > max_misses=2 → dropped
    sg4, stats4 = merge_observation(sg3, pg, fresh_objs1, match_radius_m=0.5, max_misses=2)
    lamp4 = [n for n in sg4.nodes_by_layer("object") if n.label == "lamp"]
    assert len(lamp4) == 0
    assert stats4["removed"] == 1


def test_stable_ids_with_explicit_obj_ids():
    pg = _pg(rooms=(0,))
    objs = [_obj(10, 5, "chair"), _obj(20, 11, "lamp")]
    ids = ["my-chair-id", "my-lamp-id"]
    sg = build_scene_graph(pg, objs, obj_ids=ids)
    node_ids = {n.id for n in sg.nodes_by_layer("object")}
    assert "my-chair-id" in node_ids
    assert "my-lamp-id" in node_ids


def test_save_load_and_query_traverse_pointers(tmp_path):
    """After load, parent/children/ancestors use in-memory pointers, not JSON."""
    pg = _pg(rooms=(0, 1))
    objs = [_obj(10, 5, "chair", pid=0), _obj(20, 11, "lamp", pid=1)]
    sg = build_scene_graph(pg, objs)
    p = tmp_path / "sg2.json"
    sg.save(p)
    sg2 = SceneGraph.load(p)
    # Ancestor traversal through pointers (not JSON)
    for obj_node in sg2.nodes_by_layer("object"):
        ancs = sg2.ancestors(obj_node.id)
        layers = [a.layer for a in ancs]
        assert "building" in layers
