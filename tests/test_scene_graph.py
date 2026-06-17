import numpy as np
from m3_adapter.gvd.graph import PlaceNode, PlacesGraph
from m3_adapter.gvd.objects import ObjectNode
from m3_adapter.gvd.scene_graph import SceneGraph, SceneNode, build_scene_graph


def test_crud_and_query():
    g = SceneGraph()
    g.add_node(SceneNode(id="building:0", layer="building"))
    g.add_node(SceneNode(id="room:0", layer="room"), parent_id="building:0")
    g.add_node(SceneNode(id="object:0", layer="object", label="table"), parent_id="room:0")
    assert g.parent("object:0").id == "room:0"
    assert [c.id for c in g.children("room:0")] == ["object:0"]
    assert [a.id for a in g.ancestors("object:0")] == ["room:0", "building:0"]
    # move object to a new room
    g.add_node(SceneNode(id="room:1", layer="room"), parent_id="building:0")
    g.set_parent("object:0", "room:1")
    assert g.parent("object:0").id == "room:1"
    assert g.children("room:0") == []
    # remove with reparent
    g.remove_node("room:1")           # object:0 reparents to building:0
    assert g.parent("object:0").id == "building:0"
    # json roundtrip
    g2 = SceneGraph.from_dict(g.to_dict())
    assert g2.parent("object:0").id == "building:0"


def test_build_hierarchy_with_support():
    # two places in room 0 and room 1
    nodes = [PlaceNode(idx=(10, 2, 10), clearance_m=1, degree=1, type="x", room=0),
             PlaceNode(idx=(50, 2, 50), clearance_m=1, degree=1, type="x", room=1)]
    pg = PlacesGraph(nodes, [], 0.1, np.zeros(3, np.int64))
    # table on the floor in room 0 (bbox y 0..10), cup resting on table top (y ~10..12),
    # chair in room 1
    table = ObjectNode(idx=(10,5,10), label=16, label_name="table", voxel_count=500,
                       bbox_min=(8,0,8), bbox_max=(12,10,12), place_id=0)
    cup = ObjectNode(idx=(10,12,10), label=99, label_name="cup", voxel_count=20,
                     bbox_min=(9,11,9), bbox_max=(11,13,11), place_id=0)
    chair = ObjectNode(idx=(50,3,50), label=5, label_name="chair", voxel_count=200,
                       bbox_min=(48,0,48), bbox_max=(52,6,52), place_id=1)
    sg = build_scene_graph(pg, [table, cup, chair], support_gap_m=0.30)
    # find node ids
    def by_label(lbl): return [n for n in sg.nodes_by_layer("object") if n.label == lbl][0]
    t, c, ch = by_label("table"), by_label("cup"), by_label("chair")
    # cup rests on table -> parent is the table object
    assert sg.parent(c.id).id == t.id
    # table is in room 0
    assert sg.parent(t.id).layer == "room"
    assert sg.parent(t.id).label.endswith("0") or sg.parent(t.id).id == "room:0"
    # chair is in room 1
    assert sg.parent(ch.id).id == "room:1"
    # query: objects in room 0 includes table AND cup (cup is descendant via table)
    room0 = sg.get("room:0")
    objs0 = sg.objects_in_room("room:0")
    assert t.id in [o.id for o in objs0] and c.id in [o.id for o in objs0]
