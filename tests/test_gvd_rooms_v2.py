import numpy as np
from m3_adapter.gvd.graph import PlaceNode, PlacesGraph
from m3_adapter.gvd.rooms import partition_rooms


def _graph(n, edges):
    nodes = [PlaceNode(idx=(i, 0, 0), clearance_m=1.0, degree=0, type="x") for i in range(n)]
    return PlacesGraph(nodes, edges, 1.0, np.zeros(3))


def test_two_cliques_split_at_weak_bridge():
    edges = [(0,1,1.0),(0,2,1.0),(1,2,1.0),
             (3,4,1.0),(3,5,1.0),(4,5,1.0),
             (2,3,8.0)]
    rooms = partition_rooms(_graph(6, edges), resolution=1.0)
    assert rooms[0] == rooms[1] == rooms[2]
    assert rooms[3] == rooms[4] == rooms[5]
    assert rooms[0] != rooms[3]
    assert len(set(rooms)) == 2


def test_isolated_nodes_each_own_room():
    rooms = partition_rooms(_graph(4, [(0,1,1.0)]))
    assert rooms[0] == rooms[1]
    assert len(set(rooms)) == 3


def test_no_edges_all_singletons():
    assert len(set(partition_rooms(_graph(3, [])))) == 3
