from m3_adapter.gvd_rooms import partition_rooms


def test_two_cliques_split_at_weak_bridge():
    edges = [(0,1,1.0),(0,2,1.0),(1,2,1.0),
             (3,4,1.0),(3,5,1.0),(4,5,1.0),
             (2,3,8.0)]
    rooms = partition_rooms(6, edges, resolution=1.0)
    assert rooms[0] == rooms[1] == rooms[2]
    assert rooms[3] == rooms[4] == rooms[5]
    assert rooms[0] != rooms[3]
    assert len(set(rooms)) == 2


def test_isolated_nodes_each_own_room():
    rooms = partition_rooms(4, [(0,1,1.0)])
    assert rooms[0] == rooms[1]
    assert len(set(rooms)) == 3


def test_no_edges_all_singletons():
    assert len(set(partition_rooms(3, []))) == 3
