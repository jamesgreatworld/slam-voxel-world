"""gvd_rooms.py — partition a places graph into rooms via Louvain community
detection (SP-D2). Pure: graph (counts+edges) in, room labels out. No I/O."""
from __future__ import annotations


def partition_rooms(num_nodes: int, edges, resolution: float = 1.0, seed: int = 0):
    """Return a room id per node (list length num_nodes).

    edges: iterable of (a, b, length_m). Edge weight = 1/length (short edges
    bind a room; long bottleneck edges = doorways are weak). Communities are
    numbered by descending size (room 0 is the largest). Isolated nodes each
    get their own room.
    """
    import networkx as nx
    from networkx.algorithms.community import louvain_communities

    g = nx.Graph()
    g.add_nodes_from(range(num_nodes))
    for a, b, length_m in edges:
        g.add_edge(int(a), int(b), weight=1.0 / max(float(length_m), 1e-3))
    if g.number_of_edges() == 0:
        return list(range(num_nodes))
    comms = louvain_communities(g, weight="weight", resolution=resolution, seed=seed)
    room_of = [0] * num_nodes
    for rid, comm in enumerate(sorted(comms, key=lambda c: -len(c))):
        for n in comm:
            room_of[int(n)] = rid
    return room_of
