"""rooms.py — partition a PlacesGraph into rooms via Louvain community
detection. Pure: graph in, room labels out. (GVD subsystem.)"""
from __future__ import annotations


def partition_rooms(graph, resolution: float = 1.0, seed: int = 0):
    """Return a room id per node (list length len(graph.nodes)).

    Edge weight = 1/length (short edges bind a room; long bottleneck edges =
    doorways are weak). Communities numbered by descending size (room 0 is the
    largest). Isolated nodes (no edges) each get their own room.
    """
    import networkx as nx

    n = len(graph.nodes)
    g = nx.Graph()
    g.add_nodes_from(range(n))
    for a, b, length_m in graph.edges:
        g.add_edge(int(a), int(b), weight=1.0 / max(float(length_m), 1e-3))
    if g.number_of_edges() == 0:
        return list(range(n))
    try:
        from networkx.algorithms.community import louvain_communities
        comms = louvain_communities(g, weight="weight", resolution=resolution, seed=seed)
    except ImportError:
        # networkx < 3 lacks louvain_communities; greedy modularity maximisation
        # (available since 2.x) optimises the same objective, deterministically.
        from networkx.algorithms.community import greedy_modularity_communities
        comms = greedy_modularity_communities(g, weight="weight", resolution=resolution)
    room_of = [0] * n
    for rid, comm in enumerate(sorted(comms, key=lambda c: -len(c))):
        for node in comm:
            room_of[int(node)] = rid
    return room_of
