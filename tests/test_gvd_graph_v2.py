import numpy as np
from m3_adapter.gvd.graph import skeleton_to_graph, prune_spurs, merge_close, PlacesGraph


def _vmin():
    return np.zeros(3, dtype=np.int64)


def test_line_two_endpoints_one_edge():
    g = np.zeros((21, 21, 21), dtype=bool)
    g[5:16, 10, 10] = True
    G = skeleton_to_graph(g, np.ones_like(g, float), 1.0, _vmin())
    assert len(G.nodes) == 2
    assert all(n.type == "endpoint" for n in G.nodes)
    assert len(G.edges) == 1


def test_cross_junction_and_endpoints():
    g = np.zeros((21, 21, 21), dtype=bool)
    g[5:16, 10, 10] = True
    g[10, 5:16, 10] = True
    G = skeleton_to_graph(g, np.ones_like(g, float), 1.0, _vmin())
    assert len([n for n in G.nodes if n.type == "junction"]) == 1
    assert len([n for n in G.nodes if n.type == "endpoint"]) == 4
    assert len(G.edges) == 4


def test_prune_spurs_removes_short_hairs():
    # long main corridor with a short hair off the middle. The spur must survive
    # DBSCAN node-merge (long enough) yet be short enough to prune.
    g = np.zeros((35, 35, 35), dtype=bool)
    g[5:25, 15, 15] = True       # main corridor (arms ~9 and ~10)
    g[14, 15:22, 15] = True      # short spur (~5) branching at (14,15,15)
    G = skeleton_to_graph(g, np.ones_like(g, float), 1.0, _vmin())
    n_before = len(G.nodes)
    P = prune_spurs(G, max_len_m=7.0)   # spur (~5) < 7 pruned; main arms (~9,10) kept
    assert len(P.nodes) < n_before
    # the two far endpoints of the main corridor survive
    assert len([n for n in P.nodes if n.type == "endpoint"]) == 2


def test_merge_close_contracts_near_nodes():
    # two endpoints 1 voxel apart should merge at radius 2*voxel
    g = np.zeros((21, 21, 21), dtype=bool)
    g[5:16, 10, 10] = True
    G = skeleton_to_graph(g, np.ones_like(g, float), 1.0, _vmin())
    M = merge_close(G, radius_m=100.0)   # huge radius merges both endpoints
    assert len(M.nodes) < len(G.nodes)
