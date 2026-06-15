import numpy as np
from m3_adapter.gvd_graph import skeleton_to_graph


def _line(n=21, axis=0, fixed=10, lo=5, hi=15):
    g = np.zeros((n, n, n), dtype=bool)
    sl = [fixed, fixed, fixed]
    sl[axis] = slice(lo, hi + 1)
    g[tuple(sl)] = True
    return g


def test_line_two_endpoints_one_edge():
    g = _line()
    dist = np.ones_like(g, dtype=float)
    nodes, edges = skeleton_to_graph(g, dist, voxel_size=1.0, merge_radius_m=0.15)
    assert len(nodes) == 2
    assert all(n["type"] == "endpoint" for n in nodes)
    assert len(edges) == 1


def test_cross_one_junction_four_endpoints_four_edges():
    n = 21
    g = np.zeros((n, n, n), dtype=bool)
    g[5:16, 10, 10] = True
    g[10, 5:16, 10] = True
    dist = np.ones_like(g, dtype=float)
    nodes, edges = skeleton_to_graph(g, dist, voxel_size=1.0, merge_radius_m=0.15)
    junctions = [x for x in nodes if x["type"] == "junction"]
    endpoints = [x for x in nodes if x["type"] == "endpoint"]
    assert len(junctions) == 1
    assert len(endpoints) == 4
    assert len(nodes) == 5
    assert len(edges) == 4


def test_hairball_merges_to_single_junction():
    n = 25
    g = np.zeros((n, n, n), dtype=bool)
    g[11:14, 11:14, 11:14] = True
    g[14:20, 12, 12] = True
    g[5:11, 12, 12] = True
    g[12, 14:20, 12] = True
    g[12, 5:11, 12] = True
    dist = np.ones_like(g, dtype=float)
    nodes, edges = skeleton_to_graph(g, dist, voxel_size=1.0, merge_radius_m=0.20)
    assert len([x for x in nodes if x["type"] == "junction"]) == 1
    assert len([x for x in nodes if x["type"] == "endpoint"]) == 4
    assert len(edges) == 4
