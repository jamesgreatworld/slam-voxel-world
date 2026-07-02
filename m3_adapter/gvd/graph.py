"""graph.py — places graph: PlacesGraph data class, skeleton_to_graph
construction, and cleanup transforms (prune_spurs, merge_close). Pure.
(GVD subsystem, see the architecture spec.)"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from scipy import ndimage
from m3_adapter.clustering import dbscan_labels


@dataclass
class PlaceNode:
    idx: tuple            # dense voxel index (i,j,k)
    clearance_m: float
    degree: int           # GRAPH degree (set/maintained by construction+transforms)
    type: str             # junction|passthrough|endpoint|isolated|loop
    room: int = -1        # -1 = unassigned


@dataclass
class PlacesGraph:
    nodes: list
    edges: list           # list of (a_id, b_id, length_m), a<b, deduped
    voxel_size: float
    vmin: np.ndarray

    def positions_m(self) -> np.ndarray:
        if not self.nodes:
            return np.zeros((0, 3))
        idx = np.array([n.idx for n in self.nodes], dtype=float)
        return (idx + np.asarray(self.vmin, dtype=float)) * self.voxel_size

    def adjacency(self) -> dict:
        adj = {i: {} for i in range(len(self.nodes))}
        for a, b, ln in self.edges:
            adj[a][b] = ln
            adj[b][a] = ln
        return adj


def _degree_type(num_nodes: int, edges):
    deg = [0] * num_nodes
    for a, b, _ in edges:
        deg[a] += 1
        deg[b] += 1
    types = []
    for d in deg:
        if d >= 3:
            types.append("junction")
        elif d == 2:
            types.append("passthrough")
        elif d == 1:
            types.append("endpoint")
        else:
            types.append("isolated")
    return deg, types


def skeleton_to_graph(gvd, dist_m, voxel_size, vmin, merge_radius_m=0.15) -> PlacesGraph:
    """Sparsify a 1-voxel-wide skeleton into a PlacesGraph.

    Args:
        gvd: bool array, the thinned skeleton.
        dist_m: float array, clearance (metres to nearest obstacle) per cell.
        voxel_size: metres per voxel.
        vmin: array-like shape (3,), world-space voxel offset (i,j,k origin).
        merge_radius_m: key voxels within this radius merge into one node.

    Returns:
        PlacesGraph with PlaceNode list and edge list (a_id, b_id, length_m).
    """
    if not gvd.any():
        return PlacesGraph([], [], voxel_size, np.asarray(vmin))

    kernel = np.ones((3, 3, 3), dtype=np.uint8)
    kernel[1, 1, 1] = 0
    nbr = ndimage.convolve(gvd.astype(np.uint8), kernel, mode="constant", cval=0)
    degree = nbr * gvd

    key = gvd & (degree != 2)
    key_coords = np.argwhere(key)
    if len(key_coords) == 0:
        any_idx = tuple(int(x) for x in np.argwhere(gvd)[0])
        node = PlaceNode(idx=any_idx, clearance_m=float(dist_m[any_idx]),
                         degree=0, type="loop")
        return PlacesGraph([node], [], voxel_size, np.asarray(vmin))

    eps_vox = max(merge_radius_m / voxel_size, np.sqrt(3))
    labels = dbscan_labels(key_coords, eps_vox, 1)
    n_clusters = int(labels.max()) + 1

    key_to_cluster = {}
    for coord, lab in zip(key_coords, labels):
        key_to_cluster[(int(coord[0]), int(coord[1]), int(coord[2]))] = int(lab)

    cluster_snaps = []
    cluster_clearances = []
    for c in range(n_clusters):
        members = key_coords[labels == c]
        centroid = members.mean(axis=0)
        snap = members[int(np.argmin(((members - centroid) ** 2).sum(axis=1)))]
        clearance = float(dist_m[members[:, 0], members[:, 1], members[:, 2]].max())
        cluster_snaps.append((int(snap[0]), int(snap[1]), int(snap[2])))
        cluster_clearances.append(clearance)

    edge_len = {}

    def _add_edge(a, b, length_m):
        if a == b:
            return
        e = (min(a, b), max(a, b))
        if e not in edge_len or length_m < edge_len[e]:
            edge_len[e] = length_m

    struct26 = ndimage.generate_binary_structure(3, 3)
    offsets = [(dx, dy, dz)
               for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               if not (dx == 0 and dy == 0 and dz == 0)]

    # Chain edges: label degree-2 chain voxels into components, then find which
    # key clusters each component touches via LOCAL 26-neighbour lookup.
    chain = gvd & ~key
    clab, ncomp = ndimage.label(chain, structure=struct26)
    if ncomp > 0:
        chain_coords = np.argwhere(chain)
        comp_ids = clab[chain_coords[:, 0], chain_coords[:, 1], chain_coords[:, 2]]
        comp_clusters: dict[int, set] = {}
        comp_size: dict[int, int] = {}
        for coord, cid in zip(chain_coords, comp_ids):
            cid = int(cid)
            x, y, z = int(coord[0]), int(coord[1]), int(coord[2])
            comp_size[cid] = comp_size.get(cid, 0) + 1
            cset = comp_clusters.setdefault(cid, set())
            for dx, dy, dz in offsets:
                lab = key_to_cluster.get((x + dx, y + dy, z + dz))
                if lab is not None:
                    cset.add(lab)
        for cid, clusters in comp_clusters.items():
            if len(clusters) == 2:
                a, b = sorted(clusters)
                _add_edge(a, b, comp_size[cid] * voxel_size)

    # Direct key-key adjacency across clusters.
    for coord, lab in zip(key_coords, labels):
        x, y, z = int(coord[0]), int(coord[1]), int(coord[2])
        for dx, dy, dz in offsets:
            nlab = key_to_cluster.get((x + dx, y + dy, z + dz))
            if nlab is not None and nlab != int(lab):
                _add_edge(int(lab), nlab, voxel_size)

    edges = [(a, b, edge_len[(a, b)]) for (a, b) in sorted(edge_len)]

    # Build PlaceNode list using GRAPH degree/type (not voxel degree).
    deg, types = _degree_type(n_clusters, edges)
    nodes = []
    for c in range(n_clusters):
        nodes.append(PlaceNode(
            idx=cluster_snaps[c],
            clearance_m=cluster_clearances[c],
            degree=deg[c],
            type=types[c],
        ))

    return PlacesGraph(nodes, edges, voxel_size, np.asarray(vmin))


def _rebuild(nodes, adj, voxel_size, vmin):
    """Rebuild a PlacesGraph from a node list + adjacency dict over a set of
    surviving old indices. `adj` is {old_id: {old_nbr: length}}. Renumbers."""
    alive = sorted(adj.keys())
    old_to_new = {old: i for i, old in enumerate(alive)}
    new_nodes = [nodes[old] for old in alive]
    seen = set()
    new_edges = []
    for a in alive:
        for b, ln in adj[a].items():
            if b not in old_to_new:
                continue
            key = (min(a, b), max(a, b))
            if key in seen:
                continue
            seen.add(key)
            na, nb = old_to_new[a], old_to_new[b]
            new_edges.append((min(na, nb), max(na, nb), ln))
    deg, types = _degree_type(len(new_nodes), new_edges)
    rebuilt = []
    for i, nd in enumerate(new_nodes):
        rebuilt.append(PlaceNode(idx=nd.idx, clearance_m=nd.clearance_m,
                                 degree=deg[i], type=types[i], room=nd.room))
    return PlacesGraph(rebuilt, new_edges, voxel_size, np.asarray(vmin))


def drop_small_components(g: PlacesGraph, min_nodes: int) -> PlacesGraph:
    """Remove whole connected components with fewer than min_nodes nodes —
    isolated carve specks / tiny disconnected free-space pockets. Keeps the
    real (large) structure. Returns a PlacesGraph."""
    if min_nodes <= 1 or not g.nodes:
        return g
    adj = g.adjacency()
    seen = set()
    keep = set()
    for s in range(len(g.nodes)):
        if s in seen:
            continue
        stack = [s]
        seen.add(s)
        comp = [s]
        while stack:
            u = stack.pop()
            for v in adj[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
                    comp.append(v)
        if len(comp) >= min_nodes:
            keep.update(comp)
    if not keep:
        return g
    sub = {n: {b: ln for b, ln in adj[n].items() if b in keep} for n in keep}
    return _rebuild(g.nodes, sub, g.voxel_size, g.vmin)


def prune_spurs(g: PlacesGraph, max_len_m: float) -> PlacesGraph:
    """Iteratively remove degree-1 nodes whose single edge is shorter than
    max_len_m (skeleton hairs from voxel noise). Long-edge endpoints (real
    corridor ends) are kept. Repeats until no short spur remains."""
    if max_len_m <= 0 or not g.nodes:
        return g
    adj = g.adjacency()
    changed = True
    while changed:
        changed = False
        for n in list(adj.keys()):
            if len(adj[n]) == 1:
                (nbr, ln), = adj[n].items()
                if ln < max_len_m:
                    del adj[nbr][n]
                    del adj[n]
                    changed = True
    if not adj:  # everything pruned (degenerate) — keep original
        return g
    return _rebuild(g.nodes, adj, g.voxel_size, g.vmin)


def merge_close(g: PlacesGraph, radius_m: float) -> PlacesGraph:
    """Contract nodes whose world positions are within radius_m into a single
    node (DBSCAN clusters), rewiring edges (self-loops dropped, parallel edges
    keep the shortest). Collapses residual tangles."""
    if radius_m <= 0 or len(g.nodes) < 2:
        return g
    pos = g.positions_m()
    labels = dbscan_labels(pos, radius_m, 1)
    n_clusters = int(labels.max()) + 1
    if n_clusters == len(g.nodes):
        return g  # nothing merged
    # representative node per cluster = the one with max clearance
    members = {c: np.where(labels == c)[0] for c in range(n_clusters)}
    new_nodes = []
    cluster_to_new = {}
    for c in range(n_clusters):
        ms = members[c]
        best = ms[int(np.argmax([g.nodes[i].clearance_m for i in ms]))]
        cluster_to_new[c] = len(new_nodes)
        nd = g.nodes[best]
        new_nodes.append(PlaceNode(idx=nd.idx, clearance_m=nd.clearance_m,
                                   degree=0, type=nd.type, room=nd.room))
    # rebuild edges between clusters
    edge_len = {}
    for a, b, ln in g.edges:
        ca, cb = cluster_to_new[labels[a]], cluster_to_new[labels[b]]
        if ca == cb:
            continue
        key = (min(ca, cb), max(ca, cb))
        if key not in edge_len or ln < edge_len[key]:
            edge_len[key] = ln
    new_edges = [(a, b, edge_len[(a, b)]) for (a, b) in sorted(edge_len)]
    deg, types = _degree_type(len(new_nodes), new_edges)
    for i in range(len(new_nodes)):
        new_nodes[i].degree = deg[i]
        new_nodes[i].type = types[i]
    return PlacesGraph(new_nodes, new_edges, g.voxel_size, np.asarray(g.vmin))
