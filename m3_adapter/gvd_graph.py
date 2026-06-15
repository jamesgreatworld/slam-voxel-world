"""gvd_graph.py — sparsify a thinned GVD skeleton into a places graph (SP-D).

Pure numpy/scipy/sklearn: arrays in, graph (lists) out. No file I/O. See
docs/superpowers/specs/2026-06-15-spd-places-graph-design.md.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage
from sklearn.cluster import DBSCAN


def _node_type(degrees: np.ndarray) -> str:
    if (degrees >= 3).any():
        return "junction"
    if (degrees == 1).any():
        return "endpoint"
    if (degrees == 0).any():
        return "isolated"
    return "chain"


def skeleton_to_graph(gvd, dist_m, voxel_size, merge_radius_m=0.15):
    """Sparsify a 1-voxel-wide skeleton into a places graph.

    Args:
        gvd: bool array, the thinned skeleton.
        dist_m: float array, clearance (metres to nearest obstacle) per cell.
        voxel_size: metres per voxel.
        merge_radius_m: key voxels within this radius merge into one node.

    Returns:
        (nodes, edges):
          nodes: list of {"idx":(i,j,k), "clearance_m":float, "degree":int,
                          "type":"junction"|"endpoint"|"isolated"|"loop"}
          edges: list of (a_id, b_id, length_m), a_id<b_id, deduped.
    """
    if not gvd.any():
        return [], []
    kernel = np.ones((3, 3, 3), dtype=np.uint8)
    kernel[1, 1, 1] = 0
    nbr = ndimage.convolve(gvd.astype(np.uint8), kernel, mode="constant", cval=0)
    degree = nbr * gvd

    key = gvd & (degree != 2)
    key_coords = np.argwhere(key)
    if len(key_coords) == 0:
        any_idx = tuple(int(x) for x in np.argwhere(gvd)[0])
        return ([{"idx": any_idx, "clearance_m": float(dist_m[any_idx]),
                  "degree": 2, "type": "loop"}], [])

    eps_vox = max(merge_radius_m / voxel_size, np.sqrt(3))
    labels = DBSCAN(eps=eps_vox, min_samples=1).fit(key_coords).labels_
    n_clusters = int(labels.max()) + 1

    key_to_cluster = {}
    for coord, lab in zip(key_coords, labels):
        key_to_cluster[(int(coord[0]), int(coord[1]), int(coord[2]))] = int(lab)

    nodes = []
    for c in range(n_clusters):
        members = key_coords[labels == c]
        mdeg = degree[members[:, 0], members[:, 1], members[:, 2]]
        centroid = members.mean(axis=0)
        snap = members[int(np.argmin(((members - centroid) ** 2).sum(axis=1)))]
        clearance = float(dist_m[members[:, 0], members[:, 1], members[:, 2]].max())
        nodes.append({
            "idx": (int(snap[0]), int(snap[1]), int(snap[2])),
            "clearance_m": clearance,
            "degree": int(mdeg.max()),
            "type": _node_type(mdeg),
        })

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

    # 4a. chain edges. Label the degree-2 chain voxels into components, then for
    # each component find which key clusters its 26-neighbourhood touches via a
    # LOCAL lookup (bounded by chain length). The earlier full-array
    # binary_dilation-per-component was O(ncomp * grid) and hung on real data.
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

    # 4b. direct key-key adjacency across clusters (get() returns None out of
    # bounds / off-skeleton, so no explicit bounds check is needed).
    for coord, lab in zip(key_coords, labels):
        x, y, z = int(coord[0]), int(coord[1]), int(coord[2])
        for dx, dy, dz in offsets:
            nlab = key_to_cluster.get((x + dx, y + dy, z + dz))
            if nlab is not None and nlab != int(lab):
                _add_edge(int(lab), nlab, voxel_size)

    edges = [(a, b, edge_len[(a, b)]) for (a, b) in sorted(edge_len)]
    return nodes, edges
