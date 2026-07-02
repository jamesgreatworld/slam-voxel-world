"""objects.py — scene-graph Objects layer: cluster semantic voxels into object
instances (DBSCAN tolerance clustering per non-structure class, with bbox-proximity
post-merge to de-fragment same-class splits) and link each to its containing place
node. (Hydra Objects layer.)"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

# uhumans2 structure classes (not objects): floor=3, ceiling=4, wall=19
DEFAULT_STRUCTURE = frozenset({3, 4, 19})


@dataclass
class ObjectNode:
    idx: tuple            # centroid dense voxel index
    label: int            # super_id
    label_name: str
    voxel_count: int
    bbox_min: tuple       # dense voxel
    bbox_max: tuple       # dense voxel (inclusive)
    place_id: int = -1    # nearest place node id (-1 = unassigned)
    features: dict = field(default_factory=dict)


def _bbox_gap(b1min, b1max, b2min, b2max):
    """Max axis gap (in empty voxels) between two integer-voxel bboxes.
    Overlap or touching => 0; one empty voxel between them => 1; etc.
    Formula: number of empty integer positions between the inclusive bbox edges."""
    gap = 0
    for a in range(3):
        # positive => empty voxels between the two extents; <=0 => touching/overlap
        sep = max(b2min[a] - b1max[a] - 1, b1min[a] - b2max[a] - 1, 0)
        gap = max(gap, sep)
    return gap


def _union_find_merge(clusters, merge_gap_voxels):
    """Given a list of cell arrays (one per DBSCAN cluster), merge those whose
    bounding boxes are within merge_gap_voxels of each other via union-find.
    Returns a list of merged cell arrays."""
    n = len(clusters)
    if n == 0:
        return []

    # precompute bboxes
    bmins = [c.min(axis=0) for c in clusters]
    bmaxs = [c.max(axis=0) for c in clusters]

    # union-find (path-compressed)
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[ry] = rx

    # O(n^2) pairwise; n is number of DBSCAN clusters per class — small in practice
    for i in range(n):
        for j in range(i + 1, n):
            if _bbox_gap(bmins[i], bmaxs[i], bmins[j], bmaxs[j]) <= merge_gap_voxels:
                union(i, j)

    # collect groups
    from collections import defaultdict
    groups = defaultdict(list)
    for i in range(n):
        groups[find(i)].append(i)

    merged = []
    for idxs in groups.values():
        combined = np.concatenate([clusters[i] for i in idxs], axis=0)
        merged.append(combined)
    return merged


def extract_objects(occ, sem, voxel_size, vmin, label_names=None,
                    structure_labels=DEFAULT_STRUCTURE, min_voxels=20,
                    cluster_eps_voxels: float = 3.0, min_samples: int = 5,
                    merge_gap_voxels: int = 2):
    """Cluster occupied voxels of each non-structure semantic class into object
    instances via DBSCAN tolerance clustering (bridges small geometric gaps),
    followed by a bbox-proximity merge of same-class fragments.
    Returns list[ObjectNode].

    Coordinate contract: all centroid / bbox values are in dense grid coords
    (same as the old connected-components implementation). Callers that convert
    to world metres add vmin externally — that contract is unchanged.
    """
    from m3_adapter.clustering import dbscan_labels
    from m3_adapter.gvd.features import extract_features

    label_names = label_names or {}
    structure = set(int(s) for s in structure_labels)
    objects = []
    present = [int(L) for L in np.unique(sem) if L != 0 and int(L) not in structure]

    for L in present:
        mask = occ & (sem == L)
        # argwhere over the FULL grid → coords are already full-grid indices,
        # no base-offset needed (unlike the old find_objects approach).
        cells_all = np.argwhere(mask)
        if len(cells_all) < min_voxels:
            continue

        # DBSCAN in voxel units — eps bridges gaps ≤ cluster_eps_voxels; noise (-1) dropped
        lab = dbscan_labels(cells_all, cluster_eps_voxels, min_samples)

        clusters = []
        for g in np.unique(lab):
            if g < 0:
                continue   # noise
            cells = cells_all[lab == g]
            if len(cells) < min_voxels:
                continue
            clusters.append(cells)

        if not clusters:
            continue

        # bbox-proximity merge: union-find over clusters whose bboxes are within
        # merge_gap_voxels; then build one ObjectNode per merged group
        merged_clusters = _union_find_merge(clusters, merge_gap_voxels)

        ctx = {"occ": occ, "sem": sem}
        for cells in merged_clusters:
            if len(cells) < min_voxels:
                continue
            centroid = cells.mean(axis=0).round().astype(int)
            bmin = cells.min(axis=0)
            bmax = cells.max(axis=0)
            feats = extract_features(cells, voxel_size, ctx=ctx)
            objects.append(ObjectNode(
                idx=tuple(int(x) for x in centroid),
                label=L, label_name=label_names.get(L, str(L)),
                voxel_count=len(cells),
                bbox_min=tuple(int(x) for x in bmin),
                bbox_max=tuple(int(x) for x in bmax),
                features=feats))

    return objects


def link_to_places(objects, places_graph):
    """Assign each object the nearest place node id (by centroid world-metre
    distance to place positions). Mutates objects' place_id; returns objects."""
    if not objects or not places_graph.nodes:
        return objects
    ppos = places_graph.positions_m()                  # (P,3)
    vs = places_graph.voxel_size
    vmin = np.asarray(places_graph.vmin, dtype=float)
    for o in objects:
        opos = (np.asarray(o.idx, dtype=float) + vmin) * vs
        d2 = ((ppos - opos[None, :]) ** 2).sum(axis=1)
        o.place_id = int(np.argmin(d2))
    return objects
