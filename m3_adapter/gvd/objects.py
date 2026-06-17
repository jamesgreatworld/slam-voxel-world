"""objects.py — scene-graph Objects layer: cluster semantic voxels into object
instances (connected components per non-structure class) and link each to its
containing place node. (Hydra Objects layer.)"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy import ndimage

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
    shape_sig: tuple = ()  # principal-axis extents (std-devs) in metres, descending


def extract_objects(occ, sem, voxel_size, vmin, label_names=None,
                    structure_labels=DEFAULT_STRUCTURE, min_voxels=20,
                    connectivity=3):
    """Cluster occupied voxels of each non-structure semantic class into object
    instances via connected components. Returns list[ObjectNode]."""
    label_names = label_names or {}
    structure = set(int(s) for s in structure_labels)
    objects = []
    present = [int(L) for L in np.unique(sem) if L != 0 and int(L) not in structure]
    struct = ndimage.generate_binary_structure(3, connectivity)
    for L in present:
        mask = occ & (sem == L)
        lab, n = ndimage.label(mask, structure=struct)
        if n == 0:
            continue
        # component sizes + bounding boxes via ndimage
        objs = ndimage.find_objects(lab)
        for ci, sl in enumerate(objs, start=1):
            if sl is None:
                continue
            comp = (lab[sl] == ci)
            cnt = int(comp.sum())
            if cnt < min_voxels:
                continue
            # centroid in dense coords
            local = np.argwhere(comp)
            base = np.array([s.start for s in sl])
            cells = local + base
            centroid = cells.mean(axis=0).round().astype(int)
            bmin = cells.min(axis=0); bmax = cells.max(axis=0)
            # shape signature: principal-axis extents (std-devs) in metres, descending
            c = cells.astype(np.float64)
            c = c - c.mean(axis=0)
            if len(c) >= 3:
                cov = (c.T @ c) / len(c)
                ev = np.linalg.eigvalsh(cov)   # ascending, >=0
                ev = np.clip(ev, 0, None)
                sig = tuple(float(np.sqrt(e) * voxel_size) for e in ev[::-1])  # descending, metres
            else:
                sig = (0.0, 0.0, 0.0)
            objects.append(ObjectNode(
                idx=tuple(int(x) for x in centroid),
                label=L, label_name=label_names.get(L, str(L)),
                voxel_count=cnt,
                bbox_min=tuple(int(x) for x in bmin),
                bbox_max=tuple(int(x) for x in bmax),
                shape_sig=sig))
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
