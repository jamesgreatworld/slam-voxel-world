"""scene_graph.py — hierarchical Dynamic Scene Graph (DSG): Building > Rooms >
{Places, Objects}, with object support-parenting (e.g. cup-on-table). Supports
CRUD + query + JSON persistence + incremental merge_observation. (Hydra-style
scene graph.)"""
from __future__ import annotations
import json
from dataclasses import dataclass, field
from pathlib import Path
from uuid import uuid4

import numpy as np


@dataclass
class SceneNode:
    id: str
    layer: str                 # "building" | "room" | "place" | "object"
    label: str = ""            # human label (e.g. "table", "room_3")
    pos_m: tuple = (0.0, 0.0, 0.0)
    attrs: dict = field(default_factory=dict)   # bbox, voxel_count, class id, etc.
    parent: str | None = None
    children: list = field(default_factory=list)  # child ids


class SceneGraph:
    """Nodes keyed by id; one building root. CRUD + query + JSON."""

    def __init__(self):
        self.nodes: dict = {}
        self.root_id: str | None = None

    # ---- CRUD ----
    def add_node(self, node: SceneNode, parent_id: str | None = None):
        if node.id in self.nodes:
            raise ValueError(f"node id exists: {node.id}")
        self.nodes[node.id] = node
        if node.layer == "building" and self.root_id is None:
            self.root_id = node.id
        if parent_id is not None:
            self.set_parent(node.id, parent_id)
        return node

    def get(self, node_id): return self.nodes.get(node_id)

    def update_node(self, node_id, **attrs):
        n = self.nodes[node_id]
        for k, v in attrs.items():
            if k == "attrs":
                n.attrs.update(v)
            else:
                setattr(n, k, v)
        return n

    def set_parent(self, node_id, new_parent_id):
        n = self.nodes[node_id]
        if n.parent is not None and n.parent in self.nodes:
            old = self.nodes[n.parent]
            if node_id in old.children:
                old.children.remove(node_id)
        n.parent = new_parent_id
        if new_parent_id is not None:
            p = self.nodes[new_parent_id]
            if node_id not in p.children:
                p.children.append(node_id)

    def remove_node(self, node_id, reparent_children=True):
        n = self.nodes.pop(node_id)
        # detach from parent
        if n.parent is not None and n.parent in self.nodes:
            pc = self.nodes[n.parent].children
            if node_id in pc:
                pc.remove(node_id)
        # children: reparent to grandparent, or orphan/delete
        for c in list(n.children):
            if reparent_children and n.parent is not None:
                self.set_parent(c, n.parent)
            else:
                self.nodes[c].parent = None
        return n

    # ---- query ----
    def children(self, node_id): return [self.nodes[c] for c in self.nodes[node_id].children]
    def parent(self, node_id):
        p = self.nodes[node_id].parent
        return self.nodes[p] if p is not None else None
    def ancestors(self, node_id):
        out = []; p = self.nodes[node_id].parent
        while p is not None and p in self.nodes:
            out.append(self.nodes[p]); p = self.nodes[p].parent
        return out
    def descendants(self, node_id):
        out = []; stack = list(self.nodes[node_id].children)
        while stack:
            c = stack.pop(); out.append(self.nodes[c]); stack.extend(self.nodes[c].children)
        return out
    def nodes_by_layer(self, layer): return [n for n in self.nodes.values() if n.layer == layer]
    def objects_in_room(self, room_id):
        """All object-layer descendants of a room node."""
        return [n for n in self.descendants(room_id) if n.layer == "object"]

    # ---- JSON / persistence ----
    def to_dict(self):
        return {"root": self.root_id,
                "nodes": [{"id": n.id, "layer": n.layer, "label": n.label,
                           "pos_m": list(n.pos_m), "attrs": n.attrs,
                           "parent": n.parent, "children": list(n.children)}
                          for n in self.nodes.values()]}

    @classmethod
    def from_dict(cls, d):
        """Reconstruct in-memory SceneGraph with live parent/children pointers.
        After this call, queries (parent/children/ancestors/descendants) traverse
        the in-memory node dict — not the JSON."""
        g = cls(); g.root_id = d.get("root")
        for nd in d["nodes"]:
            g.nodes[nd["id"]] = SceneNode(
                id=nd["id"], layer=nd["layer"], label=nd.get("label", ""),
                pos_m=tuple(nd.get("pos_m", (0, 0, 0))), attrs=nd.get("attrs", {}),
                parent=nd.get("parent"), children=list(nd.get("children", [])))
        return g

    def save(self, path):
        """Serialise to a JSON file. Restores fully via SceneGraph.load()."""
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path):
        """Load from a JSON file; returns a SceneGraph with live pointer structure."""
        return cls.from_dict(json.loads(Path(path).read_text()))


def build_scene_graph(places_graph, objects, world_id="apartment",
                      support_gap_m=0.20, obj_ids=None):
    """Assemble Building > Rooms > {Places, Objects}. Object parent = supporting
    object if it rests on one (bbox geometry, y up), else the room it's in.

    Parameters
    ----------
    obj_ids : list[str] | None
        Stable ids to use for each object in ``objects`` (same length).  If
        None, a uuid-based id is generated per object.  Passing stable ids from
        a prior graph is how merge_observation preserves object identity.
    """
    sg = SceneGraph()
    vs = places_graph.voxel_size
    vmin = np.asarray(places_graph.vmin, dtype=float)

    # 1. Building root
    sg.add_node(SceneNode(id="building:0", layer="building", label=world_id))

    # 2. Rooms: distinct room ids among place nodes
    positions_m = places_graph.positions_m()  # shape (N, 3)
    room_ids = sorted(set(n.room for n in places_graph.nodes if n.room >= 0))
    for r in room_ids:
        # centroid of this room's place positions
        indices = [i for i, n in enumerate(places_graph.nodes) if n.room == r]
        if indices:
            centroid = positions_m[indices].mean(axis=0)
            pos = tuple(float(x) for x in centroid)
        else:
            pos = (0.0, 0.0, 0.0)
        sg.add_node(
            SceneNode(id=f"room:{r}", layer="room", label=f"room_{r}", pos_m=pos),
            parent_id="building:0",
        )

    # 3. Places
    for i, node in enumerate(places_graph.nodes):
        pos = tuple(float(x) for x in positions_m[i])
        if node.room >= 0 and f"room:{node.room}" in sg.nodes:
            parent_id = f"room:{node.room}"
        else:
            parent_id = "building:0"
        sg.add_node(
            SceneNode(
                id=f"place:{i}",
                layer="place",
                pos_m=pos,
                attrs={"degree": node.degree, "clearance_m": node.clearance_m},
            ),
            parent_id=parent_id,
        )

    # 4. Objects — two-pass: add all first (parent=None), then set parents
    # Precompute metres bboxes for all objects
    def _bbox_m(o):
        bmin = (np.asarray(o.bbox_min, dtype=float) + vmin) * vs
        bmax = (np.asarray(o.bbox_max, dtype=float) + vmin) * vs
        return bmin, bmax

    obj_nodes = []
    obj_bboxes = []  # list of (bmin_m, bmax_m) in world metres
    for j, o in enumerate(objects):
        opos_m = (np.asarray(o.idx, dtype=float) + vmin) * vs
        # Stable id: use caller-supplied list if provided, else generate uuid-based
        if obj_ids is not None:
            oid = obj_ids[j]
        else:
            oid = f"object:{uuid4().hex[:8]}"
        bmin_m, bmax_m = _bbox_m(o)
        node = SceneNode(
            id=oid,
            layer="object",
            label=o.label_name,
            pos_m=tuple(float(x) for x in opos_m),
            attrs={
                "class": o.label,
                "voxel_count": o.voxel_count,
                "bbox_min": list(o.bbox_min),
                "bbox_max": list(o.bbox_max),
                "bbox_min_m": list(bmin_m),
                "bbox_max_m": list(bmax_m),
                "place_id": o.place_id,
                "features": dict(getattr(o, "features", {}) or {}),
                "shape": list((getattr(o, "features", {}) or {}).get("shape", ())),
                "misses": 0,
                "seen_count": 1,
            },
        )
        sg.add_node(node)  # no parent yet
        obj_nodes.append(node)
        obj_bboxes.append(_bbox_m(o))

    # Now compute support / room parenting
    for j, (o, node) in enumerate(zip(objects, obj_nodes)):
        bmin_j, bmax_j = obj_bboxes[j]
        o_bottom_y = bmin_j[1]  # y is index 1

        best_support = None
        best_top_y = -1e18

        for k, (bmin_k, bmax_k) in enumerate(obj_bboxes):
            if k == j:
                continue
            k_top_y = bmax_k[1]
            # support test: o's bottom within [k_top, k_top + gap]
            if not (k_top_y <= o_bottom_y <= k_top_y + support_gap_m):
                continue
            # horizontal (x,z) overlap: x is index 0, z is index 2
            if bmax_j[0] < bmin_k[0] or bmin_j[0] > bmax_k[0]:
                continue
            if bmax_j[2] < bmin_k[2] or bmin_j[2] > bmax_k[2]:
                continue
            # candidate — pick highest top (closest support)
            if k_top_y > best_top_y:
                best_top_y = k_top_y
                best_support = obj_nodes[k].id

        if best_support is not None:
            sg.set_parent(node.id, best_support)
        else:
            # fall through to room via place
            place_id = o.place_id
            room_parent = "building:0"
            if place_id >= 0 and place_id < len(places_graph.nodes):
                room = places_graph.nodes[place_id].room
                room_key = f"room:{room}"
                if room >= 0 and room_key in sg.nodes:
                    room_parent = room_key
            sg.set_parent(node.id, room_parent)

    return sg


def merge_observation(existing: SceneGraph, fresh_places_graph, fresh_objects,
                      world_id="apartment", support_gap_m=0.20,
                      match_radius_m=0.5, max_misses=3, shape_weight: float = 2.0):
    """Incrementally update ``existing`` from a freshly-derived places graph +
    objects. Object identity persists via class+proximity data association;
    unmatched existing objects accrue a 'misses' count and are removed past
    max_misses; new objects are added with fresh stable ids; the room/place
    scaffold is rebuilt from the fresh graph; all surviving objects are
    re-parented (room/support). Returns ``(new_scene_graph, stats)``.

    Algorithm
    ---------
    1. Pull existing object nodes (layer=="object") from `existing`.
    2. Data association: for each fresh object find the nearest existing object
       of the same class within match_radius_m (greedy, one-to-one).
    3. Build obj_ids aligned to fresh_objects (matched=reuse id, new=uuid).
    4. Build a fresh scene graph with those ids.
    5. Inject surviving carry-overs (unseen existing objects whose misses <=
       max_misses) by attaching them to their last-known room if it still exists
       in the new graph, otherwise to the building root.
    6. Return (new_sg, stats).
    """
    vs = fresh_places_graph.voxel_size
    vmin = np.asarray(fresh_places_graph.vmin, dtype=float)

    # -- Step 1: gather existing object nodes ---------------------------------
    existing_objs = existing.nodes_by_layer("object")
    # Build a dict: id -> {id, class, pos_m (np array), attrs, misses}
    ex_by_id = {}
    for n in existing_objs:
        pos = np.asarray(n.pos_m, dtype=float)
        ex_by_id[n.id] = {
            "id": n.id,
            "class": n.attrs.get("class"),
            "pos_m": pos,
            "attrs": dict(n.attrs),
            "label": n.label,
            "misses": int(n.attrs.get("misses", 0)),
        }

    # -- Step 2: data association (class-internal Hungarian + shape) ----------
    from scipy.optimize import linear_sum_assignment

    # Group existing objects by class
    ex_by_class: dict = {}
    for info in ex_by_id.values():
        cls = info["class"]
        ex_by_class.setdefault(cls, []).append(info)

    # Group fresh objects by class, tracking their original indices
    fresh_by_class: dict = {}
    for j, o in enumerate(fresh_objects):
        fresh_by_class.setdefault(o.label, []).append((j, o))

    matched: dict[int, str] = {}     # fresh index -> existing id

    # Collect all classes present in either side
    all_classes = set(ex_by_class.keys()) | set(fresh_by_class.keys())

    from m3_adapter.gvd.features import feature_cost

    for cls in all_classes:
        existing_C = ex_by_class.get(cls, [])
        fresh_C = fresh_by_class.get(cls, [])

        if not existing_C or not fresh_C:
            # Nothing to match — unmatched fresh become new ids later
            continue

        # Build cost matrix: rows = existing_C, cols = fresh_C
        n_ex = len(existing_C)
        n_fr = len(fresh_C)
        cost = np.empty((n_ex, n_fr), dtype=float)
        for i, ex_info in enumerate(existing_C):
            for jj, (_, fo) in enumerate(fresh_C):
                fo_pos = (np.asarray(fo.idx, dtype=float) + vmin) * vs
                dist = float(np.linalg.norm(ex_info["pos_m"] - fo_pos))
                fresh_feats = getattr(fo, "features", {}) or {}
                ex_feats = ex_info["attrs"].get("features", {})
                cost[i, jj] = dist + feature_cost(ex_feats, fresh_feats)

        ri, cj = linear_sum_assignment(cost)

        for k in range(len(ri)):
            ex_info = existing_C[ri[k]]
            fresh_idx, fresh_obj = fresh_C[cj[k]]
            fresh_pos = (np.asarray(fresh_obj.idx, dtype=float) + vmin) * vs
            dist_m = float(np.linalg.norm(ex_info["pos_m"] - fresh_pos))
            if dist_m <= match_radius_m:
                matched[fresh_idx] = ex_info["id"]

    # -- Step 3: build obj_ids list aligned to fresh_objects ------------------
    obj_ids = []
    for j in range(len(fresh_objects)):
        if j in matched:
            obj_ids.append(matched[j])
        else:
            obj_ids.append(f"object:{uuid4().hex[:8]}")

    # -- Step 4: build the fresh scene graph ----------------------------------
    new_sg = build_scene_graph(fresh_places_graph, fresh_objects, world_id,
                               support_gap_m, obj_ids=obj_ids)
    # Ensure misses=0 on all freshly-placed objects; update seen_count
    for j, nid in enumerate(obj_ids):
        if nid not in new_sg.nodes:
            continue
        node = new_sg.nodes[nid]
        node.attrs["misses"] = 0
        if j in matched:
            # matched: carry forward existing seen_count + 1
            ex_seen = ex_by_id[nid]["attrs"].get("seen_count", 1)
            node.attrs["seen_count"] = ex_seen + 1
        # else: newly added — seen_count=1 already set by build_scene_graph

    # -- Step 5: carry-overs (existing objects NOT seen in fresh) -------------
    seen_existing = set(matched.values())
    removed = 0
    carried = 0

    for eid, info in ex_by_id.items():
        if eid in seen_existing:
            continue  # already in new graph as a matched fresh object
        new_misses = info["misses"] + 1
        if new_misses > max_misses:
            removed += 1
            continue  # drop it
        # Carry over: re-inject into new graph
        carried += 1
        carry_attrs = dict(info["attrs"])
        carry_attrs["misses"] = new_misses
        # Determine parent: use last-known room if it still exists, else building
        # We infer the last-known room from the existing graph's parent chain
        ex_node = existing.get(eid)
        carry_parent = "building:0"
        if ex_node is not None:
            # Walk ancestors of the existing node to find its room
            for anc in existing.ancestors(eid):
                if anc.layer == "room" and anc.id in new_sg.nodes:
                    carry_parent = anc.id
                    break
        carry_node = SceneNode(
            id=eid,
            layer="object",
            label=info["label"],
            pos_m=tuple(float(x) for x in info["pos_m"]),
            attrs=carry_attrs,
        )
        new_sg.add_node(carry_node, parent_id=carry_parent)

    # -- Step 6: stats --------------------------------------------------------
    n_matched = len(matched)
    n_added = len(obj_ids) - n_matched
    n_objects_total = len(new_sg.nodes_by_layer("object"))
    stats = {
        "matched": n_matched,
        "added": n_added,
        "removed": removed,
        "carried": carried,
        "objects_total": n_objects_total,
    }
    return new_sg, stats


# ---------------------------------------------------------------------------
# Temporal fragment consolidation
# ---------------------------------------------------------------------------

def _bbox_gap_m(amin, amax, bmin, bmax):
    """Axis-aligned bbox gap in metres between two boxes (all numpy arrays).
    Returns the max per-axis gap; negative means overlap on that axis.
    We use max-of-axis-gaps: if ANY axis has positive separation the boxes
    do not touch on that axis."""
    gaps = np.maximum(np.asarray(amin) - np.asarray(bmax),
                      np.asarray(bmin) - np.asarray(amax))
    return float(np.max(gaps))


def consolidate_fragments(sg: SceneGraph, gap_m: float = 0.15, min_persist: int = 2):
    """Merge same-class object nodes whose axis-aligned bboxes (world metres) are
    within gap_m AND both have seen_count >= min_persist — persistent co-located
    fragments of one real object. The surviving node keeps the larger
    voxel_count's id; bbox is unioned, voxel_count summed, seen_count = max.
    Children of the absorbed node are reparented to the survivor. Mutates sg.
    Returns the number of merges performed."""
    candidates = [
        n for n in sg.nodes_by_layer("object")
        if n.attrs.get("seen_count", 0) >= min_persist
        and "bbox_min_m" in n.attrs and "bbox_max_m" in n.attrs
    ]

    # Union-Find helpers (operating on node ids)
    parent_uf: dict[str, str] = {n.id: n.id for n in candidates}

    def _find(x):
        while parent_uf[x] != x:
            parent_uf[x] = parent_uf[parent_uf[x]]
            x = parent_uf[x]
        return x

    def _union(a, b):
        parent_uf[_find(a)] = _find(b)

    # Pair-wise check (O(n²) — candidate list is small in practice)
    n = len(candidates)
    merges_found = 0
    for i in range(n):
        for j in range(i + 1, n):
            ni, nj = candidates[i], candidates[j]
            if ni.attrs.get("class") != nj.attrs.get("class"):
                continue
            gap = _bbox_gap_m(
                ni.attrs["bbox_min_m"], ni.attrs["bbox_max_m"],
                nj.attrs["bbox_min_m"], nj.attrs["bbox_max_m"],
            )
            if gap <= gap_m:
                _union(ni.id, nj.id)

    # Group by root
    groups: dict[str, list] = {}
    for n in candidates:
        root = _find(n.id)
        groups.setdefault(root, []).append(n)

    merges_done = 0
    for group in groups.values():
        if len(group) < 2:
            continue
        # Survivor = node with largest voxel_count
        survivor = max(group, key=lambda n: n.attrs.get("voxel_count", 0))
        absorbed_list = [n for n in group if n.id != survivor.id]

        for absorbed in absorbed_list:
            # Union bbox_*_m
            s_bmin = np.minimum(survivor.attrs["bbox_min_m"], absorbed.attrs["bbox_min_m"])
            s_bmax = np.maximum(survivor.attrs["bbox_max_m"], absorbed.attrs["bbox_max_m"])
            # Union bbox voxel coords too
            s_vmin = [min(a, b) for a, b in zip(survivor.attrs["bbox_min"], absorbed.attrs["bbox_min"])]
            s_vmax = [max(a, b) for a, b in zip(survivor.attrs["bbox_max"], absorbed.attrs["bbox_max"])]
            # Weighted average pos_m by voxel_count
            vc_s = survivor.attrs.get("voxel_count", 1)
            vc_a = absorbed.attrs.get("voxel_count", 1)
            total_vc = vc_s + vc_a
            s_pos = tuple(
                float((vc_s * ps + vc_a * pa) / total_vc)
                for ps, pa in zip(survivor.pos_m, absorbed.pos_m)
            )
            # Apply to survivor
            survivor.attrs["bbox_min_m"] = list(s_bmin)
            survivor.attrs["bbox_max_m"] = list(s_bmax)
            survivor.attrs["bbox_min"] = s_vmin
            survivor.attrs["bbox_max"] = s_vmax
            survivor.attrs["voxel_count"] = total_vc
            survivor.attrs["seen_count"] = max(
                survivor.attrs.get("seen_count", 0),
                absorbed.attrs.get("seen_count", 0),
            )
            survivor.pos_m = s_pos

            # Reparent absorbed's children to survivor
            for child_id in list(absorbed.children):
                sg.set_parent(child_id, survivor.id)

            sg.remove_node(absorbed.id, reparent_children=False)
            merges_done += 1

    return merges_done
