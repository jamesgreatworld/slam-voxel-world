"""scene_graph.py — hierarchical Dynamic Scene Graph (DSG): Building > Rooms >
{Places, Objects}, with object support-parenting (e.g. cup-on-table). Supports
CRUD + query. (Hydra-style scene graph.)"""
from __future__ import annotations
from dataclasses import dataclass, field
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

    # ---- JSON ----
    def to_dict(self):
        return {"root": self.root_id,
                "nodes": [{"id": n.id, "layer": n.layer, "label": n.label,
                           "pos_m": list(n.pos_m), "attrs": n.attrs,
                           "parent": n.parent, "children": list(n.children)}
                          for n in self.nodes.values()]}

    @classmethod
    def from_dict(cls, d):
        g = cls(); g.root_id = d.get("root")
        for nd in d["nodes"]:
            g.nodes[nd["id"]] = SceneNode(
                id=nd["id"], layer=nd["layer"], label=nd.get("label", ""),
                pos_m=tuple(nd.get("pos_m", (0, 0, 0))), attrs=nd.get("attrs", {}),
                parent=nd.get("parent"), children=list(nd.get("children", [])))
        return g


def build_scene_graph(places_graph, objects, world_id="apartment",
                      support_gap_m=0.20):
    """Assemble Building > Rooms > {Places, Objects}. Object parent = supporting
    object if it rests on one (bbox geometry, y up), else the room it's in."""
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
        node = SceneNode(
            id=f"object:{j}",
            layer="object",
            label=o.label_name,
            pos_m=tuple(float(x) for x in opos_m),
            attrs={
                "class": o.label,
                "voxel_count": o.voxel_count,
                "bbox_min": list(o.bbox_min),
                "bbox_max": list(o.bbox_max),
                "place_id": o.place_id,
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
                best_support = f"object:{k}"

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
