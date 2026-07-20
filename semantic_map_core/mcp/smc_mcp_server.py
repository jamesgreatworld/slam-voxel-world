#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""smc_mcp_server.py — 自研 Hydra 场景图的 MCP 服务(给 agent 调用)。

标准 MCP over stdio(JSON-RPC 2.0),零第三方依赖:实现 initialize / tools/list /
tools/call。数据源 = smc_live_node 每周期落盘的存档目录:
  dsg.txt         三层场景图(building/room/place/object + 父子 + bbox + seen/misses)
  robot_pose.txt  机器人实时位姿(ROS odom 系, 节点每 10 帧写)
每次调用都重新读盘 -> 查询结果反映最新地图。

坐标: DSG 内部是 vxw(y-up), 本服务**对外统一 ROS(z-up)**:(x,y,z)_ros = (-Z,-X,Y)_vxw。

用法(MCP client 直接 spawn):
  python3 smc_mcp_server.py [--map-dir DIR] [--labelspace YAML]
"""
import argparse
import json
import math
import os
import sys

DEF_MAP = "/home/james/semantic_map_core/maps/house_live"
DEF_LS = ("/home/james/Semantic_map_ws/src/Hydra/config/label_spaces/"
          "tartanground_house_label_space.yaml")


def yup2ros(p):
    """vxw(y-up) -> ROS(z-up): (x,y,z)_ros = (-Z, -X, Y)_vxw"""
    return (-p[2], -p[0], p[1])


def load_labelnames(path):
    names = {}
    try:
        with open(path) as f:
            for line in f:
                if "label:" in line and "name:" in line:
                    lid = int(line.split("label:")[1].split(",")[0].strip())
                    nm = line.split("name:")[1].strip().rstrip("}").strip()
                    names[lid] = nm
    except Exception:
        pass
    return names


class Scene:
    """一次快照:读 dsg.txt + robot_pose.txt,转成 ROS 坐标的节点表。"""

    def __init__(self, map_dir, labelspace):
        self.nodes = {}       # id -> dict
        self.order = []
        self.names = load_labelnames(labelspace)
        self.robot = None
        self.map_dir = map_dir
        self.meta = {}
        self._load_dsg(os.path.join(map_dir, "dsg.json"))
        self._load_pose(os.path.join(map_dir, "robot_pose.json"))

    def _load_dsg(self, path):
        """读节点每周期原子写的 dsg.json(vxw y-up)-> 转 ROS z-up。"""
        try:
            with open(path) as f:
                d = json.load(f)
        except Exception:
            return
        self.meta = {k: d.get(k) for k in ("stamp_sec", "cycle", "id_counter")}
        for nd in d.get("nodes", []):
            nid, layer = nd["id"], nd["layer"]
            rp = yup2ros(nd.get("pos", [0, 0, 0]))
            lo = hi = None
            if layer == "object" and "bbox_min_m" in nd:
                a, b = yup2ros(nd["bbox_min_m"]), yup2ros(nd["bbox_max_m"])
                lo = [round(min(a[i], b[i]), 3) for i in range(3)]
                hi = [round(max(a[i], b[i]), 3) for i in range(3)]
            cls = nd.get("class", -1)
            self.nodes[nid] = {
                "id": nid, "layer": layer, "parent": nd.get("parent"), "children": [],
                "position": {"x": round(rp[0], 3), "y": round(rp[1], 3), "z": round(rp[2], 3)},
                "class_id": cls,
                "name": nd.get("name", self.names.get(cls, str(cls))) if layer == "object" else None,
                "voxel_count": nd.get("voxel_count", 0),
                "place_id": nd.get("place_id", -1),
                "misses": nd.get("misses", 0),
                "seen_count": nd.get("seen_count", 0),
                "bbox_min": lo, "bbox_max": hi,
            }
            self.order.append(nid)
        for nid, n in self.nodes.items():
            p = n["parent"]
            if p and p in self.nodes:
                self.nodes[p]["children"].append(nid)

    def _load_pose(self, path):
        try:
            with open(path) as f:
                d = json.load(f)
            self.robot = {"x": round(d["x"], 3), "y": round(d["y"], 3), "z": round(d["z"], 3),
                          "stamp_sec": d.get("stamp_sec"), "integrated": d.get("integrated")}
        except Exception:
            self.robot = None

    # ---- 查询原语 ----
    def by_layer(self, layer):
        return [self.nodes[i] for i in self.order if self.nodes[i]["layer"] == layer]

    def nearest_place(self, x, y):
        best, bd = None, 1e18
        for n in self.by_layer("place"):
            d = (n["position"]["x"] - x) ** 2 + (n["position"]["y"] - y) ** 2
            if d < bd:
                bd, best = d, n
        return best, math.sqrt(bd) if best else None

    def room_of_node(self, n):
        """沿父链找 room(物体可能挂在别的物体上)。"""
        seen = set()
        while n and n["id"] not in seen:
            seen.add(n["id"])
            if n["layer"] == "room":
                return n["id"]
            p = n.get("parent")
            n = self.nodes.get(p) if p else None
        return None


# ------------------------------ 工具实现 ------------------------------
def t_get_robot_pose(sc, _):
    if not sc.robot:
        return {"error": "机器人位姿不可用(节点未运行或尚未积分)"}
    return {"frame": "odom", "position": sc.robot,
            "note": "ROS 坐标系(z-up), 传感器原点≈机器人"}


def t_get_robot_room(sc, _):
    if not sc.robot:
        return {"error": "机器人位姿不可用"}
    pl, d = sc.nearest_place(sc.robot["x"], sc.robot["y"])
    if not pl:
        return {"error": "场景图中无 place"}
    rid = sc.room_of_node(pl)
    objs = [n for n in sc.by_layer("object") if sc.room_of_node(n) == rid]
    return {"room_id": rid, "via_place": pl["id"],
            "distance_to_place_m": round(d, 3),
            "object_count": len(objs),
            "objects": sorted({n["name"] for n in objs})}


def t_list_rooms(sc, _):
    out = []
    for r in sc.by_layer("room"):
        objs = [n for n in sc.by_layer("object") if sc.room_of_node(n) == r["id"]]
        places = [c for c in r["children"] if sc.nodes[c]["layer"] == "place"]
        out.append({"room_id": r["id"], "center": r["position"],
                    "place_count": len(places), "object_count": len(objs),
                    "object_kinds": sorted({n["name"] for n in objs})})
    return {"room_count": len(out), "rooms": out}


def t_room_contents(sc, args):
    rid = args.get("room_id", "")
    if rid not in sc.nodes or sc.nodes[rid]["layer"] != "room":
        return {"error": "房间不存在: %s" % rid,
                "available": [r["id"] for r in sc.by_layer("room")]}
    objs = [n for n in sc.by_layer("object") if sc.room_of_node(n) == rid]
    return {"room_id": rid, "center": sc.nodes[rid]["position"],
            "object_count": len(objs),
            "objects": [{"id": n["id"], "name": n["name"], "position": n["position"],
                         "voxel_count": n["voxel_count"], "seen_count": n["seen_count"],
                         "on_top_of": n["parent"] if (n["parent"] or "").startswith("object") else None}
                        for n in objs]}


def t_find_object(sc, args):
    q = str(args.get("name", "")).lower()
    hits = [n for n in sc.by_layer("object") if q in (n["name"] or "").lower()]
    return {"query": q, "match_count": len(hits),
            "matches": [{"id": n["id"], "name": n["name"], "position": n["position"],
                         "room": sc.room_of_node(n), "bbox_min": n["bbox_min"],
                         "bbox_max": n["bbox_max"], "voxel_count": n["voxel_count"],
                         "seen_count": n["seen_count"]} for n in hits]}


def t_get_object(sc, args):
    oid = args.get("object_id", "")
    n = sc.nodes.get(oid)
    if not n or n["layer"] != "object":
        return {"error": "物体不存在: %s" % oid}
    return {"id": n["id"], "name": n["name"], "class_id": n["class_id"],
            "position": n["position"], "bbox_min": n["bbox_min"], "bbox_max": n["bbox_max"],
            "voxel_count": n["voxel_count"], "seen_count": n["seen_count"],
            "misses": n["misses"], "nearest_place": "place:%d" % n["place_id"],
            "room": sc.room_of_node(n),
            "parent": n["parent"],
            "parent_layer": sc.nodes[n["parent"]]["layer"] if n["parent"] in sc.nodes else None,
            "children": [{"id": c, "name": sc.nodes[c]["name"]} for c in n["children"]]}


def t_get_relations(sc, args):
    nid = args.get("node_id", "")
    n = sc.nodes.get(nid)
    if not n:
        return {"error": "节点不存在: %s" % nid}
    chain, cur = [], n.get("parent")
    while cur and cur in sc.nodes:
        chain.append({"id": cur, "layer": sc.nodes[cur]["layer"]})
        cur = sc.nodes[cur].get("parent")
    return {"id": nid, "layer": n["layer"],
            "parent": n["parent"], "ancestors": chain,
            "children": [{"id": c, "layer": sc.nodes[c]["layer"],
                          "name": sc.nodes[c]["name"]} for c in n["children"]]}


def t_scene_summary(sc, _):
    counts = {}
    for n in sc.nodes.values():
        counts[n["layer"]] = counts.get(n["layer"], 0) + 1
    stacked = [n for n in sc.by_layer("object") if (n["parent"] or "").startswith("object")]
    return {"layers": counts,
            "dsg_meta": sc.meta,
            "robot_available": sc.robot is not None,
            "objects_on_other_objects": [
                {"id": n["id"], "name": n["name"], "on_top_of": n["parent"],
                 "support_name": sc.nodes[n["parent"]]["name"]} for n in stacked],
            "map_dir": sc.map_dir}


# ---------------- 在线通道: 直连节点内存(TCP RPC, 亚毫秒) ----------------
class LiveRPC:
    """连 smc_live_node 的查询端口, 直接查它内存里的 SceneGraph。
    节点没跑 -> 抛异常, 上层回退到读存档(离线查询)。"""

    def __init__(self, host="127.0.0.1", port=18080, timeout=1.5):
        self.host, self.port, self.timeout = host, port, timeout

    def call(self, op, **kw):
        import socket
        req = dict(kw); req["op"] = op
        with socket.create_connection((self.host, self.port), self.timeout) as s:
            s.sendall((json.dumps(req) + "\n").encode())
            buf = b""
            while not buf.endswith(b"\n"):
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
        return json.loads(buf.decode())


TOOLS = [
    ("get_robot_pose", "获取机器人当前位置坐标(ROS odom 系)", {}, t_get_robot_pose),
    ("get_robot_room", "查询机器人当前所在房间及该房间内的物体", {}, t_get_robot_room),
    ("list_rooms", "列出所有房间(中心坐标/place 数/物体数/物体种类)", {}, t_list_rooms),
    ("room_contents", "查询某个房间里有什么物体",
     {"room_id": {"type": "string", "description": "房间 id, 如 room:3"}}, t_room_contents),
    ("find_object", "按名称查询物体所在位置(支持子串匹配)",
     {"name": {"type": "string", "description": "物体名, 如 cabinets / bathtub"}}, t_find_object),
    ("get_object", "获取某个物体的详情(位置/bbox/所属房间/父子)",
     {"object_id": {"type": "string", "description": "物体 id, 如 object:0000000d"}}, t_get_object),
    ("get_relations", "查询任意节点的父子结构关系(祖先链 + 子节点)",
     {"node_id": {"type": "string", "description": "节点 id(building/room/place/object)"}},
     t_get_relations),
    ("scene_summary", "场景图总览(各层节点数 + 支撑关系物体)", {}, t_scene_summary),
]
TOOLMAP = {t[0]: t for t in TOOLS}


def tool_schema(t):
    name, desc, props, _ = t
    return {"name": name, "description": desc,
            "inputSchema": {"type": "object", "properties": props,
                            "required": list(props.keys())}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", default=DEF_MAP)
    ap.add_argument("--labelspace", default=DEF_LS)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=18080)
    ap.add_argument("--offline", action="store_true", help="只读存档, 不连节点")
    a = ap.parse_args()
    rpc = LiveRPC(a.host, a.port)

    def respond(rid, result=None, error=None):
        msg = {"jsonrpc": "2.0", "id": rid}
        if error is not None:
            msg["error"] = error
        else:
            msg["result"] = result
        sys.stdout.write(json.dumps(msg, ensure_ascii=False) + "\n")
        sys.stdout.flush()

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception:
            continue
        m, rid = req.get("method"), req.get("id")
        if m == "initialize":
            respond(rid, {"protocolVersion": "2024-11-05",
                          "capabilities": {"tools": {}},
                          "serverInfo": {"name": "semantic-map-core", "version": "0.1.0"}})
        elif m == "notifications/initialized":
            continue
        elif m == "tools/list":
            respond(rid, {"tools": [tool_schema(t) for t in TOOLS]})
        elif m == "tools/call":
            p = req.get("params", {})
            name, args = p.get("name"), p.get("arguments", {}) or {}
            if name not in TOOLMAP:
                respond(rid, error={"code": -32601, "message": "unknown tool: %s" % name})
                continue
            try:
                # 优先在线: 直查节点内存(亚毫秒, 数据即当前 cycle)
                out, src = None, "live_rpc"
                if not a.offline:
                    try:
                        out = rpc.call(name, **args)
                    except Exception:
                        out, src = None, "archive"
                if out is None:                          # 兜底: 节点没跑 -> 读存档
                    sc = Scene(a.map_dir, a.labelspace)
                    out = TOOLMAP[name][3](sc, args)
                    src = "archive"
                if isinstance(out, dict):
                    out["_source"] = src
                respond(rid, {"content": [{"type": "text",
                                           "text": json.dumps(out, ensure_ascii=False, indent=2)}]})
            except Exception as e:
                respond(rid, error={"code": -32000, "message": "%s: %s" % (type(e).__name__, e)})
        elif m == "ping":
            respond(rid, {})
        else:
            respond(rid, error={"code": -32601, "message": "unknown method: %s" % m})


if __name__ == "__main__":
    main()
