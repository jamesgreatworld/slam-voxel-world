# -*- coding: utf-8 -*-
"""gvd_live.py — 增量式 GVD/场景图节点(在线,配合 live_ros_stream 的快照)。

盯 <vxw_dir>/obsmap.npz 的 mtime;每次更新:
  紧致裁剪 -> ESDF -> GVD -> places 图 -> 房间(Louvain) -> 物体聚类
  -> scene_graph.merge_observation 增量合并(物体 id 稳定, 消失计数删除)
  -> 写 scene_graph.json + 发布 /dsg MarkerArray(rviz 分层显示)。

不走 pipeline.run 的 .vxw 染色路径(House 语义 id 高会撞 palette 255 上限),
计算链与 pipeline.run 逐步等价(band_max=0 语义: 不做 shell-band 过滤)。
rclpy 懒加载, Windows 下 import 本模块不报错。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))

from m3_adapter.obsmap import ObsMap  # noqa: E402
from m3_adapter.gvd import field  # noqa: E402
from m3_adapter.gvd.graph import (  # noqa: E402
    skeleton_to_graph, prune_spurs, merge_close, drop_small_components,
)
from m3_adapter.gvd.rooms import partition_rooms  # noqa: E402
from m3_adapter.gvd.objects import extract_objects, link_to_places  # noqa: E402
from m3_adapter.gvd.scene_graph import build_scene_graph, merge_observation  # noqa: E402
from m3_adapter.uhumans2_to_vxw import load_label_space  # noqa: E402

ROOM_COLORS = [(255, 80, 80), (80, 255, 80), (80, 80, 255), (255, 255, 80),
               (255, 80, 255), (80, 255, 255), (255, 160, 40), (160, 80, 255),
               (40, 255, 160), (255, 120, 160), (160, 255, 80), (120, 160, 255)]
# 结构类关键字(不算"物体": 墙/顶/地/建筑本体等大面积结构面) —— 按名字数据驱动,
# 跨数据集通用, 取代 objects.py 里写死的 uHumans2 数字 id {3,4,19}。
STRUCT_KW = ("wall", "ceiling", "roof", "building", "floor", "ground",
             "carpet", "rug", "pillar", "column", "beam", "stair", "sky")


def _structure_ids(label_names, extra=()):
    ids = set(int(x) for x in extra)
    for i, n in label_names.items():
        if any(k in str(n).lower() for k in STRUCT_KW):
            ids.add(int(i))
    return ids
BOX_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]


def yup2ros(p):
    """ros_zup_to_vxw_yup 的逆: (x,y,z)_ros = (-Z, -X, Y)_vxw"""
    return np.array([-p[2], -p[0], p[1]], dtype=float)


def compute_scene_graph(obs, label_names, min_component=30, d_min=0.20,
                        theta_sep=0.40, prune_m=0.3, merge_m=0.2, drop_small=5,
                        door_clearance_m=0.85, min_room_nodes=8, obj_min_voxels=20,
                        exclude_extra=()):
    vs = obs.voxel_size
    occ_full = obs.occupancy_mask()
    if int(occ_full.sum()) < 500:
        return None, None, None, None
    # 紧致裁剪(pad=1), 与批处理 densify 网格语义一致
    idx = np.argwhere(occ_full)
    lo = np.maximum(idx.min(0) - 1, 0)
    hi = np.minimum(idx.max(0) + 2, np.array(occ_full.shape))
    sl = tuple(slice(a, b) for a, b in zip(lo, hi))
    occ = occ_full[sl]
    sem = obs.semantic_grid()[sl]
    free = obs.observed_free_mask()[sl] & ~occ
    vmin = obs.vmin + lo

    occ = field.denoise_occupancy(occ, min_component)
    dist_m, parent = field.compute_esdf(occ, vs)
    gvd = field.extract_gvd(free, dist_m, parent, vs, d_min=d_min, theta_sep=theta_sep)
    gvd = field.thin_gvd(gvd)
    # GVD 骨架体素 -> vxw 世界网格坐标(可视化用)
    gvd_cells = np.argwhere(gvd) + np.asarray(vmin, dtype=np.int64)
    g = skeleton_to_graph(gvd, dist_m, vs, vmin, merge_radius_m=0.15)
    if prune_m > 0:
        g = prune_spurs(g, prune_m)
    if merge_m > 0:
        g = merge_close(g, merge_m)
    if drop_small > 1:
        g = drop_small_components(g, drop_small)
    if not g.nodes:
        return None, None, None, None
    # 房间: clearance 切门口(取代 Louvain), 再合并嵌套/过小
    for nd, r in zip(g.nodes,
                     _partition_rooms_clearance(g, door_clearance_m, min_room_nodes)):
        nd.room = int(r)
    _merge_nested_rooms(g, vs)
    objs = extract_objects(occ, sem, vs, vmin, label_names=label_names,
                           structure_labels=_structure_ids(label_names, exclude_extra),
                           min_voxels=obj_min_voxels)
    link_to_places(objs, g)
    surf = _surface_tiles(free, occ, vmin, vs)
    return g, objs, surf, gvd_cells


def _partition_rooms_clearance(g, door_clearance_m=0.85, min_room_nodes=4):
    """Hydra 式房间分割: 在 clearance 低的门口 place 处断图 -> 连通分量=房间。
    门~0.8m 宽 => 门口 place 的 clearance~0.4m, 房间中心 clearance 大(1.5-3m)。
    门口/未标节点多源 BFS 归就近房间; 过小房间并入最强相邻房间。"""
    import collections
    n = len(g.nodes)
    if n == 0:
        return []
    adj = g.adjacency()
    is_door = [g.nodes[i].clearance_m < door_clearance_m for i in range(n)]
    room_of = [-1] * n
    rid = 0
    # 1. 非门口节点的连通分量 = 房间核
    for s in range(n):
        if is_door[s] or room_of[s] != -1:
            continue
        stack = [s]; room_of[s] = rid
        while stack:
            u = stack.pop()
            for v in adj[u]:
                if not is_door[v] and room_of[v] == -1:
                    room_of[v] = rid; stack.append(v)
        rid += 1
    # 2. 门口/未标节点 -> 多源 BFS 从已标房间扩散
    frontier = collections.deque(i for i in range(n) if room_of[i] != -1)
    while frontier:
        u = frontier.popleft()
        for v in adj[u]:
            if room_of[v] == -1:
                room_of[v] = room_of[u]; frontier.append(v)
    for i in range(n):          # 孤立节点各自成房间
        if room_of[i] == -1:
            room_of[i] = rid; rid += 1
    # 3. 过小房间并入最常见相邻房间(迭代到稳定)
    changed = True
    while changed:
        changed = False
        sizes = collections.Counter(room_of)
        small = {r for r, c in sizes.items() if c < min_room_nodes}
        if not small:
            break
        for r in small:
            members = [i for i in range(n) if room_of[i] == r]
            nbr_rooms = collections.Counter(
                room_of[v] for i in members for v in adj[i] if room_of[v] != r)
            if nbr_rooms:
                tgt = nbr_rooms.most_common(1)[0][0]
                for i in members:
                    room_of[i] = tgt
                changed = True
    # 4. 房间号重新紧凑编号(0..k-1)
    remap = {r: k for k, r in enumerate(sorted(set(room_of)))}
    return [remap[r] for r in room_of]


def _merge_nested_rooms(g, vs, margin_m=0.3):
    """先验规则: 房间不得嵌套 —— 小房间的 places 包围盒若被大房间包含,
    并入大房间(按成员数小并入大)。"""
    from collections import defaultdict
    while True:
        boxes = defaultdict(lambda: [np.full(2, 1e9), np.full(2, -1e9), 0])
        for nd in g.nodes:
            p = np.asarray(nd.idx, dtype=float)  # (X, Y_up, Z) 网格
            xy = np.array([p[0], p[2]]) * vs
            b = boxes[int(nd.room)]
            b[0] = np.minimum(b[0], xy)
            b[1] = np.maximum(b[1], xy)
            b[2] += 1
        merged = None
        rooms = list(boxes)
        for a in rooms:
            for b in rooms:
                if a == b:
                    continue
                la, ha, na = boxes[a]
                lb, hb, nb = boxes[b]
                if na <= nb and np.all(la >= lb - margin_m) and np.all(ha <= hb + margin_m):
                    merged = (a, b)
                    break
            if merged:
                break
        if merged is None:
            return
        small, big = merged
        for nd in g.nodes:
            if int(nd.room) == small:
                nd.room = big


def _surface_tiles(free, occ, vmin, vs, clearance_vox=4):
    """Hydra 式地面可通行区域: 地板高度上方的 free 列(带净空)投成 2D 瓦片。
    返回 (N,3) vxw 世界坐标(瓦片中心, 地板高度)。"""
    if not free.any():
        return None
    occ_y = np.argwhere(occ)[:, 1]
    y_floor = int(np.percentile(occ_y, 3))              # 地板层(占据)
    y0, y1 = y_floor + 1, y_floor + 1 + clearance_vox   # 其上净空带
    band = free[:, y0:y1, :]
    walkable = band.all(axis=1)                          # (X,Z) 全 free 才算可通行
    ij = np.argwhere(walkable)
    if len(ij) == 0:
        return None
    pts = np.zeros((len(ij), 3))
    pts[:, 0] = (ij[:, 0] + vmin[0] + 0.5) * vs
    pts[:, 1] = (y_floor + vmin[1] + 1.0) * vs
    pts[:, 2] = (ij[:, 1] + vmin[2] + 0.5) * vs
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("vxw_dir", type=Path, help="live_ros_stream 的输出目录(含 obsmap.npz)")
    ap.add_argument("--labelspace", type=Path, required=True)
    ap.add_argument("--frame", default="odom")
    ap.add_argument("--interval", type=float, default=10.0, help="最小循环间隔秒")
    ap.add_argument("--out-json", type=Path, default=None)
    ap.add_argument("--door-clearance", type=float, default=0.85,
                    help="房间分割门口净空阈值(m): place clearance 低于此=门口切断点")
    ap.add_argument("--min-room-nodes", type=int, default=8,
                    help="小于此 place 数的房间并入相邻房间")
    ap.add_argument("--show-balls", action="store_true",
                    help="3D places 画成 clearance 半径的自由空间球(全局导航 roadmap)")
    a = ap.parse_args()

    import rclpy
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from rclpy.qos import QoSProfile, DurabilityPolicy

    label_names = load_label_space(a.labelspace)
    obsmap_path = a.vxw_dir / "obsmap.npz"
    out_json = a.out_json or (a.vxw_dir / "scene_graph_live.json")

    rclpy.init()
    node = rclpy.create_node("gvd_live")
    pub = node.create_publisher(
        MarkerArray, "/dsg",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log = node.get_logger()

    def room_color(rid, alpha=1.0):
        c = ROOM_COLORS[int(str(rid).split(":")[-1]) % len(ROOM_COLORS)]
        return ColorRGBA(r=c[0] / 255.0, g=c[1] / 255.0, b=c[2] / 255.0, a=alpha)

    def publish_sg(sg, g, gvd_cells, surf=None):
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        mid = [0]

        def mk(t, ns):
            m = Marker()
            m.header.frame_id = a.frame
            m.ns = ns
            m.id = mid[0]; mid[0] += 1
            m.type = t
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        nodes = sg.nodes
        rooms = [n for n in nodes.values() if n.layer == "room"]
        places = [n for n in nodes.values() if n.layer == "place"]
        objects = [n for n in nodes.values() if n.layer == "object"]

        # --- C. GVD 骨架曲线(自由空间中轴, 青色细点) ---
        if gvd_cells is not None and len(gvd_cells) > 0:
            sk = mk(Marker.CUBE_LIST, "gvd_skeleton")
            sk.scale.x = sk.scale.y = sk.scale.z = g.voxel_size * 0.6
            skc = ColorRGBA(r=0.0, g=0.9, b=0.9, a=0.9)
            for c in gvd_cells:
                x, y, z = yup2ros((c + 0.5) * g.voxel_size)
                sk.points.append(Point(x=x, y=y, z=z))
                sk.colors.append(skc)
            arr.markers.append(sk)

        # --- B. 3D places = clearance 空间球 + roadmap 边(全局导航) ---
        gpos = g.positions_m()
        pcenters = [yup2ros(gpos[i]) for i in range(len(g.nodes))]
        # roadmap 边(白色细线, 连通的可通行图)
        road = mk(Marker.LINE_LIST, "roadmap")
        road.scale.x = 0.015
        rc = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.7)
        for ea, eb, _ln in g.edges:
            if ea < len(pcenters) and eb < len(pcenters):
                pa, pb = pcenters[ea], pcenters[eb]
                road.points.append(Point(x=pa[0], y=pa[1], z=pa[2]))
                road.points.append(Point(x=pb[0], y=pb[1], z=pb[2]))
                road.colors.append(rc); road.colors.append(rc)
        arr.markers.append(road)
        # place 节点小球(按房间着色)
        sp = mk(Marker.SPHERE_LIST, "places")
        sp.scale.x = sp.scale.y = sp.scale.z = 0.12
        for i, p in enumerate(places):
            x, y, z = yup2ros(p.pos_m)
            sp.points.append(Point(x=x, y=y, z=z))
            sp.colors.append(room_color(p.parent or "0", 0.95))
        arr.markers.append(sp)
        # 自由空间球(半径=clearance): 每球一个半透明 SPHERE
        if a.show_balls:
            for i, nd in enumerate(g.nodes):
                cx, cy, cz = pcenters[i]
                b = mk(Marker.SPHERE, "free_balls")
                b.pose.position.x, b.pose.position.y, b.pose.position.z = cx, cy, cz
                d = 2.0 * max(nd.clearance_m, 0.05)
                b.scale.x = b.scale.y = b.scale.z = d
                rc2 = room_color("place:%d" % (nd.room if nd.room >= 0 else 0), 0.10)
                b.color = rc2
                arr.markers.append(b)

        lines = mk(Marker.LINE_LIST, "room_edges")
        lines.scale.x = 0.01
        for r in rooms:
            x, y, z = yup2ros(r.pos_m); zl = z + 3.0
            s = mk(Marker.SPHERE, "rooms")
            s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, zl
            s.scale.x = s.scale.y = s.scale.z = 0.45
            s.color = room_color(r.id)
            arr.markers.append(s)
            t = mk(Marker.TEXT_VIEW_FACING, "room_labels")
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, zl + 0.4
            t.scale.z = 0.35; t.text = r.label or r.id
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            arr.markers.append(t)
            for cid in r.children:
                c = nodes.get(cid)
                if c is None or c.layer != "place":
                    continue
                cx, cy, cz = yup2ros(c.pos_m)
                lines.points.append(Point(x=x, y=y, z=zl))
                lines.points.append(Point(x=cx, y=cy, z=cz))
                lines.colors.append(room_color(r.id, 0.35))
                lines.colors.append(room_color(r.id, 0.35))
        arr.markers.append(lines)

        import colorsys
        wire = mk(Marker.LINE_LIST, "object_bbox")
        wire.scale.x = 0.02
        for o in objects:
            at = o.attrs
            if "bbox_min_m" not in at:
                continue
            lo = np.minimum(yup2ros(at["bbox_min_m"]), yup2ros(at["bbox_max_m"]))
            hi = np.maximum(yup2ros(at["bbox_min_m"]), yup2ros(at["bbox_max_m"]))
            h = (int(at.get("class", 0)) * 137.508) % 360.0 / 360.0
            r_, g_, b_ = colorsys.hsv_to_rgb(h, 0.75, 0.95)
            col = ColorRGBA(r=r_, g=g_, b=b_, a=1.0)
            corners = [np.array([lo[0] if i & 1 == 0 else hi[0],
                                 lo[1] if i & 2 == 0 else hi[1],
                                 lo[2] if i & 4 == 0 else hi[2]]) for i in range(8)]
            for e0, e1 in BOX_EDGES:
                wire.points.append(Point(x=corners[e0][0], y=corners[e0][1], z=corners[e0][2]))
                wire.points.append(Point(x=corners[e1][0], y=corners[e1][1], z=corners[e1][2]))
                wire.colors.append(col); wire.colors.append(col)
            t = mk(Marker.TEXT_VIEW_FACING, "object_labels")
            t.pose.position.x, t.pose.position.y = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
            t.pose.position.z = hi[2] + 0.12
            t.scale.z = 0.16
            cls = at.get("class")
            t.text = label_names.get(cls, str(o.label))
            t.color = ColorRGBA(r=1.0, g=0.8, b=0.4, a=1.0)
            arr.markers.append(t)
            # Hydra 式物体节点: 头顶矩形块 + 连线(下连物体, 上连所属 place)
            nc = mk(Marker.CUBE, "object_nodes")
            cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
            nz = hi[2] + 0.8
            nc.pose.position.x, nc.pose.position.y, nc.pose.position.z = cx, cy, nz
            nc.scale.x = nc.scale.y = nc.scale.z = 0.14
            nc.color = col
            arr.markers.append(nc)
            el = mk(Marker.LINE_LIST, "object_edges")
            el.scale.x = 0.012
            el.points.append(Point(x=cx, y=cy, z=nz))
            el.points.append(Point(x=cx, y=cy, z=hi[2]))
            el.colors.append(col); el.colors.append(col)
            pid = at.get("place_id")
            pn = nodes.get("place:%s" % pid) if pid is not None else None
            if pn is not None:
                px, py, pz = yup2ros(pn.pos_m)
                el.points.append(Point(x=cx, y=cy, z=nz))
                el.points.append(Point(x=px, y=py, z=pz))
                c2 = ColorRGBA(r=col.r, g=col.g, b=col.b, a=0.45)
                el.colors.append(c2); el.colors.append(c2)
            arr.markers.append(el)
            # --- D. 支撑父子(杯在桌上): 父=另一物体 -> 品红实线连两物体中心 ---
            par = nodes.get(o.parent) if o.parent else None
            if par is not None and par.layer == "object":
                px, py, pz = yup2ros(par.pos_m)
                se = mk(Marker.LINE_LIST, "support_edges")
                se.scale.x = 0.03
                sc = ColorRGBA(r=1.0, g=0.1, b=1.0, a=0.95)
                ox, oy, oz = yup2ros(o.pos_m)
                se.points.append(Point(x=ox, y=oy, z=oz))
                se.points.append(Point(x=px, y=py, z=pz))
                se.colors.append(sc); se.colors.append(sc)
                arr.markers.append(se)
        arr.markers.append(wire)

        # 地面可通行区域(surface places)
        if surf is not None and len(surf) > 0:
            sm = mk(Marker.CUBE_LIST, "surface_places")
            sm.scale.x = sm.scale.y = 0.095
            sm.scale.z = 0.02
            c = ColorRGBA(r=0.15, g=0.85, b=0.75, a=0.55)
            for p in surf:
                x, y, z = yup2ros(p)
                sm.points.append(Point(x=x, y=y, z=z))
                sm.colors.append(c)
            arr.markers.append(sm)
        pub.publish(arr)

    sg = None
    last_mtime = 0.0
    log.info("gvd_live watching %s (interval>=%.0fs)" % (obsmap_path, a.interval))
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        try:
            mt = obsmap_path.stat().st_mtime
        except FileNotFoundError:
            time.sleep(1.0)
            continue
        if mt <= last_mtime:
            time.sleep(0.5)
            continue
        last_mtime = mt
        t0 = time.time()
        try:
            obs = ObsMap.load(obsmap_path)
            g, objs, surf, gvd_cells = compute_scene_graph(
                obs, label_names, door_clearance_m=a.door_clearance,
                min_room_nodes=a.min_room_nodes)
        except Exception as e:  # 快照写一半等瞬态问题, 下轮重试
            log.warn("cycle failed: %s" % str(e)[:200])
            time.sleep(2.0)
            continue
        if g is None:
            time.sleep(1.0)
            continue
        if sg is None:
            sg = build_scene_graph(g, objs)
            stats = {"mode": "initial"}
        else:
            sg, stats = merge_observation(sg, g, objs)
        out_json.write_text(json.dumps(sg.to_dict(), indent=2))
        counts = {l: sum(1 for n in sg.nodes.values() if n.layer == l)
                  for l in ("room", "place", "object")}
        publish_sg(sg, g, gvd_cells, surf)
        log.info("cycle %.1fs  rooms=%d places=%d objects=%d  merge=%s"
                 % (time.time() - t0, counts["room"], counts["place"],
                    counts["object"], str(stats)[:120]))
        # 间隔控制
        el = time.time() - t0
        if el < a.interval:
            time.sleep(a.interval - el)


if __name__ == "__main__":
    main()
