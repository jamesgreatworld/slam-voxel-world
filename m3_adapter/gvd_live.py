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
BOX_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]


def yup2ros(p):
    """ros_zup_to_vxw_yup 的逆: (x,y,z)_ros = (-Z, -X, Y)_vxw"""
    return np.array([-p[2], -p[0], p[1]], dtype=float)


def compute_scene_graph(obs, label_names, min_component=30, d_min=0.20,
                        theta_sep=0.40, prune_m=0.3, merge_m=0.2, drop_small=5,
                        room_res=0.3, obj_min_voxels=20):
    vs = obs.voxel_size
    occ_full = obs.occupancy_mask()
    if int(occ_full.sum()) < 500:
        return None, None
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
    g = skeleton_to_graph(gvd, dist_m, vs, vmin, merge_radius_m=0.15)
    if prune_m > 0:
        g = prune_spurs(g, prune_m)
    if merge_m > 0:
        g = merge_close(g, merge_m)
    if drop_small > 1:
        g = drop_small_components(g, drop_small)
    if not g.nodes:
        return None, None
    for nd, r in zip(g.nodes, partition_rooms(g, resolution=room_res)):
        nd.room = int(r)
    objs = extract_objects(occ, sem, vs, vmin, label_names=label_names,
                           min_voxels=obj_min_voxels)
    link_to_places(objs, g)
    return g, objs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("vxw_dir", type=Path, help="live_ros_stream 的输出目录(含 obsmap.npz)")
    ap.add_argument("--labelspace", type=Path, required=True)
    ap.add_argument("--frame", default="odom")
    ap.add_argument("--interval", type=float, default=10.0, help="最小循环间隔秒")
    ap.add_argument("--out-json", type=Path, default=None)
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

    def publish_sg(sg):
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

        sp = mk(Marker.SPHERE_LIST, "places")
        sp.scale.x = sp.scale.y = sp.scale.z = 0.12
        for p in places:
            x, y, z = yup2ros(p.pos_m)
            sp.points.append(Point(x=x, y=y, z=z))
            sp.colors.append(room_color(p.parent or "0", 0.95))
        arr.markers.append(sp)

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
        arr.markers.append(wire)
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
            g, objs = compute_scene_graph(obs, label_names)
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
        publish_sg(sg)
        log.info("cycle %.1fs  rooms=%d places=%d objects=%d  merge=%s"
                 % (time.time() - t0, counts["room"], counts["place"],
                    counts["object"], str(stats)[:120]))
        # 间隔控制
        el = time.time() - t0
        if el < a.interval:
            time.sleep(a.interval - el)


if __name__ == "__main__":
    main()
