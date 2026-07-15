# -*- coding: utf-8 -*-
"""nav_frontend.py — 独立的每帧 2D surface-place 前端(地面机器人导航/避障)。

*多层*版本: 建筑可有多层(House 有楼梯)。单层 2D 投影会把上层地板压到下层 ->
规划穿墙。故对地面点高度在线聚类, 楼层高度差 > level_gap 自动分层, 每层独立
维护 2D 可通行栅格 -> 独立代价图 + surface places, 按真实高度堆叠。

穿墙防护两重: (1) 分层杜绝垂直投影穿墙; (2) 层内墙(walls 等非 surface 类且落在
[地面, 地面+robot_height] 高度带) = 障碍格 -> 非可走。

独立于建图后端: 订阅 depth+seg+camera_info + SLAM(lightning) TF, 位姿无 GT。
surface.py 的聚类算法后端也能复用(喂累积栅格)。rclpy 懒加载, Windows import 不炸。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))

from m3_adapter.uhumans2_to_vxw import unproject_depth  # noqa: E402
from m3_adapter.gvd.surface import cluster_surface_places, occupancy_from_counts  # noqa: E402


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _load_surface_labels(labelspace_path):
    import yaml
    spec = yaml.safe_load(open(labelspace_path))
    surf = set(int(x) for x in spec.get("surface_places_labels", []))
    inval = set(int(x) for x in spec.get("invalid_labels", [0]))
    names = {it["label"]: it["name"] for it in spec.get("label_names", [])}
    return surf, inval, names


class Level:
    """一个楼层: 独立的 2D 累加栅格 + 代表高度。"""
    __slots__ = ("surf", "obst", "floor_z", "z_sum", "z_cnt")

    def __init__(self, ny, nx, rep_z):
        self.surf = np.zeros((ny, nx), np.int32)
        self.obst = np.zeros((ny, nx), np.int32)
        self.floor_z = np.full((ny, nx), np.inf, np.float32)
        self.z_sum = float(rep_z)
        self.z_cnt = 1

    @property
    def rep_z(self):
        return self.z_sum / self.z_cnt


def build_argparser():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labelspace", type=Path, required=True)
    ap.add_argument("--depth-topic", default="/tg/depth")
    ap.add_argument("--seg-topic", default="/tg/semantic")
    ap.add_argument("--camera-info-topic", default="/tg/camera_info")
    ap.add_argument("--map-frame", default="odom")
    ap.add_argument("--cam-frame", default="cam_optical")
    ap.add_argument("--bounds", default="-16,-9,5,8",
                    help="导航平面范围(ROS map系,米): xmin,ymin,xmax,ymax")
    ap.add_argument("--res", type=float, default=0.10, help="栅格分辨率(米)")
    ap.add_argument("--robot-height", type=float, default=0.5,
                    help="地面上方多高内的障碍算挡路(米)")
    ap.add_argument("--ground-eps", type=float, default=0.08,
                    help="贴地容差(米)")
    ap.add_argument("--level-gap", type=float, default=0.8,
                    help="楼层分层阈值(米): 地面高度差超过此值 -> 新楼层")
    ap.add_argument("--level-min-pts", type=int, default=200,
                    help="开新楼层所需的最少地面点(抗噪, 防误分层)")
    ap.add_argument("--pixel-stride", type=int, default=4)
    ap.add_argument("--depth-min", type=float, default=0.2)
    ap.add_argument("--depth-max", type=float, default=10.0)
    ap.add_argument("--surface-every", type=int, default=15)
    ap.add_argument("--door-bridge", type=float, default=0.9,
                    help="surface 区域经门连通的桥接距离(m): 跨房 roadmap 边")
    ap.add_argument("--max-levels", type=int, default=4)
    return ap


def main():
    args = build_argparser().parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import OccupancyGrid
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from message_filters import Subscriber, ApproximateTimeSynchronizer
    from tf2_ros import Buffer, TransformListener

    surf_labels, invalid_labels, label_names = _load_surface_labels(args.labelspace)
    surf_arr = np.array(sorted(surf_labels))
    inval_arr = np.array(sorted(invalid_labels))
    x0, y0, x1, y1 = [float(v) for v in args.bounds.split(",")]
    res = args.res
    nx = int(np.ceil((x1 - x0) / res))
    ny = int(np.ceil((y1 - y0) / res))
    levels: list = []                      # 按 rep_z 升序维护

    def assign_levels(z):
        """给一批高度 z 分配楼层索引(可能开新层)。返回 int 数组, -1=本帧未开成。"""
        out = np.full(len(z), -1, np.int64)
        remaining = np.ones(len(z), bool)
        for _ in range(args.max_levels + 1):
            if not remaining.any():
                break
            if levels:
                reps = np.array([lv.rep_z for lv in levels])
                d = np.abs(z[remaining, None] - reps[None, :])
                nearest = d.argmin(1)
                within = d[np.arange(len(nearest)), nearest] <= args.level_gap
                ridx = np.where(remaining)[0]
                out[ridx[within]] = nearest[within]
                remaining[ridx[within]] = False
            if not remaining.any():
                break
            # 剩余点: 取最低的一簇(在 level_gap 内)开新层
            zr = z[remaining]
            if len(zr) < args.level_min_pts or len(levels) >= args.max_levels:
                break
            zmin = zr.min()
            new_band = zr <= zmin + args.level_gap
            if new_band.sum() < args.level_min_pts:
                break
            lv = Level(ny, nx, float(np.median(zr[new_band])))
            levels.append(lv)
            levels.sort(key=lambda L: L.rep_z)
        return out

    class NavFrontend(Node):
        def __init__(self):
            super().__init__("nav_frontend")
            self.K = None
            self.n = 0
            self.n_skip = 0
            self.pending = []
            self.tfbuf = Buffer()
            self.tflis = TransformListener(self.tfbuf, self)
            self.latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_pubs = {}
            self.sp_pub = self.create_publisher(MarkerArray, "/nav/surface_places", self.latch)
            self.create_subscription(CameraInfo, args.camera_info_topic, self._on_info, 10)
            subs = [Subscriber(self, Image, args.depth_topic),
                    Subscriber(self, Image, args.seg_topic)]
            self.sync = ApproximateTimeSynchronizer(subs, queue_size=30, slop=0.02)
            self.sync.registerCallback(self._on_frame)
            self.get_logger().info(
                "nav_frontend(multi-level): surface=%s robot_h=%.2f res=%.2f "
                "grid=%dx%d level_gap=%.2f"
                % (sorted(surf_labels), args.robot_height, res, nx, ny, args.level_gap))

        def _on_info(self, m):
            if self.K is None:
                self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

        def _on_frame(self, dm, sm):
            self.pending.append((time.monotonic(), dm, sm))
            still = []
            for t_in, dm2, sm2 in self.pending:
                try:
                    tr = self.tfbuf.lookup_transform(
                        args.map_frame, dm2.header.frame_id or args.cam_frame,
                        Time.from_msg(dm2.header.stamp))
                except Exception:
                    if time.monotonic() - t_in < 5.0:
                        still.append((t_in, dm2, sm2))
                    continue
                self._integrate(dm2, sm2, tr)
            self.pending = still[-40:]

        def _integrate(self, dm, sm, tr):
            if self.K is None or dm.encoding != "32FC1" or sm.encoding != "mono8":
                return
            s = args.pixel_stride
            depth = np.frombuffer(dm.data, np.float32).reshape(dm.height, dm.width)[::s, ::s]
            seg = np.frombuffer(sm.data, np.uint8).reshape(sm.height, sm.width)[::s, ::s]
            fx, fy, cx, cy = self.K
            xyz_cam, valid = unproject_depth(depth, fx / s, fy / s, cx / s, cy / s,
                                             z_min=args.depth_min, z_max=args.depth_max)
            if xyz_cam.shape[0] == 0:
                return
            labels = seg[valid].astype(np.int32)
            q = tr.transform.rotation
            t = tr.transform.translation
            R = _quat_to_R(q.x, q.y, q.z, q.w)
            pw = xyz_cam @ R.T + np.array([t.x, t.y, t.z])

            gx = np.floor((pw[:, 0] - x0) / res).astype(int)
            gy = np.floor((pw[:, 1] - y0) / res).astype(int)
            inb = (gx >= 0) & (gx < nx) & (gy >= 0) & (gy < ny)
            gx, gy, pz, labels = gx[inb], gy[inb], pw[inb, 2], labels[inb]

            is_surf = np.isin(labels, surf_arr)
            is_inval = np.isin(labels, inval_arr)

            # --- surface 点: 分层, 更新各层地面高度 + 可走命中 ---
            if is_surf.any():
                sx, sy, sz = gx[is_surf], gy[is_surf], pz[is_surf]
                lv_idx = assign_levels(sz)
                for k, lv in enumerate(levels):
                    m = lv_idx == k
                    if not m.any():
                        continue
                    flat = sy[m] * nx + sx[m]
                    np.minimum.at(lv.floor_z.reshape(-1), flat, sz[m])
                    np.add.at(lv.surf.reshape(-1), flat, 1)
                    lv.z_sum += float(sz[m].sum()); lv.z_cnt += int(m.sum())

            # --- 障碍点(墙/家具): 落在某层 [地面+eps, 地面+robot_h] 高度带 -> 该层挡路。
            # 用该层代表地面高度 rep_z(而非逐格 floor_z): 墙格收不到地面点, 逐格高度
            # 永远未知会漏掉墙 -> 墙必须能挡路。每点只归属高度最贴合的那一层。 ---
            obm = (~is_surf) & (~is_inval)
            if obm.any() and levels:
                ox, oy, oz = gx[obm], gy[obm], pz[obm]
                flat = oy * nx + ox
                for lv in levels:
                    band = (oz > lv.rep_z + args.ground_eps) & (oz < lv.rep_z + args.robot_height)
                    if band.any():
                        np.add.at(lv.obst.reshape(-1), flat[band], 1)

            self.n += 1
            if self.n % args.surface_every == 0:
                self._publish_all()

        def _make_grid(self, cost, z):
            g = OccupancyGrid()
            g.header.frame_id = args.map_frame
            g.info.resolution = res
            g.info.width = nx
            g.info.height = ny
            g.info.origin.position.x = x0
            g.info.origin.position.y = y0
            g.info.origin.position.z = float(z)
            g.info.origin.orientation.w = 1.0
            g.data = cost.reshape(-1).astype(np.int8).tolist()
            return g

        def _publish_all(self):
            arr = MarkerArray()
            w = Marker(); w.action = Marker.DELETEALL; arr.markers.append(w)
            mid = [0]

            def mk(t, ns):
                m = Marker(); m.header.frame_id = args.map_frame
                m.ns = ns; m.id = mid[0]; mid[0] += 1; m.type = t
                m.action = Marker.ADD; m.pose.orientation.w = 1.0
                return m

            import colorsys
            summary = []
            for k, lv in enumerate(levels):
                walkable, cost = occupancy_from_counts(lv.surf, lv.obst)
                blocked = lv.obst >= 2
                # 每层独立代价图, origin.z=该层真实高度 -> rviz 按高度堆叠
                topic = "/nav/surface_map_L%d" % k
                if topic not in self.map_pubs:
                    self.map_pubs[topic] = self.create_publisher(OccupancyGrid, topic, self.latch)
                self.map_pubs[topic].publish(self._make_grid(cost, lv.rep_z))

                regions, edges, _lab = cluster_surface_places(
                    walkable, (x0, y0), res, min_cells=15,
                    door_bridge_m=args.door_bridge, blocked=blocked)
                # roadmap 连通分量(该层)
                adj = {r["id"]: set() for r in regions}
                for i, j in edges:
                    adj[i].add(j); adj[j].add(i)
                seen = set(); ncomp = 0
                for r0 in adj:
                    if r0 in seen:
                        continue
                    ncomp += 1; st = [r0]; seen.add(r0)
                    while st:
                        u = st.pop()
                        for v in adj[u]:
                            if v not in seen:
                                seen.add(v); st.append(v)
                summary.append("L%d(z=%.2f):%dsp/%ded/%dcc" %
                               (k, lv.rep_z, len(regions), len(edges), ncomp))

                zc = lv.rep_z + 0.15
                cen = {r["id"]: r["centroid_xy"] for r in regions}
                for r in regions:
                    h = ((r["id"] + k * 40) * 137.508) % 360.0 / 360.0
                    rr, gg, bb = colorsys.hsv_to_rgb(h, 0.55, 1.0)
                    sph = mk(Marker.SPHERE, "L%d_nodes" % k)
                    cx_, cy_ = r["centroid_xy"]
                    sph.pose.position.x, sph.pose.position.y, sph.pose.position.z = cx_, cy_, zc
                    sph.scale.x = sph.scale.y = sph.scale.z = 0.25
                    sph.color = ColorRGBA(r=rr, g=gg, b=bb, a=0.95)
                    arr.markers.append(sph)
                    txt = mk(Marker.TEXT_VIEW_FACING, "L%d_labels" % k)
                    txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = cx_, cy_, zc + 0.3
                    txt.scale.z = 0.20
                    txt.text = "L%d.S%d %.1fm2" % (k, r["id"], r["area_m2"])
                    txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                    arr.markers.append(txt)
                eln = mk(Marker.LINE_LIST, "L%d_edges" % k)
                eln.scale.x = 0.03
                for i, j in edges:
                    ax, ay = cen[i]; bx, by = cen[j]
                    eln.points.append(Point(x=ax, y=ay, z=zc))
                    eln.points.append(Point(x=bx, y=by, z=zc))
                    c = ColorRGBA(r=0.2, g=1.0, b=0.4, a=0.9)
                    eln.colors.append(c); eln.colors.append(c)
                arr.markers.append(eln)
            self.sp_pub.publish(arr)
            self.get_logger().info(
                "frames=%d levels=%d  %s" % (self.n, len(levels), "  ".join(summary)))

    rclpy.init()
    node = NavFrontend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
