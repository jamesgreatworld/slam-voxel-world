# -*- coding: utf-8 -*-
"""live_ros_stream.py — Hydra 式在线语义建图节点(订阅话题 + TF,不读 GT)。

订阅(与 Hydra 输入同构):
  --rgb-topic       sensor_msgs/Image  rgb8      (可选,--use-rgb 开启颜色通道)
  --depth-topic     sensor_msgs/Image  32FC1     米制深度
  --seg-topic       sensor_msgs/Image  mono8     原始语义标签 id(TartanGround 风格)
  --camera-info     sensor_msgs/CameraInfo       内参
  TF: <map-frame> ← <cam-frame>                  位姿唯一来源 = SLAM(lightning)的 TF 树

流程: ApproximateTimeSync(depth,seg[,rgb]) → tf2 查位姿 → unproject → ObsMap.integrate_frame
      → 周期快照 obsmap.npz + live.vxw(语义体素),Ctrl-C 时写最终产物。

跨平台注意: rclpy/tf2 仅在 main() 内导入 —— Windows(pixi, 无 ROS)下 import 本模块不报错,
其余依赖(numpy/yaml + m3_adapter 内部模块)与既有代码一致,不用 cv2/cv_bridge。

用法(VM, 配合 ros2 bag play house_sim_bag + lightning + base→cam 静态 TF):
  python3 m3_adapter/live_ros_stream.py out/house_live.vxw \
      --labelspace ~/Semantic_map_ws/src/Hydra/config/label_spaces/tartanground_house_label_space.yaml \
      --use-rgb --pixel-stride 4 --snapshot-every 50
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))

from m3_adapter.common import ros_zup_to_vxw_yup  # noqa: E402
from m3_adapter.obsmap import ObsMap  # noqa: E402
from m3_adapter.obsmap_export import occupancy_to_vxw  # noqa: E402
from m3_adapter.uhumans2_to_vxw import (  # noqa: E402
    load_label_space, build_palette, unproject_depth,
)


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_vxw", type=Path, help="输出 .vxw 目录(obsmap.npz 同目录)")
    ap.add_argument("--labelspace", type=Path, required=True,
                    help="Hydra 风格 label_space yaml(label id → name)")
    ap.add_argument("--rgb-topic", default="/tg/rgb")
    ap.add_argument("--depth-topic", default="/tg/depth")
    ap.add_argument("--seg-topic", default="/tg/semantic")
    ap.add_argument("--camera-info-topic", default="/tg/camera_info")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--cam-frame", default="cam_optical")
    ap.add_argument("--use-rgb", action="store_true", help="启用逐点 RGB 颜色通道")
    ap.add_argument("--pixel-stride", type=int, default=4)
    ap.add_argument("--depth-min", type=float, default=0.2)
    ap.add_argument("--depth-max", type=float, default=15.0)
    ap.add_argument("--free-margin", type=float, default=0.10)
    ap.add_argument("--voxel-size", type=float, default=0.10)
    ap.add_argument("--bounds", default="-20,-12,-3,8,10,5",
                    help="地图边界(ROS z-up, 米): x0,y0,z0,x1,y1,z1")
    ap.add_argument("--snapshot-every", type=int, default=50,
                    help="每 N 帧写一次 obsmap.npz + live .vxw 快照")
    ap.add_argument("--tf-timeout", type=float, default=0.15,
                    help="单帧 TF 查询等待秒数(查不到丢帧)")
    ap.add_argument("--publish-voxels", action="store_true",
                    help="增量发布语义着色体素 CUBE_LIST 到 /semantic_voxels")
    ap.add_argument("--publish-every", type=int, default=10,
                    help="每积分 N 帧发布一次体素可视化")
    ap.add_argument("--see-through", action="store_true",
                    help="结构类体素(墙/顶/地/门窗)半透明, 可透墙看物体")
    ap.add_argument("--structure-alpha", type=float, default=0.28)
    ap.add_argument("--lidar-carve", action="store_true",
                    help="订阅 360° 雷达做自由空间雕刻(几何/free 由雷达负责, "
                         "语义由相机负责) — 房间/GVD 覆盖不再受相机 FOV 限制")
    ap.add_argument("--lidar-topic", default="/livox/lidar")
    ap.add_argument("--lidar-frame", default="lidar")
    ap.add_argument("--lidar-decimate", type=int, default=8)
    ap.add_argument("--lidar-max-range", type=float, default=20.0)
    return ap


STRUCTURE_KEYWORDS = ("wall", "ceiling", "floor", "stair", "door", "window",
                      "blind", "roof", "beam")


def _label_color(label_id: int):
    """确定性鲜明配色: 黄金角遍历色相, 同一类跨节点/跨次运行同色。
    label 0 = 占据但语义未知(如仅被雷达看到) -> 中性灰。"""
    if label_id == 0:
        return (0.55, 0.55, 0.55)
    import colorsys
    h = (label_id * 137.508) % 360.0 / 360.0
    r, g, b = colorsys.hsv_to_rgb(h, 0.75, 0.95)
    return (r, g, b)


def main() -> None:
    args = build_argparser().parse_args()

    # ---- ROS 依赖懒加载(Windows 兼容:无 ROS 环境 import 本文件不炸) ----
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from message_filters import Subscriber, ApproximateTimeSynchronizer
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA

    label_names = load_label_space(args.labelspace)
    palette = build_palette(label_names)
    structure_ids = {i for i, n in label_names.items()
                     if any(k in n.lower() for k in STRUCTURE_KEYWORDS)}

    x0, y0, z0, x1, y1, z1 = [float(v) for v in args.bounds.split(",")]
    vs = args.voxel_size
    # ROS z-up → vxw Y-up: (x, z, y)
    vmin = np.array([int(np.floor(x0 / vs)), int(np.floor(z0 / vs)),
                     int(np.floor(y0 / vs))], dtype=np.int64)
    shape = (int(np.ceil((x1 - x0) / vs)), int(np.ceil((z1 - z0) / vs)),
             int(np.ceil((y1 - y0) / vs)))
    obs = ObsMap.new(shape, vmin, vs)

    out_dir = args.out_vxw
    out_dir.mkdir(parents=True, exist_ok=True)
    obsmap_path = out_dir / "obsmap.npz"

    class LiveMapper(Node):
        def __init__(self):
            super().__init__("mc_live_mapper")
            self.K = None
            self.n_int = 0
            self.n_skip_tf = 0
            self.pending = []
            self.tfbuf = Buffer(cache_time=Duration(seconds=60.0))
            self.tflis = TransformListener(self.tfbuf, self)
            self.create_subscription(CameraInfo, args.camera_info_topic,
                                     self._on_info, 10)
            subs = [Subscriber(self, Image, args.depth_topic),
                    Subscriber(self, Image, args.seg_topic)]
            if args.use_rgb:
                subs.append(Subscriber(self, Image, args.rgb_topic))
            self.sync = ApproximateTimeSynchronizer(subs, queue_size=30, slop=0.02)
            self.sync.registerCallback(self._on_frame)
            self.vox_pub = None
            if args.publish_voxels:
                self.vox_pub = self.create_publisher(
                    MarkerArray, '/semantic_voxels',
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.pending_lidar = []
            self.n_lidar = 0
            if args.lidar_carve:
                from livox_ros_driver2.msg import CustomMsg
                self.create_subscription(CustomMsg, args.lidar_topic,
                                         self._on_lidar, 10)

        def _on_lidar(self, m):
            d = args.lidar_decimate
            pts = np.array([(p.x, p.y, p.z) for p in m.points[::d]],
                           dtype=np.float64)
            if len(pts) == 0:
                return
            r = np.linalg.norm(pts, axis=1)
            pts = pts[(r > 0.3) & (r < args.lidar_max_range)]
            self.pending_lidar.append((time.monotonic(), m.header, pts))
            still = []
            for t_in, hdr, pw in self.pending_lidar:
                try:
                    tr = self.tfbuf.lookup_transform(
                        args.map_frame, hdr.frame_id or args.lidar_frame,
                        Time.from_msg(hdr.stamp))
                except Exception:
                    if time.monotonic() - t_in < 5.0:
                        still.append((t_in, hdr, pw))
                    continue
                q = tr.transform.rotation
                t = tr.transform.translation
                T = np.eye(4)
                T[:3, :3] = _quat_to_R(q.x, q.y, q.z, q.w)
                T[:3, 3] = (t.x, t.y, t.z)
                ph = np.concatenate([pw, np.ones((len(pw), 1))], 1)
                pw_w = np.einsum("nj,kj->nk", ph, T)[:, :3]
                P_m = ros_zup_to_vxw_yup(pw_w)
                O_m = ros_zup_to_vxw_yup(T[:3, 3:4].T)[0]
                # 纯几何: 只做占据/free 雕刻, 不带语义(语义由相机通道负责)
                obs.integrate_frame(O_m, P_m, free_margin_m=args.free_margin)
                self.n_lidar += 1
            self.pending_lidar = still[-30:]
            self.get_logger().info(
                "listening: depth=%s seg=%s rgb=%s tf=%s<-%s" %
                (args.depth_topic, args.seg_topic,
                 args.rgb_topic if args.use_rgb else "-",
                 args.map_frame, args.cam_frame))

        def _on_info(self, m):
            if self.K is None:
                self.K = (m.k[0], m.k[4], m.k[2], m.k[5])
                self.get_logger().info("camera_info: fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
                                       % self.K)

        def _on_frame(self, dm, sm, rm=None):
            # 不能在回调里阻塞等 TF(单线程 executor 收不到 /tf 会死锁):
            # 帧先入待处理队列,每次回调把"TF 已就绪"的帧消化掉。
            # 年龄用"入队后经过的墙钟秒数"(time.monotonic)——不能拿节点时钟减
            # 消息戳:bag 回放时两者时钟域不同(录制时刻 vs 当前),会误杀全部帧。
            self.pending.append((time.monotonic(), dm, sm, rm))
            still = []
            for t_in, dm2, sm2, rm2 in self.pending:
                st = Time.from_msg(dm2.header.stamp)
                try:
                    tr = self.tfbuf.lookup_transform(
                        args.map_frame, dm2.header.frame_id or args.cam_frame, st)
                except Exception as e:
                    if time.monotonic() - t_in < 5.0:
                        still.append((t_in, dm2, sm2, rm2))   # TF 未到,下轮再试
                    else:
                        self.n_skip_tf += 1
                        if self.n_skip_tf % 25 == 1:
                            self.get_logger().warn(
                                "丢帧 %d (TF 5s 未就绪): %s" % (self.n_skip_tf, str(e)[:150]))
                    continue
                self._integrate(dm2, sm2, rm2, tr)
            self.pending = still[-50:]

        def _integrate(self, dm, sm, rm, tr):
            if self.K is None:
                return
            if dm.encoding != "32FC1" or sm.encoding != "mono8":
                self.get_logger().error("encoding mismatch: depth=%s seg=%s"
                                        % (dm.encoding, sm.encoding))
                return

            q = tr.transform.rotation
            t = tr.transform.translation
            T = np.eye(4)
            T[:3, :3] = _quat_to_R(q.x, q.y, q.z, q.w)
            T[:3, 3] = (t.x, t.y, t.z)

            s = args.pixel_stride
            depth = np.frombuffer(dm.data, np.float32).reshape(dm.height, dm.width)[::s, ::s]
            seg = np.frombuffer(sm.data, np.uint8).reshape(sm.height, sm.width)[::s, ::s]
            rgb = None
            if rm is not None:
                rgb = np.frombuffer(rm.data, np.uint8).reshape(rm.height, rm.width, 3)[::s, ::s]

            fx, fy, cx, cy = self.K
            xyz_cam, valid = unproject_depth(depth, fx / s, fy / s, cx / s, cy / s,
                                             z_min=args.depth_min, z_max=args.depth_max)
            if xyz_cam.shape[0] == 0:
                return
            labels = seg[valid].astype(np.uint8)
            colors = rgb[valid] if rgb is not None else None

            xyz_h = np.concatenate([xyz_cam, np.ones((len(xyz_cam), 1), np.float32)], 1)
            xyz_world = np.einsum("nj,kj->nk", xyz_h, T.astype(np.float32))[:, :3]
            P_m = ros_zup_to_vxw_yup(xyz_world.astype(np.float64))
            O_m = ros_zup_to_vxw_yup(T[:3, 3:4].T)[0]

            obs.integrate_frame(O_m, P_m, point_labels=labels,
                                free_margin_m=args.free_margin,
                                point_colors=colors)
            self.n_int += 1
            if self.n_int % 25 == 0:
                self.get_logger().info(
                    "integrated=%d lidar=%d skipped_tf=%d occ=%d free=%d"
                    % (self.n_int, self.n_lidar, self.n_skip_tf,
                       int(obs.occupancy_mask().sum()),
                       int(obs.observed_free_mask().sum())))
            if self.vox_pub is not None and self.n_int % args.publish_every == 0:
                self._publish_voxels()
            if self.n_int % args.snapshot_every == 0:
                self._snapshot()

        def _publish_voxels(self):
            occ = obs.occupancy_mask()
            sem = obs.semantic_grid()
            idx = np.argwhere(occ)
            if len(idx) == 0:
                return
            # vxw(Godot) -> ROS: ros_zup_to_vxw_yup 的逆 = (x,y,z)_ros = (-Z, -X, Y)_vxw
            pv = (idx + obs.vmin + 0.5) * obs.voxel_size
            pw = np.stack([-pv[:, 2], -pv[:, 0], pv[:, 1]], 1)
            labels = sem[occ]
            m = Marker()
            m.header.frame_id = args.map_frame
            m.ns = 'semantic_voxels'
            m.id = 0
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = obs.voxel_size * 0.95
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pw]
            lut = {}
            cols = []
            for l in labels:
                l = int(l)
                if l not in lut:
                    r, g, b = _label_color(l)
                    if l == 0:                      # 无语义(雷达补的灰体素): 淡但可见
                        a = 0.15
                    elif args.see_through and l in structure_ids:
                        a = args.structure_alpha
                    else:
                        a = 1.0
                    lut[l] = ColorRGBA(r=r, g=g, b=b, a=a)
                cols.append(lut[l])
            m.colors = cols
            arr = MarkerArray()
            arr.markers = [m]
            self.vox_pub.publish(arr)

        def _snapshot(self):
            t0 = time.time()
            obs.save(obsmap_path)
            occupancy_to_vxw(obs.occupancy_mask(), obs.vmin, obs.voxel_size,
                             out_dir, semantic_grid=obs.semantic_grid(),
                             palette=palette)
            self.get_logger().info("snapshot -> %s (%.2fs)" % (out_dir, time.time() - t0))

    rclpy.init()
    node = LiveMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("final export: integrated=%d skipped_tf=%d"
                               % (node.n_int, node.n_skip_tf))
        obs.save(obsmap_path)
        occupancy_to_vxw(obs.occupancy_mask(), obs.vmin, obs.voxel_size,
                         out_dir, semantic_grid=obs.semantic_grid(),
                         palette=palette)
        node.destroy_node()


if __name__ == "__main__":
    main()
