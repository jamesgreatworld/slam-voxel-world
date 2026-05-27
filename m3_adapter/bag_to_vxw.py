"""bag_to_vxw — convert a ROS2 rosbag2 (livox) directly into a .vxw world.

M1 of the "Python-replacement-for-Hydra" path. Reads raw `/livox/lidar`
CustomMsg messages from a rosbag2 directory and accumulates the points into a
single .vxw world.

Pose handling: pass `--trajectory <path.txt>` with a TUM-format trajectory
(one row per keyframe: `timestamp_sec tx ty tz qx qy qz qw`, world←sensor),
e.g. produced by lightning_lm_foxy's offline runner. Each lidar frame's
header.stamp is interpolated against the keyframe series (position linear,
quaternion slerp). Without --trajectory, points are appended in the sensor
frame and a moving capture will ghost.

Usage:
    pixi run python m3_adapter/bag_to_vxw.py \\
        E:/aros_slam_ws/rosbag2_dataset/14 \\
        out/bag14_raw.vxw \\
        --voxel-size 0.10 --swap-yz --max-frames 200
"""

from __future__ import annotations

import argparse
import csv
import logging
import sys
import time
import uuid
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.pcd_to_vxw import (  # noqa: E402
    build_palette,
    ros_zup_to_vxw_yup,
    voxelize_and_group,
)


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
log = logging.getLogger("bag_to_vxw")

_COMPRESSION_MAP = {
    "raw": vxw.Compression.RAW,
    "gzip": vxw.Compression.GZIP,
    "zstd": vxw.Compression.ZSTD,
}

_LIVOX_MSG_DIR = Path(
    "E:/aros_slam_ws/lightning_lm_foxy/thirdparty/livox_ros_driver/msg"
)


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"bag_{ts}.log"
    fmt = logging.Formatter(
        "[%(asctime)s] %(levelname)-5s %(name)s  %(message)s", datefmt="%H:%M:%S"
    )
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setFormatter(fmt)
    ch = logging.StreamHandler()
    ch.setFormatter(fmt)
    log.handlers = [fh, ch]
    log.setLevel(logging.INFO)
    return log_path


def _register_livox_types(typestore) -> None:
    from rosbags.typesys import get_types_from_msg

    new_types = {}
    new_types.update(get_types_from_msg(
        (_LIVOX_MSG_DIR / "CustomPoint.msg").read_text(),
        "livox_ros_driver2/msg/CustomPoint",
    ))
    new_types.update(get_types_from_msg(
        (_LIVOX_MSG_DIR / "CustomMsg.msg").read_text(),
        "livox_ros_driver2/msg/CustomMsg",
    ))
    typestore.register(new_types)


def _load_tum_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read TUM-format trajectory file → (times_s, positions Nx3, quats_xyzw Nx4).

    Each line: `timestamp_sec tx ty tz qx qy qz qw` (lightning_lm_foxy default).
    Lines starting with '#' or blank are skipped.
    """
    times, pos, quat = [], [], []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = (
                float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            )
            times.append(t)
            pos.append([tx, ty, tz])
            quat.append([qx, qy, qz, qw])
    times = np.asarray(times, dtype=np.float64)
    pos = np.asarray(pos, dtype=np.float64)
    quat = np.asarray(quat, dtype=np.float64)
    # ensure quaternions are unit
    nq = np.linalg.norm(quat, axis=1, keepdims=True)
    nq[nq == 0] = 1.0
    quat = quat / nq
    # ensure monotone ascending time
    order = np.argsort(times)
    return times[order], pos[order], quat[order]


def _slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    """SLERP for unit quats (xyzw), u in [0,1]."""
    if np.dot(q0, q1) < 0:
        q1 = -q1
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot > 0.9995:
        out = q0 + u * (q1 - q0)
        return out / np.linalg.norm(out)
    theta = np.arccos(dot)
    sin_t = np.sin(theta)
    return (np.sin((1 - u) * theta) / sin_t) * q0 + (np.sin(u * theta) / sin_t) * q1


def _interp_pose(
    t_query: float,
    times: np.ndarray,
    pos: np.ndarray,
    quat: np.ndarray,
) -> np.ndarray | None:
    """Return 4x4 T_world_sensor at t_query, or None if out of range."""
    if t_query < times[0] or t_query > times[-1]:
        return None
    j = int(np.searchsorted(times, t_query, side="right"))
    if j == 0:
        i = 0
        u = 0.0
    elif j >= len(times):
        i = len(times) - 2
        u = 1.0
    else:
        i = j - 1
        dt = times[j] - times[i]
        u = 0.0 if dt <= 0 else (t_query - times[i]) / dt
    p = (1 - u) * pos[i] + u * pos[j if j < len(times) else i + 1]
    q = _slerp(quat[i], quat[j if j < len(times) else i + 1], u)
    qx, qy, qz, qw = q
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def _frame_points(msg) -> np.ndarray:
    """Extract Nx3 points (sensor frame) from a livox CustomMsg deserialized object."""
    pts = msg.points
    arr = np.empty((len(pts), 3), dtype=np.float32)
    for i, p in enumerate(pts):
        arr[i, 0] = p.x
        arr[i, 1] = p.y
        arr[i, 2] = p.z
    return arr


def collect_points(
    bag_dir: Path,
    topic: str,
    max_frames: int | None,
    traj: tuple[np.ndarray, np.ndarray, np.ndarray] | None,
) -> np.ndarray:
    """Iterate the bag, deserialize CustomMsg, return Nx3 float32 (concatenated).

    If `traj` (times, positions, quats_xyzw) is given, each lidar frame's stamp
    is interpolated into a 4x4 T_world_sensor and points are transformed to
    world frame before concatenation. Frames whose stamp falls outside the
    trajectory time range are dropped.
    """
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    _register_livox_types(typestore)

    chunks_xyz: list[np.ndarray] = []
    seen = 0
    used = 0
    skipped_oob = 0
    total_pts = 0
    times = pos = quat = None
    if traj is not None:
        times, pos, quat = traj
    with Reader(str(bag_dir)) as reader:
        lidar_conns = [c for c in reader.connections if c.topic == topic]
        if not lidar_conns:
            raise RuntimeError(
                f"topic {topic!r} not found; available: {[c.topic for c in reader.connections]}"
            )
        for conn, _bag_ts, raw in reader.messages(connections=lidar_conns):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            xyz = _frame_points(msg)
            if traj is not None:
                t_frame = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                T = _interp_pose(t_frame, times, pos, quat)
                if T is None:
                    skipped_oob += 1
                    seen += 1
                    continue
                xyz_h = np.concatenate(
                    [xyz, np.ones((xyz.shape[0], 1), dtype=np.float32)], axis=1
                )
                xyz = (xyz_h @ T.T.astype(np.float32))[:, :3]
            chunks_xyz.append(xyz)
            total_pts += xyz.shape[0]
            used += 1
            seen += 1
            if used % 100 == 0:
                log.info("  decoded %d frames  used=%d skipped=%d  %d pts",
                         seen, used, skipped_oob, total_pts)
            if max_frames is not None and seen >= max_frames:
                break

    log.info("collected %d frames seen  used=%d skipped_oob=%d  %d total points",
             seen, used, skipped_oob, total_pts)
    return np.concatenate(chunks_xyz, axis=0) if chunks_xyz else np.zeros((0, 3), dtype=np.float32)


def main() -> None:
    log_path = _setup_logging()
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--voxel-size", type=float, default=0.10)
    ap.add_argument("--chunk-extent", type=int, default=32)
    ap.add_argument("--compression", choices=["raw", "gzip", "zstd"], default="gzip")
    ap.add_argument("--swap-yz", action="store_true",
                    help="ROS Z-up → vxw Y-up (after pose application)")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="cap frames read (default: all)")
    ap.add_argument("--trajectory", type=Path, default=None,
                    help="TUM-format trajectory file: t tx ty tz qx qy qz qw, "
                         "world<-sensor (e.g. lightning_lm_foxy trajectory.txt)")
    ap.add_argument("--source-slam", default="lightning_lm_foxy_bag")
    args = ap.parse_args()

    log.info("session start  log=%s", log_path)
    log.info(
        "config  bag=%s  topic=%s  voxel=%.3fm  swap_yz=%s  max_frames=%s  traj=%s",
        args.bag_dir, args.topic, args.voxel_size, args.swap_yz,
        args.max_frames, args.trajectory,
    )

    if not (args.bag_dir / "metadata.yaml").is_file():
        log.error("not a rosbag2 dir (missing metadata.yaml): %s", args.bag_dir)
        sys.exit(1)

    traj = None
    if args.trajectory is not None:
        log.info("[1/4] loading trajectory %s", args.trajectory)
        times, pos, quat = _load_tum_trajectory(args.trajectory)
        log.info("      %d keyframes  t=[%.3f, %.3f]  bbox=[%.2f,%.2f,%.2f]..[%.2f,%.2f,%.2f]",
                 len(times), times[0], times[-1],
                 pos[:,0].min(), pos[:,1].min(), pos[:,2].min(),
                 pos[:,0].max(), pos[:,1].max(), pos[:,2].max())
        traj = (times, pos, quat)
    else:
        log.info("[1/4] no --trajectory: points accumulated in sensor frame (will show ghosting)")

    log.info("[2/4] reading bag")
    t0 = time.perf_counter()
    xyz = collect_points(args.bag_dir, args.topic, args.max_frames, traj)
    log.info("      %d points total in %.2fs", len(xyz), time.perf_counter() - t0)
    if len(xyz) == 0:
        log.error("no points collected; aborting")
        sys.exit(1)
    log.info(
        "      raw bbox  X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]",
        xyz[:, 0].min(), xyz[:, 0].max(),
        xyz[:, 1].min(), xyz[:, 1].max(),
        xyz[:, 2].min(), xyz[:, 2].max(),
    )

    if args.swap_yz:
        xyz = ros_zup_to_vxw_yup(xyz)

    log.info(
        "[3/4] voxelizing voxel=%.3fm  chunk_extent=%d  compression=%s",
        args.voxel_size, args.chunk_extent, args.compression,
    )
    t0 = time.perf_counter()
    chunks, bmin, bmax = voxelize_and_group(
        xyz, args.voxel_size, args.chunk_extent, _COMPRESSION_MAP[args.compression]
    )
    total_voxels = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in chunks.values()
    )
    log.info(
        "      %d chunks  %d occupied voxels  in %.2fs",
        len(chunks), total_voxels, time.perf_counter() - t0,
    )

    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=args.voxel_size,
        chunk_extent=args.chunk_extent,
        bounds_chunks_min=bmin,
        bounds_chunks_max=bmax,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system=args.source_slam,
        source_sensor="livox",
        raw_data_hash=f"sha-skip:{args.bag_dir.name}",
    )
    world = vxw.World(manifest=manifest, palette=build_palette(), chunks=chunks)

    log.info("[4/4] writing %s", args.output_vxw)
    t0 = time.perf_counter()
    vxw.write_world(args.output_vxw, world)
    log.info("      wrote in %.2fs", time.perf_counter() - t0)

    loaded = vxw.read_world(args.output_vxw)
    loaded_occ = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in loaded.chunks.values()
    )
    if loaded_occ != total_voxels:
        log.error("round-trip lost voxels! expected=%d got=%d",
                  total_voxels, loaded_occ)
        sys.exit(2)
    total_size = sum(
        p.stat().st_size for p in args.output_vxw.rglob("*") if p.is_file()
    )
    log.info(
        "RESULT  frames~=%s  total_pts=%d  voxels=%d  chunks=%d  vxw_bytes=%d",
        args.max_frames or "all", len(xyz), total_voxels, len(chunks), total_size,
    )
    log.info("DONE    written=%s  log=%s", args.output_vxw, log_path)


if __name__ == "__main__":
    main()
