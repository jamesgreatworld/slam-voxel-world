"""[DEPRECATED for new use] Batch uHumans2 -> .vxw adapter. The ObsMap +
uhumans2_stream + obsmap_to_vxw path is now the single source of truth
(occupancy/free/semantic in one persistent incremental map). This module is
kept for its reusable seg/entity/spawn helpers (imported by obsmap_export) and
as the reference batch implementation. See docs SP-B + the unification spec.

uhumans2_to_vxw — convert a uHumans2 / TESSE rosbag2 directly into .vxw.

Pipeline (M2a, single-layer voxel demo; entity extraction comes in M2b):

  1. Scan /tesse/odom (200 Hz) → list of (t, T_world_body).
  2. For each RGB-D frame (depth_cam + seg_cam, same timestamp, ~13 Hz):
       a. Interpolate the body pose at that timestamp.
       b. Unproject depth (32FC1, metres) with camera_info K to camera-frame points.
       c. Look up each seg pixel's RGB in the Hydra apartment CSV → super_id.
       d. Transform points to world via T_world_cam.
       e. Voxelise this frame and accumulate (vc, label) tuples.
  3. Per-voxel majority-vote on accumulated labels → material_id == super_id.
  4. Build 21-material palette from the label_space yaml, write .vxw.

Inputs (all auto-located):
  --bag       rosbag2 dir (must contain metadata.yaml)
  --hydra-cfg root of an installed Hydra workspace; we use
                  install/hydra/share/hydra/config/label_spaces/uhumans2_apartment_label_space.yaml
                  install/hydra_ros/share/hydra_ros/config/color/uhumans2_apartment.csv
              (or src/Hydra-ROS/hydra_ros/config/color/uhumans2_apartment.csv)

Pose handling: this bag is fully-supervised — /tesse/odom carries world←body GT.
We pull body→cam from /tf_static once (T_body_cam) and compose.

Run:
  pixi run python m3_adapter/uhumans2_to_vxw.py \\
    F:/hydra_ws/datasets/uhumans2/apartment_scene/uHumans2_apartment_s1_00h_ros2 \\
    out/uhumans2_apt.vxw \\
    --hydra-cfg E:/aros_slam_ws/hydra_ws \\
    --voxel-size 0.05 --max-frames 200
"""

from __future__ import annotations

import argparse
import csv
import logging
import sys
import time
import uuid
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.common import ros_zup_to_vxw_yup  # noqa: E402
from m3_adapter.voxel_postprocess import close_holes, denoise_by_count  # noqa: E402


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_LOG_DIR = _PROJECT_ROOT / "out" / "logs"
log = logging.getLogger("uhumans2_to_vxw")

_COMPRESSION_MAP = {
    "raw": vxw.Compression.RAW,
    "gzip": vxw.Compression.GZIP,
    "zstd": vxw.Compression.ZSTD,
}

# Hard-coded palette colours per super_id — picked for visual clarity in Godot.
# (Spec ties material_id to super_id; CSV-derived per-asset colours would
# blow the 256 cap, so we use one colour per semantic class.)
# Hydra apartment label-space classifies these super_ids as "objects" (the
# rest — wall/floor/ceiling/stairs — are map structure). M2b lifts each
# connected component of these labels out of the voxel grid into an entity.
_OBJECT_LABELS: frozenset[int] = frozenset({5, 7, 10, 11, 14, 16, 18})

_SUPER_COLOURS: dict[int, tuple[int, int, int]] = {
    0: (90, 90, 90),     # unknown
    1: (200, 130, 60),   # appliance
    2: (210, 180, 80),   # books
    3: (160, 140, 110),  # floor
    4: (220, 220, 220),  # ceiling
    5: (220, 50, 50),    # chair
    6: (180, 100, 200),  # vase
    7: (240, 130, 40),   # couch
    8: (60, 180, 80),    # plant (8)
    9: (140, 80, 50),    # furniture
    10: (30, 180, 220),  # computer
    11: (250, 220, 60),  # lamp
    12: (180, 60, 110),  # painting
    13: (40, 160, 60),   # plant (13 — same family)
    14: (60, 90, 220),   # bed
    15: (130, 130, 200), # stairs
    16: (130, 80, 30),   # table
    17: (40, 200, 230),  # screens
    18: (90, 110, 60),   # trashcan
    19: (210, 200, 180), # wall
    20: (240, 0, 120),   # human (dynamic)
}


def _setup_logging() -> Path:
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_path = _LOG_DIR / f"uhumans2_{ts}.log"
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


# ----------------------------------------------------------------------------
# Hydra config: label space + colour CSV
# ----------------------------------------------------------------------------

def _resolve_hydra_paths(hydra_cfg_root: Path, scene: str = "apartment") -> tuple[Path, Path]:
    """Return (label_space_yaml, color_csv) under either install/ or src/ layout."""
    # src/ first — Hydra's install/ copies are symlinks to Linux paths and
    # appear is_file()-OK but fail to read on Windows.
    candidates_yaml = [
        hydra_cfg_root / f"src/Hydra/config/label_spaces/uhumans2_{scene}_label_space.yaml",
        hydra_cfg_root / f"install/hydra/share/hydra/config/label_spaces/uhumans2_{scene}_label_space.yaml",
    ]
    candidates_csv = [
        hydra_cfg_root / f"src/Hydra-ROS/hydra_ros/config/color/uhumans2_{scene}.csv",
        hydra_cfg_root / f"install/hydra_ros/share/hydra_ros/config/color/uhumans2_{scene}.csv",
    ]
    def _readable(p: Path) -> bool:
        try:
            with p.open("rb") as f:
                f.read(1)
            return True
        except OSError:
            return False
    yaml_path = next((p for p in candidates_yaml if _readable(p)), None)
    csv_path = next((p for p in candidates_csv if _readable(p)), None)
    if yaml_path is None or csv_path is None:
        raise FileNotFoundError(
            f"could not find uhumans2 config under {hydra_cfg_root}\n"
            f"  yaml tried: {candidates_yaml}\n"
            f"  csv tried : {candidates_csv}"
        )
    return yaml_path, csv_path


def load_label_space(yaml_path: Path) -> dict[int, str]:
    with yaml_path.open() as f:
        spec = yaml.safe_load(f)
    return {item["label"]: item["name"] for item in spec.get("label_names", [])}


def load_color_to_super_id(csv_path: Path) -> dict[tuple[int, int, int], int]:
    """CSV row: name, red, green, blue, alpha, id → {(r,g,b): super_id}."""
    mapping: dict[tuple[int, int, int], int] = {}
    with csv_path.open(encoding="utf-8-sig") as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            r, g, b = int(row["red"]), int(row["green"]), int(row["blue"])
            mapping[(r, g, b)] = int(row["id"])
    return mapping


def build_palette(label_names: dict[int, str]) -> vxw.Palette:
    materials = []
    semclasses = []
    color_lut = []
    # Ensure id=0 (air) exists per spec — overwrite "unknown" name to "air".
    for super_id in sorted(set(label_names) | set(_SUPER_COLOURS)):
        name = label_names.get(super_id, f"id{super_id}")
        rgb = _SUPER_COLOURS.get(super_id, (128, 128, 128))
        if super_id == 0:
            name = "air"
            flags: tuple[str, ...] = ("empty",)
        elif super_id in (5, 7, 10, 11, 14, 16, 18):  # object_labels
            flags = ("solid", "destructible", "object")
        elif super_id == 20:  # human
            flags = ("solid", "dynamic")
        else:
            flags = ("solid", "destructible")
        materials.append(vxw.Material(
            id=super_id, name=name, color_rgb=rgb, flags=flags,
        ))
        semclasses.append(vxw.SemanticClass(
            id=super_id, name=name, default_material=super_id,
        ))
        color_lut.append(rgb)
    return vxw.Palette(
        materials=materials, semantic_classes=semclasses, color_lut=color_lut,
    )


# ----------------------------------------------------------------------------
# Pose handling — odom + tf_static
# ----------------------------------------------------------------------------

def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = qw * qw + qx * qx + qy * qy + qz * qz
    if n > 0 and abs(n - 1.0) > 1e-6:
        s = 1.0 / np.sqrt(n)
        qw, qx, qy, qz = qw * s, qx * s, qy * s, qz * s
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    if np.dot(q0, q1) < 0:
        q1 = -q1
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot > 0.9995:
        out = q0 + u * (q1 - q0)
        return out / np.linalg.norm(out)
    th = np.arccos(dot)
    s = np.sin(th)
    return (np.sin((1 - u) * th) / s) * q0 + (np.sin(u * th) / s) * q1


def collect_odom(reader, typestore) -> tuple[np.ndarray, np.ndarray, np.ndarray, str, str]:
    """Walk /tesse/odom, return (times_s, positions Nx3, quats_xyzw Nx4,
    frame_id, child_frame_id). World ← child_frame at each timestamp."""
    times, pos, quat = [], [], []
    fid = cid = ""
    conn = [c for c in reader.connections if c.topic == "/tesse/odom"][0]
    for _c, _t, raw in reader.messages(connections=[conn]):
        m = typestore.deserialize_cdr(raw, conn.msgtype)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        times.append(t)
        pos.append([p.x, p.y, p.z])
        quat.append([q.x, q.y, q.z, q.w])
        if not fid:
            fid = m.header.frame_id
            cid = m.child_frame_id
    times = np.asarray(times, dtype=np.float64)
    pos = np.asarray(pos, dtype=np.float64)
    quat = np.asarray(quat, dtype=np.float64)
    nq = np.linalg.norm(quat, axis=1, keepdims=True)
    nq[nq == 0] = 1.0
    quat = quat / nq
    o = np.argsort(times)
    return times[o], pos[o], quat[o], fid, cid


def collect_tf_static(reader, typestore) -> dict[tuple[str, str], np.ndarray]:
    """Walk /tf_static and return {(parent, child): 4x4 T_parent_child}."""
    out: dict[tuple[str, str], np.ndarray] = {}
    conns = [c for c in reader.connections if c.topic == "/tf_static"]
    if not conns:
        return out
    for c in conns:
        for _c, _t, raw in reader.messages(connections=[c]):
            m = typestore.deserialize_cdr(raw, c.msgtype)
            for tf in m.transforms:
                p, ch = tf.header.frame_id, tf.child_frame_id
                t = tf.transform.translation
                q = tf.transform.rotation
                T = np.eye(4)
                T[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
                T[:3, 3] = (t.x, t.y, t.z)
                out[(p, ch)] = T
    return out


def _interp_T_world_body(t_q: float, times, pos, quat) -> np.ndarray | None:
    if t_q < times[0] or t_q > times[-1]:
        return None
    j = int(np.searchsorted(times, t_q, side="right"))
    if j == 0:
        i, u = 0, 0.0
    elif j >= len(times):
        i, u = len(times) - 2, 1.0
    else:
        i = j - 1
        dt = times[j] - times[i]
        u = 0.0 if dt <= 0 else (t_q - times[i]) / dt
    p = (1 - u) * pos[i] + u * pos[j if j < len(times) else i + 1]
    q = _slerp(quat[i], quat[j if j < len(times) else i + 1], u)
    T = np.eye(4)
    T[:3, :3] = _quat_to_matrix(q[0], q[1], q[2], q[3])
    T[:3, 3] = p
    return T


def _chain_tf(tf_static: dict, parent: str, child: str) -> np.ndarray | None:
    """Compose T_parent_child by walking tf_static graph (limited depth)."""
    if parent == child:
        return np.eye(4)
    if (parent, child) in tf_static:
        return tf_static[(parent, child)]
    # try inverse
    if (child, parent) in tf_static:
        T = tf_static[(child, parent)]
        return np.linalg.inv(T)
    # BFS up to depth 4
    seen = {parent}
    queue = [(parent, np.eye(4))]
    for _ in range(4):
        nxt = []
        for cur, T in queue:
            for (p2, c2), T2 in tf_static.items():
                if p2 == cur and c2 not in seen:
                    new_T = T @ T2
                    if c2 == child:
                        return new_T
                    seen.add(c2)
                    nxt.append((c2, new_T))
                if c2 == cur and p2 not in seen:
                    new_T = T @ np.linalg.inv(T2)
                    if p2 == child:
                        return new_T
                    seen.add(p2)
                    nxt.append((p2, new_T))
        queue = nxt
    return None


# ----------------------------------------------------------------------------
# RGB-D unprojection + seg lookup
# ----------------------------------------------------------------------------

def unproject_depth(depth: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                    z_min: float = 0.1, z_max: float = 20.0
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Return (Nx3 camera-frame xyz, valid_mask H×W bool)."""
    H, W = depth.shape
    valid = (depth > z_min) & (depth < z_max) & np.isfinite(depth)
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    Z = depth[valid].astype(np.float32)
    X = (us[valid] - cx) * Z / fx
    Y = (vs[valid] - cy) * Z / fy
    xyz = np.stack([X, Y, Z], axis=1)
    return xyz, valid


def seg_pixels_to_labels(seg_rgb: np.ndarray,
                          color_to_id: dict[tuple[int, int, int], int],
                          valid: np.ndarray) -> np.ndarray:
    """Vectorised RGB → super_id via numpy.searchsorted over packed uint32 keys."""
    rgb = seg_rgb[valid]  # (N,3) uint8
    packed = (rgb[:, 0].astype(np.uint32) << 16) | \
             (rgb[:, 1].astype(np.uint32) << 8) | \
              rgb[:, 2].astype(np.uint32)
    keys = np.fromiter(
        ((r << 16) | (g << 8) | b for (r, g, b) in color_to_id),
        dtype=np.uint32,
    )
    vals = np.fromiter(color_to_id.values(), dtype=np.uint8)
    order = np.argsort(keys)
    keys_s = keys[order]
    vals_s = vals[order]
    idx = np.searchsorted(keys_s, packed)
    # mark unknown when not found (idx out of range or key mismatch)
    out = np.zeros(packed.shape, dtype=np.uint8)
    hit = (idx < len(keys_s)) & (keys_s[np.clip(idx, 0, len(keys_s) - 1)] == packed)
    out[hit] = vals_s[idx[hit]]
    return out


# ----------------------------------------------------------------------------
# Entity extraction (M2b)
# ----------------------------------------------------------------------------

def _mat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix → (qx, qy, qz, qw). Shepperd's method, branchless."""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m02 - m20) / s    # unused but kept for symmetry
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return float(qx / n), float(qy / n), float(qz / n), float(qw / n)


def extract_entities(
    final_vc: np.ndarray,
    final_lbl: np.ndarray,
    voxel_size: float,
    label_names: dict[int, str],
    dbscan_eps_voxels: float = 2.0,
    min_samples: int = 10,
) -> tuple[list, np.ndarray]:
    """For each object label, DBSCAN-cluster its voxels, fit PCA OBB per
    cluster, and emit one Entity per cluster. Returns (entities, keep_mask)
    where keep_mask[i] is False for voxels that became part of an entity."""
    from sklearn.cluster import DBSCAN

    entities: list = []
    keep_mask = np.ones(len(final_vc), dtype=bool)
    for lbl in sorted(_OBJECT_LABELS):
        mask = (final_lbl == lbl)
        if mask.sum() < min_samples:
            continue
        sub_idx = np.flatnonzero(mask)
        vc_lbl = final_vc[mask].astype(np.float64)
        clusters = DBSCAN(
            eps=dbscan_eps_voxels, min_samples=min_samples, n_jobs=-1,
        ).fit_predict(vc_lbl)
        n_clusters = int(clusters.max()) + 1 if clusters.max() >= 0 else 0
        log.info("    label %2d %-12s: %d voxels → %d clusters",
                 int(lbl), label_names.get(int(lbl), "?"),
                 int(mask.sum()), n_clusters)
        for cid in range(n_clusters):
            in_c = (clusters == cid)
            cluster_global = sub_idx[in_c]
            vc_c = vc_lbl[in_c]
            xyz = (vc_c + 0.5) * voxel_size  # voxel centres in world metres
            centroid = xyz.mean(axis=0)
            centered = xyz - centroid
            if len(centered) < 3:
                continue
            cov = (centered.T @ centered) / len(centered)
            eigvals, eigvecs = np.linalg.eigh(cov)
            # eigh returns columns sorted ascending; reverse so axis 0 is the
            # major axis (more intuitive for an "object pointing forward").
            eigvecs = eigvecs[:, ::-1]
            if np.linalg.det(eigvecs) < 0:
                eigvecs[:, -1] *= -1
            local = centered @ eigvecs
            mn = local.min(axis=0)
            mx = local.max(axis=0)
            dims = mx - mn
            center = centroid + eigvecs @ ((mn + mx) / 2)
            qx, qy, qz, qw = _mat_to_quat(eigvecs)
            entities.append(vxw.Entity(
                id=str(uuid.uuid4()),
                label=int(lbl),
                label_name=label_names.get(int(lbl), "?"),
                position=tuple(float(c) for c in center),
                rotation=(qx, qy, qz, qw),
                bbox_dims=tuple(float(c) for c in dims),
                voxel_count=int(len(vc_c)),
            ))
            keep_mask[cluster_global] = False
    return entities, keep_mask


def _find_spawn_hint(final_vc, final_lbl, voxel_size, floor_label: int = 3
                     ) -> list | None:
    """Pick a spawn point on top of an open floor cell with at least 2.5 m
    of head-clearance air above it. Returns [x, y, z, yaw_deg] in metres,
    or None if no suitable floor was found.

    Strategy:
      1. Filter voxels labeled `floor` (super_id=3 in the uhumans2 label space).
      2. For each floor voxel, count vertical air clearance above it (no occupied
         voxel up to `headroom` cells). Skip if clearance < headroom.
      3. Among candidates, pick the one closest to the (x, z) centroid of all
         floor voxels — keeps the spawn away from edges / hallway ends and
         roughly in the middle of the largest room.
      4. yaw faces the centroid (so the rig walks into the apartment, not at a
         wall).
    """
    if final_vc.size == 0:
        return None
    floor_mask = final_lbl == floor_label
    if not floor_mask.any():
        return None
    floor_vc = final_vc[floor_mask]
    # Occupancy lookup as a set of tuples — fine at hundreds of thousands.
    occupied = {tuple(v) for v in final_vc.tolist()}
    headroom_cells = max(2, int(round(2.5 / voxel_size)))

    candidates = []
    for v in floor_vc.tolist():
        clear = True
        for h in range(1, headroom_cells + 1):
            if (v[0], v[1] + h, v[2]) in occupied:
                clear = False
                break
        if clear:
            candidates.append(v)
    if not candidates:
        return None

    import numpy as np
    floor_xz = floor_vc[:, [0, 2]].astype(np.float64)
    cx, cz = floor_xz.mean(axis=0)
    cand_np = np.asarray(candidates, dtype=np.float64)
    dx = cand_np[:, 0] - cx
    dz = cand_np[:, 2] - cz
    dist2 = dx * dx + dz * dz
    best = candidates[int(dist2.argmin())]
    # Spawn 1.0 m above the floor cell top; floor cell top sits at (y+1)*size.
    spawn_x = (best[0] + 0.5) * voxel_size
    spawn_y = (best[1] + 1) * voxel_size + 1.0
    spawn_z = (best[2] + 0.5) * voxel_size
    # Face roughly the centroid so we look into the room.
    import math
    dx_to_centroid = cx * voxel_size - spawn_x
    dz_to_centroid = cz * voxel_size - spawn_z
    yaw_deg = math.degrees(math.atan2(dx_to_centroid, -dz_to_centroid))
    return [round(spawn_x, 3), round(spawn_y, 3),
            round(spawn_z, 3), round(yaw_deg, 1)]


# ----------------------------------------------------------------------------
# Main pipeline
# ----------------------------------------------------------------------------

def main() -> None:
    log_path = _setup_logging()
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("output_vxw", type=Path)
    ap.add_argument("--hydra-cfg", type=Path,
                    default=Path("E:/aros_slam_ws/hydra_ws"),
                    help="Hydra workspace root (uses install/ or src/ uhumans2 config)")
    ap.add_argument("--scene", choices=["apartment", "office", "subway"],
                    default="apartment",
                    help="uHumans2 scene name; selects matching label_space yaml + colour CSV")
    ap.add_argument("--voxel-size", type=float, default=0.05)
    ap.add_argument("--chunk-extent", type=int, default=32)
    ap.add_argument("--compression", choices=["raw", "gzip", "zstd"], default="gzip")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--depth-min", type=float, default=0.2)
    ap.add_argument("--depth-max", type=float, default=15.0)
    ap.add_argument("--pixel-stride", type=int, default=1,
                    help="downsample depth pixels by this factor (1=full)")
    ap.add_argument("--no-swap-yz", action="store_true",
                    help="skip the ROS Z-up → vxw Y-up swap (uHumans2/TESSE odom "
                         "publishes Z-up; without the swap the world ends up "
                         "lying on its side and the player floats in mid-air)")
    ap.add_argument("--min-observations", type=int, default=3,
                    help="drop voxels seen in fewer than N frames "
                         "(noise filter; 1 = off, 3 = recommended for uhumans2)")
    ap.add_argument("--close-iters", type=int, default=1,
                    help="binary closing iterations on the voxel grid "
                         "(fills 1-cell holes in walls / surfaces; 0 = off)")
    ap.add_argument("--extract-entities", action="store_true",
                    help="DBSCAN-cluster object-label voxels into entities + remove "
                         "their voxels from the grid (writes entities.json)")
    ap.add_argument("--entity-eps", type=float, default=2.0,
                    help="DBSCAN eps in voxel units (default 2 → 8-conn-ish)")
    ap.add_argument("--entity-min-samples", type=int, default=10,
                    help="DBSCAN min_samples — clusters smaller than this dropped")
    args = ap.parse_args()

    log.info("session start  log=%s", log_path)
    log.info("config  bag=%s  voxel=%.3fm  max_frames=%s  stride=%d  depth=[%.1f,%.1f]",
             args.bag_dir, args.voxel_size, args.max_frames,
             args.pixel_stride, args.depth_min, args.depth_max)

    if not (args.bag_dir / "metadata.yaml").is_file():
        log.error("not a rosbag2 dir: %s", args.bag_dir); sys.exit(1)

    # ---- config ----
    yaml_path, csv_path = _resolve_hydra_paths(args.hydra_cfg, args.scene)
    log.info("[1/5] loading Hydra cfg yaml=%s  csv=%s", yaml_path.name, csv_path.name)
    label_names = load_label_space(yaml_path)
    color_to_id = load_color_to_super_id(csv_path)
    log.info("      %d label names, %d colour rows", len(label_names), len(color_to_id))

    # ---- bag scan ----
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)

    log.info("[2/5] scanning bag for odom + tf_static")
    t0 = time.perf_counter()
    with Reader(str(args.bag_dir)) as reader:
        odom_t, odom_p, odom_q, world_frame, body_frame = collect_odom(reader, ts)
        tf_static = collect_tf_static(reader, ts)
        # depth camera_info — fetch first
        dci_conn = [c for c in reader.connections if c.topic == "/tesse/depth_cam/camera_info"][0]
        for _, _, raw in reader.messages(connections=[dci_conn]):
            dci = ts.deserialize_cdr(raw, dci_conn.msgtype)
            fx, fy = float(dci.k[0]), float(dci.k[4])
            cx, cy = float(dci.k[2]), float(dci.k[5])
            cam_frame = dci.header.frame_id
            break
    log.info("      odom %d kfr  t=[%.3f,%.3f]  world=%s  body=%s",
             len(odom_t), odom_t[0], odom_t[-1], world_frame, body_frame)
    log.info("      tf_static entries: %d  cam_frame=%s",
             len(tf_static), cam_frame)
    log.info("      depth K  fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx, fy, cx, cy)

    T_body_cam = _chain_tf(tf_static, body_frame, cam_frame)
    if T_body_cam is None:
        log.warning("could not resolve %s → %s via tf_static; using identity",
                    body_frame, cam_frame)
        T_body_cam = np.eye(4)
    log.info("      T_body_cam translation=%s", T_body_cam[:3, 3].tolist())
    log.info("      bag scan done in %.2fs", time.perf_counter() - t0)

    # ---- frame loop ----
    log.info("[3/5] iterating depth+seg frames")
    t0 = time.perf_counter()
    # accumulator: list of small ndarrays Nx4 (vx, vy, vz, label) per frame
    accum: list[np.ndarray] = []
    frames_used = 0
    frames_skipped = 0
    total_pts = 0
    with Reader(str(args.bag_dir)) as reader:
        dconn = [c for c in reader.connections if c.topic == "/tesse/depth_cam/mono/image_raw"][0]
        sconn = [c for c in reader.connections if c.topic == "/tesse/seg_cam/rgb/image_raw"][0]

        depth_iter = reader.messages(connections=[dconn])
        seg_iter = reader.messages(connections=[sconn])

        while True:
            try:
                _dc, _dt, draw = next(depth_iter)
                _sc, _st, sraw = next(seg_iter)
            except StopIteration:
                break
            dm = ts.deserialize_cdr(draw, dconn.msgtype)
            sm = ts.deserialize_cdr(sraw, sconn.msgtype)
            if dm.encoding != "32FC1" or sm.encoding != "rgb8":
                log.error("unexpected encoding depth=%s seg=%s", dm.encoding, sm.encoding)
                sys.exit(2)
            depth = np.frombuffer(dm.data, dtype=np.float32).reshape(dm.height, dm.width)
            seg = np.frombuffer(sm.data, dtype=np.uint8).reshape(sm.height, sm.width, 3)
            if args.pixel_stride > 1:
                depth = depth[::args.pixel_stride, ::args.pixel_stride]
                seg = seg[::args.pixel_stride, ::args.pixel_stride]
            t_frame = dm.header.stamp.sec + dm.header.stamp.nanosec * 1e-9

            T_world_body = _interp_T_world_body(t_frame, odom_t, odom_p, odom_q)
            if T_world_body is None:
                frames_skipped += 1
                continue
            T_world_cam = T_world_body @ T_body_cam

            H, W = depth.shape
            xyz_cam, valid = unproject_depth(
                depth,
                fx / args.pixel_stride, fy / args.pixel_stride,
                cx / args.pixel_stride, cy / args.pixel_stride,
                z_min=args.depth_min, z_max=args.depth_max,
            )
            if xyz_cam.shape[0] == 0:
                frames_skipped += 1
                continue
            labels = seg_pixels_to_labels(seg, color_to_id, valid)

            xyz_h = np.concatenate(
                [xyz_cam, np.ones((xyz_cam.shape[0], 1), dtype=np.float32)], axis=1
            )
            xyz_world = (xyz_h @ T_world_cam.T.astype(np.float32))[:, :3]
            if not args.no_swap_yz:
                xyz_world = ros_zup_to_vxw_yup(xyz_world)
            vc = np.floor(xyz_world / args.voxel_size).astype(np.int32)
            stacked = np.concatenate([vc, labels[:, None].astype(np.int32)], axis=1)
            stacked = np.unique(stacked, axis=0)
            accum.append(stacked)
            frames_used += 1
            total_pts += xyz_cam.shape[0]
            if frames_used % 50 == 0:
                log.info("  used=%d skipped=%d  acc_rows=%d  pts=%d",
                         frames_used, frames_skipped,
                         sum(a.shape[0] for a in accum), total_pts)
            if args.max_frames is not None and frames_used >= args.max_frames:
                break

    log.info("      frames used=%d skipped=%d  raw_pts=%d  in %.2fs",
             frames_used, frames_skipped, total_pts, time.perf_counter() - t0)
    if not accum:
        log.error("no usable frames"); sys.exit(2)

    # ---- majority vote per voxel ----
    log.info("[4/5] majority vote per voxel")
    t0 = time.perf_counter()
    all_rows = np.concatenate(accum, axis=0)
    # count (vc, label) occurrences
    keys, counts = np.unique(all_rows, axis=0, return_counts=True)
    # for each unique vc, pick the label with max count
    vc_only = keys[:, :3]
    labels = keys[:, 3]
    # sort by vc lex so equal vc rows are adjacent
    order = np.lexsort((labels, vc_only[:, 2], vc_only[:, 1], vc_only[:, 0]))
    keys = keys[order]
    counts = counts[order]
    vc_only = vc_only[order]
    labels = labels[order]
    # group by vc: detect run starts
    diff = np.any(np.diff(vc_only, axis=0) != 0, axis=1)
    starts = np.concatenate([[0], np.where(diff)[0] + 1])
    ends = np.concatenate([starts[1:], [len(vc_only)]])
    final_vc = np.empty((len(starts), 3), dtype=np.int32)
    final_lbl = np.empty(len(starts), dtype=np.uint8)
    final_total = np.zeros(len(starts), dtype=np.int64)
    for k, (s, e) in enumerate(zip(starts, ends)):
        run_counts = counts[s:e]
        run_labels = labels[s:e]
        winner = run_labels[int(np.argmax(run_counts))]
        final_vc[k] = vc_only[s]
        final_lbl[k] = winner
        final_total[k] = int(run_counts.sum())
    # Drop unknown (super_id=0); vxw treats material_id=0 as air, so any
    # unknown voxel would be invisible anyway and would break round-trip count.
    keep = final_lbl != 0
    dropped_unknown = int((~keep).sum())
    final_vc = final_vc[keep]
    final_lbl = final_lbl[keep]
    final_total = final_total[keep]
    # --- quality passes delegated to m3_adapter/voxel_postprocess so every
    # adapter shares a single implementation. Hydra adapter keeps both off
    # by default to mirror raw Hydra; this one defaults to mild cleanup.
    final_vc, final_lbl, dropped_noise = denoise_by_count(
        final_vc, final_lbl, final_total, args.min_observations,
    )
    if dropped_noise > 0:
        log.info("      noise filter (>=%d obs) dropped %d voxels",
                 args.min_observations, dropped_noise)
    log.info("      %d unique voxels (dropped %d unknown) in %.2fs",
             len(final_vc), dropped_unknown, time.perf_counter() - t0)
    if args.close_iters > 0 and len(final_vc) > 0:
        t1 = time.perf_counter()
        final_vc, final_lbl, filled = close_holes(
            final_vc, final_lbl, args.close_iters,
        )
        log.info("      morphological close (%d iter) filled %d voxels in %.2fs",
                 args.close_iters, filled, time.perf_counter() - t1)
    # label histogram
    uniq_l, cnt_l = np.unique(final_lbl, return_counts=True)
    log.info("      label histogram:")
    for l, c in sorted(zip(uniq_l, cnt_l), key=lambda x: -x[1])[:21]:
        log.info("        id=%2d  %-12s  %d", int(l),
                 label_names.get(int(l), "?"), int(c))

    # ---- entity extraction ----
    entities: list = []
    if args.extract_entities:
        log.info("[4.5/5] extracting entities from object_labels (eps=%.1f vox, min=%d)",
                 args.entity_eps, args.entity_min_samples)
        t0 = time.perf_counter()
        entities, keep = extract_entities(
            final_vc, final_lbl, args.voxel_size, label_names,
            dbscan_eps_voxels=args.entity_eps,
            min_samples=args.entity_min_samples,
        )
        n_removed = int((~keep).sum())
        final_vc = final_vc[keep]
        final_lbl = final_lbl[keep]
        log.info("      %d entities  removed %d voxels from grid  in %.2fs",
                 len(entities), n_removed, time.perf_counter() - t0)

    # ---- chunkise + write ----
    log.info("[5/5] chunkising and writing")
    chunk_extent = args.chunk_extent
    cc = np.floor_divide(final_vc, chunk_extent)
    local = (final_vc - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)
    compression = _COMPRESSION_MAP[args.compression]
    chunks = {}
    for ki in range(len(chunk_keys)):
        mask = inverse == ki
        locs = local[mask]
        lbls = final_lbl[mask]
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = lbls
        arr["semantic_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = lbls
        ck = tuple(int(x) for x in chunk_keys[ki])
        chunks[ck] = vxw.Chunk(
            coord=ck, voxels=arr,
            encoding=vxw.Encoding.RLE, compression=compression,
        )
    bmin = tuple(int(x) for x in cc.min(axis=0))
    bmax = tuple(int(x) + 1 for x in cc.max(axis=0))
    spawn_hint = _find_spawn_hint(final_vc, final_lbl, args.voxel_size)
    if spawn_hint is not None:
        log.info("spawn_hint: %s", spawn_hint)
    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=args.voxel_size,
        chunk_extent=chunk_extent,
        bounds_chunks_min=bmin,
        bounds_chunks_max=bmax,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system=f"uhumans2_tesse_{args.scene}",
        source_sensor="depth+seg",
        raw_data_hash=f"sha-skip:{args.bag_dir.name}",
        spawn_hint=spawn_hint,
    )
    palette = build_palette(label_names)
    world = vxw.World(manifest=manifest, palette=palette,
                      chunks=chunks, entities=entities)
    log.info("      %d chunks  %d voxels  bounds=%s..%s",
             len(chunks), len(final_vc), bmin, bmax)
    t0 = time.perf_counter()
    vxw.write_world(args.output_vxw, world)
    log.info("      wrote in %.2fs", time.perf_counter() - t0)

    loaded = vxw.read_world(args.output_vxw)
    loaded_occ = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in loaded.chunks.values()
    )
    if loaded_occ != len(final_vc):
        log.error("round-trip mismatch: expected=%d got=%d", len(final_vc), loaded_occ)
        sys.exit(2)
    total_size = sum(
        p.stat().st_size for p in args.output_vxw.rglob("*") if p.is_file()
    )
    log.info("RESULT  frames=%d  voxels=%d  chunks=%d  entities=%d  vxw_bytes=%d  log=%s",
             frames_used, len(final_vc), len(chunks), len(entities), total_size, log_path)


if __name__ == "__main__":
    main()
