"""uhumans2_stream.py — resume-capable streaming ObsMap driver for uHumans2.

Integrates rosbag depth frames into a persistent ObsMap. Supports incremental
continue/resume: run once, save map, run again with --start-frame to pick up
where you left off. Each run appends to the same obsmap.npz.

Usage:
  pixi run python m3_adapter/uhumans2_stream.py \\
    F:/hydra_ws/datasets/uhumans2/apartment_scene/uHumans2_apartment_s1_00h_ros2 \\
    out/uhumans2_apt.vxw \\
    --start-frame 0 --end-frame 500 --pixel-stride 4

Output:
  <vxw_dir>/obsmap.npz          — persistent log-odds map (save/resume)
  <vxw_dir>/observed_free.npz   — observed-free mask (same format as uhumans2_carve)
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import vxw_format as vxw  # noqa: E402
from m3_adapter.common import ros_zup_to_vxw_yup  # noqa: E402
from m3_adapter.gvd.field import (  # noqa: E402
    densify_occupancy, extract_gvd_local, replace_region, thin_gvd,
    compute_esdf, extract_gvd)
from m3_adapter.obsmap import ObsMap  # noqa: E402
from m3_adapter.gvd.pipeline import GvdConfig, run as gvd_run  # noqa: E402
from m3_adapter.obsmap_export import occupancy_to_vxw  # noqa: E402
from m3_adapter.uhumans2_to_vxw import (  # noqa: E402
    collect_odom,
    collect_tf_static,
    unproject_depth,
    _interp_T_world_body,
    _chain_tf,
    _resolve_hydra_paths,
    load_label_space,
    load_color_to_super_id,
    build_palette,
    seg_pixels_to_labels,
)

log = logging.getLogger("uhumans2_stream")


def _live_snapshot(obs, batch_idx, frames_so_far, out_vxw: Path, vxw_dir: Path,
                   sem_palette=None) -> None:
    """Write a renderable GVD .vxw from the current ObsMap state.

    Grid alignment: occupancy_to_vxw covers only the occupied subset of the obsmap
    grid. densify_occupancy(pad=1) inside the pipeline re-derives its own vmin/shape
    from that tight bbox. We resample observed_free onto that derived grid so shapes
    and vmins align exactly when the pipeline calls load_observed_free.

    When sem_palette is provided the temp geometric .vxw is semantic-coloured.
    """
    temp_geom = vxw_dir / "_live_geom.vxw"

    occ_mask = obs.occupancy_mask()
    occ_count = int(occ_mask.sum())
    free_count = int(obs.observed_free_mask().sum())

    if occ_count == 0:
        log.info("  [live] batch=%d  fi=%d  occ=0 — skipping (no occupied voxels yet)",
                 batch_idx, frames_so_far)
        return

    # 1. Write temp geometric .vxw (semantic-coloured if palette provided)
    if sem_palette is not None:
        occupancy_to_vxw(occ_mask, obs.vmin, obs.voxel_size, temp_geom,
                         semantic_grid=obs.semantic_grid(), palette=sem_palette)
    else:
        occupancy_to_vxw(occ_mask, obs.vmin, obs.voxel_size, temp_geom)

    # 2. Derive densify grid that pipeline will use (pad=1 matches GvdConfig default)
    world2 = vxw.read_world(temp_geom)
    occ2, vmin2 = densify_occupancy(world2, pad=1)

    # 3. Resample obsmap observed_free onto the densify grid
    of_world = np.argwhere(obs.observed_free_mask()).astype(np.int64) + obs.vmin  # (N,3) world-voxel coords
    of2 = np.zeros(occ2.shape, dtype=bool)
    if len(of_world) > 0:
        of_idx2 = of_world - vmin2  # shift into densify-grid indices
        inb = (
            (of_idx2[:, 0] >= 0) & (of_idx2[:, 0] < occ2.shape[0]) &
            (of_idx2[:, 1] >= 0) & (of_idx2[:, 1] < occ2.shape[1]) &
            (of_idx2[:, 2] >= 0) & (of_idx2[:, 2] < occ2.shape[2])
        )
        idx = of_idx2[inb]
        of2[idx[:, 0], idx[:, 1], idx[:, 2]] = True

    # 4. Save aligned observed_free next to temp_geom
    temp_free = vxw_dir / "_live_observed_free.npz"
    np.savez_compressed(temp_free, mask=of2, vmin=vmin2,
                        voxel_size=np.float64(obs.voxel_size))

    # 5. Run GVD pipeline → out_vxw
    cfg = GvdConfig(
        input_vxw=str(temp_geom),
        output_vxw=str(out_vxw),
        observed_free_path=str(temp_free),
        band_max=None,
        min_component=0,
        thin=True,
        rooms=True,
        prune_spurs_m=0.3,
        merge_close_m=0.2,
        drop_small_nodes=5,
        room_resolution=0.3,
    )
    gvd_run(cfg)

    print(f"[live] batch={batch_idx}  fi={frames_so_far}  occ={occ_count}  free={free_count}  -> {out_vxw}")


def _incremental_snapshot(
    obs,
    batch_idx: int,
    frames_so_far: int,
    out_vxw: Path,
    vxw_dir: Path,
    persistent_gvd: np.ndarray | None,
) -> np.ndarray:
    """Lightweight incremental GVD snapshot: recompute only the dirty bbox.

    Maintains `persistent_gvd` (full obsmap-grid-shape bool array) across calls.
    Returns the (possibly freshly allocated) persistent_gvd.

    Writes a geometric .vxw with the thinned persistent GVD skeleton stamped in
    cyan. Skips room detection (v2.0 scope bound).
    """
    from m3_adapter.gvd.render import stamp_skeleton  # local import to avoid circular

    occ_mask = obs.occupancy_mask()
    occ_count = int(occ_mask.sum())
    if occ_count == 0:
        log.info("  [incr] batch=%d  fi=%d  occ=0 — skipping", batch_idx, frames_so_far)
        if persistent_gvd is None:
            persistent_gvd = np.zeros(obs.logodds.shape, dtype=bool)
        return persistent_gvd

    # Allocate on first call
    if persistent_gvd is None:
        persistent_gvd = np.zeros(obs.logodds.shape, dtype=bool)

    bbox = obs.pop_dirty_bbox()
    if bbox is None:
        log.info("  [incr] batch=%d  fi=%d  no dirty bbox — skipping", batch_idx, frames_so_far)
        return persistent_gvd

    bmin, bmax = bbox
    box_size = tuple(int(x) for x in (bmax - bmin))

    free_mask = obs.observed_free_mask()

    # Time the local ESDF vs estimated full cost
    t0 = time.perf_counter()
    local = extract_gvd_local(
        occ_mask, free_mask, obs.voxel_size,
        bmin, bmax,
        margin_vox=25, d_min=0.20, theta_sep=0.40,
    )
    t_local = time.perf_counter() - t0

    replace_region(persistent_gvd, bmin, bmax, local)
    thinned = thin_gvd(persistent_gvd)

    # Write geometric vxw
    temp_geom = vxw_dir / "_live_geom.vxw"
    occupancy_to_vxw(occ_mask, obs.vmin, obs.voxel_size, temp_geom)

    # Stamp thinned GVD skeleton onto geometric world and write as live.vxw
    world2 = vxw.read_world(temp_geom)
    stamp_skeleton(world2, thinned, obs.vmin)
    vxw.write_world(out_vxw, world2)

    print(
        f"[incr] batch={batch_idx}  fi={frames_so_far}"
        f"  bbox={box_size}  local={t_local*1000:.0f}ms  -> {out_vxw}"
    )
    return persistent_gvd


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(levelname)-5s %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("bag_dir", type=Path,
                    help="rosbag2 directory (must contain metadata.yaml)")
    ap.add_argument("vxw_dir", type=Path,
                    help="path to .vxw directory — grid alignment source; "
                         "obsmap.npz and observed_free.npz are written here")
    ap.add_argument("--start-frame", type=int, default=0,
                    help="absolute depth-frame index to start integrating (default 0)")
    ap.add_argument("--end-frame", type=int, default=None,
                    help="absolute depth-frame index to stop (exclusive, default: all)")
    ap.add_argument("--pixel-stride", type=int, default=4,
                    help="downsample depth pixels by this factor (default 4)")
    ap.add_argument("--free-margin", type=float, default=0.10,
                    help="metres to stop short of surface (default 0.10)")
    ap.add_argument("--depth-min", type=float, default=0.2,
                    help="minimum valid depth in metres (default 0.2)")
    ap.add_argument("--depth-max", type=float, default=15.0,
                    help="maximum valid depth in metres (default 15.0)")
    ap.add_argument("--live", action="store_true",
                    help="write a renderable .vxw after each --batch frames")
    ap.add_argument("--batch", type=int, default=50,
                    help="frames per live snapshot batch (default 50)")
    ap.add_argument("--out-vxw", type=str, default=None,
                    help="output .vxw for live snapshots (default: <vxw_dir>/live.vxw)")
    ap.add_argument("--incremental", action="store_true",
                    help="only meaningful with --live: recompute GVD only in the "
                         "dirty box each batch (fast incremental path, skips rooms)")
    ap.add_argument("--rgb", action="store_true",
                    help="also integrate observed colour from /tesse/left_cam/rgb "
                         "into the ObsMap RGB channel (requires --semantic path)")
    ap.add_argument("--small-objects", action="store_true",
                    help="divert small-object-class hit points (books/vase — cup/key "
                         "scale, below grid resolution) into the point-level instance "
                         "channel; writes <vxw_dir>/small_objects.json")
    ap.add_argument("--skip-integrate", action="store_true",
                    help="decode/unproject only, no ObsMap integration — fast second "
                         "pass to harvest e.g. --small-objects without touching the map")
    ap.add_argument("--semantic", action="store_true",
                    help="read seg frames in lockstep with depth, feed per-point "
                         "labels into ObsMap, and export semantic-coloured .vxw")
    ap.add_argument("--hydra-cfg", type=Path, default=None,
                    help="Hydra workspace root for label space / colour CSV "
                         "(default: E:/aros_slam_ws/hydra_ws)")
    ap.add_argument("--scene", default="apartment",
                    choices=["apartment", "office", "subway"],
                    help="uHumans2 scene name (default: apartment)")
    args = ap.parse_args()

    live_out_vxw = Path(args.out_vxw) if args.out_vxw else None

    # ---- semantic setup (only when --semantic) ----
    sem_palette = None
    color_to_id = None
    if args.semantic:
        hydra_root = args.hydra_cfg if args.hydra_cfg is not None else Path("E:/aros_slam_ws/hydra_ws")
        yaml_path, csv_path = _resolve_hydra_paths(hydra_root, args.scene)
        label_names = load_label_space(yaml_path)
        color_to_id = load_color_to_super_id(csv_path)
        sem_palette = build_palette(label_names)
        log.info("[sem] loaded label space: %d names, %d colour rows",
                 len(label_names), len(color_to_id))

    S = args.start_frame
    E = args.end_frame  # may be None

    if not (args.bag_dir / "metadata.yaml").is_file():
        log.error("not a rosbag2 directory: %s", args.bag_dir)
        sys.exit(1)

    # ---- [1/5] load .vxw for grid alignment ----
    log.info("[1/5] loading .vxw for grid alignment: %s", args.vxw_dir)
    world = vxw.read_world(args.vxw_dir)
    occ, vmin = densify_occupancy(world, pad=1)
    voxel_size = float(world.manifest.voxel_size_meters)
    shape = occ.shape
    log.info("      grid shape=%s  vmin=%s  voxel_size=%.4fm",
             shape, vmin.tolist(), voxel_size)

    # ---- [2/5] load or create ObsMap ----
    obsmap_path = args.vxw_dir / "obsmap.npz"
    if obsmap_path.exists():
        log.info("[2/5] resuming: loading existing obsmap from %s", obsmap_path)
        obs = ObsMap.load(obsmap_path)
        # Verify compatibility with current grid
        if obs.logodds.shape != tuple(shape):
            raise ValueError(
                f"Stale obsmap: shape mismatch. "
                f"obsmap has {obs.logodds.shape}, grid requires {tuple(shape)}. "
                f"Delete {obsmap_path} to start fresh."
            )
        if not np.array_equal(obs.vmin, vmin):
            raise ValueError(
                f"Stale obsmap: vmin mismatch. "
                f"obsmap has vmin={obs.vmin.tolist()}, grid requires vmin={vmin.tolist()}. "
                f"Delete {obsmap_path} to start fresh."
            )
        log.info("      resumed: obsmap shape=%s vmin=%s", obs.logodds.shape, obs.vmin.tolist())
    else:
        log.info("[2/5] creating new obsmap (shape=%s)", shape)
        obs = ObsMap.new(shape, vmin, voxel_size)

    # Record state before this run
    occ0 = int(obs.occupancy_mask().sum())
    free0 = int(obs.observed_free_mask().sum())
    log.info("      state before run: occupied=%d  observed_free=%d", occ0, free0)

    if args.live and live_out_vxw is None:
        live_out_vxw = args.vxw_dir / "live.vxw"

    # Persistent GVD grid for --incremental mode (allocated lazily on first snapshot)
    persistent_gvd: np.ndarray | None = None

    # Small-object point-level channel (--small-objects, semantic path only)
    small_buf = None
    if args.small_objects and args.semantic:
        from m3_adapter.small_objects import SmallObjectBuffer
        small_buf = SmallObjectBuffer()

    # ---- [3/5] scan bag: odom + tf_static + camera_info ----
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)

    log.info("[3/5] scanning bag for odom + tf_static + depth camera_info")
    t0 = time.perf_counter()
    with Reader(str(args.bag_dir)) as reader:
        odom_t, odom_p, odom_q, world_frame, body_frame = collect_odom(reader, ts)
        tf_static = collect_tf_static(reader, ts)
        dci_conn = [c for c in reader.connections
                    if c.topic == "/tesse/depth_cam/camera_info"][0]
        for _, _, raw in reader.messages(connections=[dci_conn]):
            dci = ts.deserialize_cdr(raw, dci_conn.msgtype)
            fx, fy = float(dci.k[0]), float(dci.k[4])
            cx, cy = float(dci.k[2]), float(dci.k[5])
            cam_frame = dci.header.frame_id
            break

    T_body_cam = _chain_tf(tf_static, body_frame, cam_frame)
    if T_body_cam is None:
        log.warning("could not resolve %s -> %s via tf_static; using identity",
                    body_frame, cam_frame)
        T_body_cam = np.eye(4)

    log.info("      odom %d kfr  t=[%.3f,%.3f]  body=%s  cam=%s",
             len(odom_t), odom_t[0], odom_t[-1], body_frame, cam_frame)
    log.info("      tf_static entries: %d  T_body_cam t=%s",
             len(tf_static), T_body_cam[:3, 3].tolist())
    log.info("      depth K  fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx, fy, cx, cy)
    log.info("      bag scan done in %.2fs", time.perf_counter() - t0)

    # ---- [4/5] frame loop — depth + pose, integrate into ObsMap ----
    range_desc = f"[{S}, {E})" if E is not None else f"[{S}, end)"
    log.info("[4/5] integrating depth frames  frame range=%s  stride=%d",
             range_desc, args.pixel_stride)
    t0 = time.perf_counter()

    fi = 0                  # absolute depth-frame counter (counts ALL depth messages)
    frames_integrated = 0   # frames actually integrated this run
    frames_skipped = 0      # pose-invalid or out-of-range
    stride = args.pixel_stride

    with Reader(str(args.bag_dir)) as reader:
        dconn = [c for c in reader.connections
                 if c.topic == "/tesse/depth_cam/mono/image_raw"][0]

        if args.semantic:
            sconn = [c for c in reader.connections
                     if c.topic == "/tesse/seg_cam/rgb/image_raw"][0]
            depth_iter = reader.messages(connections=[dconn])
            seg_iter = reader.messages(connections=[sconn])
            rgb_iter = None
            if args.rgb:
                rconn = [c for c in reader.connections
                         if c.topic == "/tesse/left_cam/rgb/image_raw"][0]
                rgb_iter = reader.messages(connections=[rconn])

            while True:
                # Early-exit: if we've passed E there's nothing more to integrate
                if E is not None and fi >= E:
                    break

                try:
                    _dc, _dt, draw = next(depth_iter)
                    _sc, _st, sraw = next(seg_iter)
                    rraw = next(rgb_iter)[2] if rgb_iter is not None else None
                except StopIteration:
                    break

                dm = ts.deserialize_cdr(draw, dconn.msgtype)
                sm = ts.deserialize_cdr(sraw, sconn.msgtype)
                rm = ts.deserialize_cdr(rraw, rconn.msgtype) if rraw is not None else None

                if dm.encoding != "32FC1":
                    log.error("unexpected depth encoding: %s", dm.encoding)
                    sys.exit(2)
                if sm.encoding != "rgb8":
                    log.error("unexpected seg encoding: %s", sm.encoding)
                    sys.exit(2)
                if rm is not None and rm.encoding != "rgb8":
                    log.error("unexpected rgb encoding: %s", rm.encoding)
                    sys.exit(2)

                fi_current = fi
                fi += 1

                if fi_current < S:
                    continue

                depth = np.frombuffer(dm.data, dtype=np.float32).reshape(
                    dm.height, dm.width)
                seg = np.frombuffer(sm.data, dtype=np.uint8).reshape(
                    sm.height, sm.width, 3)
                rgb_img = None
                if rm is not None:
                    rgb_img = np.frombuffer(rm.data, dtype=np.uint8).reshape(
                        rm.height, rm.width, 3)
                if stride > 1:
                    depth = depth[::stride, ::stride]
                    seg = seg[::stride, ::stride]
                    if rgb_img is not None:
                        rgb_img = rgb_img[::stride, ::stride]

                t_frame = dm.header.stamp.sec + dm.header.stamp.nanosec * 1e-9
                T_world_body = _interp_T_world_body(t_frame, odom_t, odom_p, odom_q)
                if T_world_body is None:
                    frames_skipped += 1
                    continue

                # einsum, not @: the pixi env's numpy BLAS delay-load is broken (matmul/inv
                # crash natively); einsum is pure-C and numerically identical here.
                T_world_cam = np.einsum('ij,jk->ik', T_world_body, T_body_cam)

                xyz_cam, valid = unproject_depth(
                    depth,
                    fx / stride, fy / stride,
                    cx / stride, cy / stride,
                    z_min=args.depth_min, z_max=args.depth_max,
                )
                if xyz_cam.shape[0] == 0:
                    frames_skipped += 1
                    continue

                # Align seg labels to unprojected points via the valid mask
                seg_labels = seg_pixels_to_labels(seg, color_to_id, valid)
                point_colors = rgb_img[valid] if rgb_img is not None else None

                xyz_h = np.concatenate(
                    [xyz_cam, np.ones((xyz_cam.shape[0], 1), dtype=np.float32)], axis=1
                )
                xyz_world = np.einsum('nj,kj->nk', xyz_h, T_world_cam.astype(np.float32))[:, :3]
                P_m = ros_zup_to_vxw_yup(xyz_world.astype(np.float64))

                origin_ros = T_world_cam[:3, 3:4].T
                O_m = ros_zup_to_vxw_yup(origin_ros)[0]

                if small_buf is not None:
                    # 小物体点级实例通道:装不进栅格的类(杯/钥匙量级)按点分流
                    small_buf.add_frame(P_m, seg_labels, point_colors)

                if not args.skip_integrate:
                    obs.integrate_frame(O_m, P_m, point_labels=seg_labels,
                                        free_margin_m=args.free_margin,
                                        point_colors=point_colors)
                frames_integrated += 1

                if frames_integrated % 50 == 0:
                    occ_now = int(obs.occupancy_mask().sum())
                    free_now = int(obs.observed_free_mask().sum())
                    log.info("  fi=%d  integrated=%d skipped=%d  occ=%d free=%d",
                             fi_current, frames_integrated, frames_skipped,
                             occ_now, free_now)
                if args.live and frames_integrated % args.batch == 0:
                    if args.incremental:
                        persistent_gvd = _incremental_snapshot(
                            obs, frames_integrated // args.batch,
                            frames_integrated, live_out_vxw, args.vxw_dir,
                            persistent_gvd,
                        )
                    else:
                        _live_snapshot(obs, frames_integrated // args.batch,
                                       frames_integrated, live_out_vxw, args.vxw_dir,
                                       sem_palette=sem_palette)

        else:
            # Non-semantic path: depth-only (original behaviour)
            for _c, _t, draw in reader.messages(connections=[dconn]):
                # Early-exit: if we've passed E there's nothing more to integrate
                if E is not None and fi >= E:
                    break

                dm = ts.deserialize_cdr(draw, dconn.msgtype)

                if dm.encoding != "32FC1":
                    log.error("unexpected depth encoding: %s", dm.encoding)
                    sys.exit(2)

                # fi is counted for every depth message, whether in range or not
                fi_current = fi
                fi += 1

                # Skip frames outside [S, E)
                if fi_current < S:
                    continue

                # fi_current is in [S, E) — attempt to integrate
                depth = np.frombuffer(dm.data, dtype=np.float32).reshape(
                    dm.height, dm.width)
                if stride > 1:
                    depth = depth[::stride, ::stride]

                t_frame = dm.header.stamp.sec + dm.header.stamp.nanosec * 1e-9
                T_world_body = _interp_T_world_body(t_frame, odom_t, odom_p, odom_q)
                if T_world_body is None:
                    frames_skipped += 1
                    continue

                # einsum, not @: the pixi env's numpy BLAS delay-load is broken (matmul/inv
                # crash natively); einsum is pure-C and numerically identical here.
                T_world_cam = np.einsum('ij,jk->ik', T_world_body, T_body_cam)

                xyz_cam, _valid = unproject_depth(
                    depth,
                    fx / stride, fy / stride,
                    cx / stride, cy / stride,
                    z_min=args.depth_min, z_max=args.depth_max,
                )
                if xyz_cam.shape[0] == 0:
                    frames_skipped += 1
                    continue

                # Transform points to world frame (homogeneous multiply)
                xyz_h = np.concatenate(
                    [xyz_cam, np.ones((xyz_cam.shape[0], 1), dtype=np.float32)], axis=1
                )
                xyz_world = np.einsum('nj,kj->nk', xyz_h, T_world_cam.astype(np.float32))[:, :3]
                P_m = ros_zup_to_vxw_yup(xyz_world.astype(np.float64))

                # Camera origin: same coordinate swap as points
                origin_ros = T_world_cam[:3, 3:4].T  # (1, 3)
                O_m = ros_zup_to_vxw_yup(origin_ros)[0]  # (3,)

                obs.integrate_frame(O_m, P_m, free_margin_m=args.free_margin)
                frames_integrated += 1

                if frames_integrated % 50 == 0:
                    occ_now = int(obs.occupancy_mask().sum())
                    free_now = int(obs.observed_free_mask().sum())
                    log.info("  fi=%d  integrated=%d skipped=%d  occ=%d free=%d",
                             fi_current, frames_integrated, frames_skipped,
                             occ_now, free_now)
                if args.live and frames_integrated % args.batch == 0:
                    if args.incremental:
                        persistent_gvd = _incremental_snapshot(
                            obs, frames_integrated // args.batch,
                            frames_integrated, live_out_vxw, args.vxw_dir,
                            persistent_gvd,
                        )
                    else:
                        _live_snapshot(obs, frames_integrated // args.batch,
                                       frames_integrated, live_out_vxw, args.vxw_dir)

    log.info("      frame loop done: fi_total=%d  integrated=%d  skipped=%d  in %.2fs",
             fi, frames_integrated, frames_skipped, time.perf_counter() - t0)

    # ---- [5/5] save obsmap + observed_free export ----
    if args.skip_integrate:
        log.info("[5/5] --skip-integrate: leaving obsmap/observed_free untouched")
    else:
        log.info("[5/5] saving obsmap and observed_free")
        obs.save(obsmap_path)
        log.info("      obsmap saved: %s", obsmap_path)
    if small_buf is not None:
        import json
        small_ents = small_buf.finalize(label_names if args.semantic else {})
        sp = args.vxw_dir / "small_objects.json"
        sp.write_text(json.dumps(
            {"format_version": "1.0", "entities": small_ents}, indent=2))
        log.info("      small objects: %d instances -> %s", len(small_ents), sp)

    # Export observed-free mask for GVD pipeline (same format as uhumans2_carve)
    if not args.skip_integrate:
        free_out = args.vxw_dir / "observed_free.npz"
        np.savez_compressed(
            free_out,
            mask=obs.observed_free_mask(),
            vmin=obs.vmin,
            voxel_size=np.float64(obs.voxel_size),
        )
        log.info("      observed_free exported: %s", free_out)

    # Export final geometric .vxw (semantic-coloured when --semantic)
    if args.skip_integrate:
        pass          # harvest-only pass: nothing map-derived to export
    elif not args.live:
        # Non-live: we write a final geometric vxw from the full map
        occ_mask = obs.occupancy_mask()
        if int(occ_mask.sum()) > 0:
            final_geom = args.vxw_dir / "obsmap_geom.vxw"
            if args.semantic:
                occupancy_to_vxw(
                    occ_mask, obs.vmin, obs.voxel_size, final_geom,
                    semantic_grid=obs.semantic_grid(), palette=sem_palette,
                )
            else:
                occupancy_to_vxw(occ_mask, obs.vmin, obs.voxel_size, final_geom)
            log.info("      geometric vxw exported: %s", final_geom)
    elif args.semantic:
        # Live path: emit one final semantic snapshot
        occ_mask = obs.occupancy_mask()
        if int(occ_mask.sum()) > 0:
            final_sem = live_out_vxw if live_out_vxw else args.vxw_dir / "live.vxw"
            occupancy_to_vxw(
                occ_mask, obs.vmin, obs.voxel_size, final_sem,
                semantic_grid=obs.semantic_grid(), palette=sem_palette,
            )
            log.info("      semantic geometric vxw exported: %s", final_sem)

    # Final stats
    occN = int(obs.occupancy_mask().sum())
    freeN = int(obs.observed_free_mask().sum())
    print(
        f"\nRESULT"
        f"  frames_integrated={frames_integrated}"
        f"  frame_range={range_desc}"
        f"  occupied={occ0}->{occN}"
        f"  observed_free={free0}->{freeN}"
        f"  obsmap={obsmap_path}"
    )


if __name__ == "__main__":
    main()
