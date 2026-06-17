"""bench_incremental.py — show that local (dirty-box) GVD recompute is far
cheaper than full-grid recompute. (SP-B v2.) Usage:
  pixi run python m3_adapter/bench_incremental.py <vxw_dir> [--box 60]"""
from __future__ import annotations
import sys, time, argparse
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.gvd.field import (  # noqa: E402
    densify_occupancy, compute_esdf, extract_gvd, extract_gvd_local)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("vxw_dir")
    ap.add_argument("--box", type=int, default=60, help="dirty box side in voxels")
    ap.add_argument("--margin", type=int, default=25)
    args = ap.parse_args()
    world = vxw.read_world(Path(args.vxw_dir))
    occ, vmin = densify_occupancy(world, pad=1)
    vs = world.manifest.voxel_size_meters
    free = ~occ  # geometry-only free for the benchmark
    # full
    t0 = time.perf_counter()
    dist, parent = compute_esdf(occ, vs)
    _ = extract_gvd(free, dist, parent, vs)
    t_full = time.perf_counter() - t0
    # local: a box in the middle of the occupied region
    c = (np.argwhere(occ).mean(axis=0)).astype(int)
    h = args.box // 2
    bmin = np.maximum(c - h, 0)
    bmax = np.minimum(c + h, np.array(occ.shape))
    t0 = time.perf_counter()
    _ = extract_gvd_local(occ, free, vs, bmin, bmax, margin_vox=args.margin)
    t_local = time.perf_counter() - t0
    print(f"[bench] grid {occ.shape}  box {tuple(int(x) for x in (bmax-bmin))}  margin {args.margin}")
    print(f"[bench] full  ESDF+GVD: {t_full*1000:.0f} ms")
    print(f"[bench] local ESDF+GVD: {t_local*1000:.0f} ms  ({t_full/max(t_local,1e-6):.0f}x faster)")


if __name__ == "__main__":
    main()
