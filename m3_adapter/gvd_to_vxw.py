"""gvd_to_vxw.py — CLI entry for the batch GVD pipeline.

Thin shell: parse args -> GvdConfig -> m3_adapter.gvd.pipeline.run. All logic
lives in the m3_adapter/gvd/ subpackage (field / graph / rooms / render /
pipeline). See docs/superpowers/specs/2026-06-16-gvd-subpackage-architecture.md.

Usage (settled apartment command):
  pixi run python m3_adapter/gvd_to_vxw.py in.vxw out.vxw \
      --seed 3.875,1.2,5.575 --band-max 1.0 --min-component 30 --thin \
      --rooms --prune-spurs 0.3 --merge-close 0.2 --room-resolution 0.3
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from m3_adapter.gvd.pipeline import GvdConfig, run  # noqa: E402


def _parse_seed(s: str):
    parts = s.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--seed needs 'x,y,z' in metres")
    return [float(p) for p in parts]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input_vxw")
    ap.add_argument("output_vxw")
    ap.add_argument("--seed", type=_parse_seed, default=None,
                    help="interior seed 'x,y,z' in metres (else spawn_hint/auto)")
    ap.add_argument("--pad", type=int, default=1)
    ap.add_argument("--band-max", type=float, default=1.0,
                    help="shell-band: keep free cells within N m of a surface (<=0 off)")
    ap.add_argument("--min-component", type=int, default=0,
                    help="drop obstacle components smaller than N voxels (0=off)")
    ap.add_argument("--d-min", type=float, default=0.20,
                    help="min clearance (m) for a GVD voxel")
    ap.add_argument("--theta-sep", type=float, default=0.40,
                    help="min parent spacing (m) for the GVD divergence test")
    ap.add_argument("--thin", action="store_true",
                    help="skeletonize the GVD voxel set to ~1-voxel curves")
    ap.add_argument("--graph", action="store_true",
                    help="sparsify the (thinned) skeleton into a places graph")
    ap.add_argument("--merge-radius", type=float, default=0.15,
                    help="skeleton_to_graph key-voxel merge radius (m)")
    ap.add_argument("--prune-spurs", type=float, default=0.0,
                    help="prune dead-end (degree-1) edges shorter than N m (0=off)")
    ap.add_argument("--merge-close", type=float, default=0.0,
                    help="merge graph nodes within N m of each other (0=off)")
    ap.add_argument("--rooms", action="store_true",
                    help="partition the places graph into rooms (implies --graph)")
    ap.add_argument("--room-resolution", type=float, default=1.0,
                    help="Louvain resolution: higher = more, smaller rooms")
    ap.add_argument("--observed-free", type=str, default=None,
                    help="path to observed_free.npz; use it as the free-space "
                         "source instead of flood (ray-carved, bounds the leak)")
    args = ap.parse_args()

    bmax = args.band_max if args.band_max and args.band_max > 0 else None
    cfg = GvdConfig(
        input_vxw=args.input_vxw, output_vxw=args.output_vxw,
        seed_metres=args.seed, pad=args.pad, band_max=bmax,
        min_component=args.min_component, d_min=args.d_min, theta_sep=args.theta_sep,
        thin=args.thin, graph=args.graph, merge_radius_m=args.merge_radius,
        prune_spurs_m=args.prune_spurs, merge_close_m=args.merge_close,
        rooms=args.rooms, room_resolution=args.room_resolution,
        observed_free_path=args.observed_free,
    )
    run(cfg)


if __name__ == "__main__":
    main()
