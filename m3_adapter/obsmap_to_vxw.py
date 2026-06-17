"""obsmap_to_vxw.py — CLI: export a game-ready .vxw from an ObsMap .npz.

The ObsMap is the single source of truth for the map. This script derives a
complete, game-ready .vxw (semantic voxels + furniture entities + spawn_hint)
by calling obsmap_export.obsmap_to_vxw_full, which internally reuses the
entity/spawn helpers from uhumans2_to_vxw.

Usage:
    pixi run python m3_adapter/obsmap_to_vxw.py \\
        <obsmap_npz_or_dir> <out_vxw> \\
        --hydra-cfg E:/aros_slam_ws/hydra_ws \\
        --scene apartment \\
        [--dbscan-eps 2.0] [--min-samples 10]

If the first argument is a directory, obsmap.npz is looked up inside it.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Export a game-ready .vxw from an ObsMap .npz "
                    "(semantic voxels + entities + spawn_hint).",
    )
    ap.add_argument(
        "obsmap_npz_or_dir",
        type=Path,
        help="Path to obsmap.npz OR a directory containing obsmap.npz.",
    )
    ap.add_argument(
        "out_vxw",
        type=Path,
        help="Destination .vxw directory.",
    )
    ap.add_argument(
        "--hydra-cfg",
        type=Path,
        required=True,
        help="Root of the Hydra workspace (contains install/ or src/ uhumans2 config).",
    )
    ap.add_argument(
        "--scene",
        choices=["apartment", "office", "subway"],
        default="apartment",
        help="uHumans2 scene name (selects label_space yaml). Default: apartment.",
    )
    ap.add_argument(
        "--chunk-extent",
        type=int,
        default=32,
        help="Voxels per chunk side (default 32).",
    )
    ap.add_argument(
        "--dbscan-eps",
        type=float,
        default=2.0,
        dest="dbscan_eps",
        help="DBSCAN epsilon in voxel units for entity clustering (default 2.0).",
    )
    ap.add_argument(
        "--min-samples",
        type=int,
        default=10,
        dest="min_samples",
        help="DBSCAN min_samples for entity clustering (default 10).",
    )
    args = ap.parse_args()

    # Resolve input path
    npz_path = args.obsmap_npz_or_dir
    if npz_path.is_dir():
        npz_path = npz_path / "obsmap.npz"
    if not npz_path.is_file():
        print(f"ERROR: cannot find obsmap npz at {npz_path}", file=sys.stderr)
        sys.exit(1)

    print(f"ObsMap : {npz_path}")
    print(f"Output : {args.out_vxw}")
    print(f"Scene  : {args.scene}  Hydra cfg: {args.hydra_cfg}")

    from m3_adapter.obsmap_export import obsmap_to_vxw_full

    world, entities, obsmap = obsmap_to_vxw_full(
        obsmap_npz_path=npz_path,
        out_vxw=args.out_vxw,
        hydra_cfg=args.hydra_cfg,
        scene=args.scene,
        chunk_extent=args.chunk_extent,
        dbscan_eps_voxels=args.dbscan_eps,
        min_samples=args.min_samples,
    )

    # Count occupied voxels in output
    total_occ = sum(
        int((c.voxels["material_id"] != 0).sum()) for c in world.chunks.values()
    )
    print(f"Occupied voxels (all) : {int(obsmap.occupancy_mask().sum())}")
    print(f"Structure voxels      : {total_occ}")
    print(f"Entities              : {len(entities)}")
    print(f"Spawn hint            : {world.manifest.spawn_hint}")
    print(f"Written to            : {args.out_vxw}")


if __name__ == "__main__":
    main()
