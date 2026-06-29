"""build_apt_coarse.py — canonical, reproducible build of the apartment world.

The GENERAL path (not per-file post-processing): take the persisted fine ObsMap
and run it through the standard completion pipeline. Every fix lives in a
generator / pipeline flag that works for ANY scene:

  • SlabFill(floor/ceiling) — solidify the horizontal slabs
  • WallFill                — complete walls from plane-peak detection
  • StairsFill              — solidify observed stair treads into solid steps,
                              stopping at observed-free/occupied (no overflow,
                              no scene-specific ground clamp)
  • OcclusionFill           — seal near-enclosed unknown pinholes
  • clean=True              — drop floating sensor specks (small components)
  • coarsen_to_m=0.2        — Minecraft-scale blocks

The earlier apt_coarse.vxw was built with only [SlabFill, SlabFill, WallFill]
and clean=False — that's why stairs weren't solid and floating junk remained.
This restores the full set. Openings (doors/windows) stay open because the
generators read the ObsMap observed-free channel; close_radius=0 so no
morphological closing walls them shut.

Usage:  python scripts/build_apt_coarse.py [out_dir]
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import vxw_format as V
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.export import obsmap_to_completed_vxw
from m3_adapter.vlayer.generators.slab import SlabFill
from m3_adapter.vlayer.generators.wall import WallFill
from m3_adapter.vlayer.generators.stairs import StairsFill
from m3_adapter.vlayer.generators.occlusion import OcclusionFill

OBSMAP = "out/uhumans2_apt_full.vxw/obsmap.npz"
PALETTE_FROM = "out/apt_coarse.vxw/palette.json"   # reuse the original semantic palette


def main(out_dir: str = "out/apt_coarse_v2.vxw") -> None:
    obsmap = ObsMap.load(OBSMAP)
    palette = V.read_palette(Path(PALETTE_FROM))
    generators = [
        SlabFill(3, "floor"),
        SlabFill(4, "ceiling"),
        WallFill(),
        StairsFill(),
        OcclusionFill(),
    ]
    print("[build] obsmap occ=%d  vs=%.3f  -> %s"
          % (int(obsmap.occupancy_mask().sum()), obsmap.voxel_size, out_dir))
    obsmap_to_completed_vxw(
        obsmap, out_dir, generators=generators, palette=palette,
        coarsen_to_m=0.2, clean=True, clean_close_radius=0, clean_keep_largest=True,
    )
    print("[build] done")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out/apt_coarse_v2.vxw")
