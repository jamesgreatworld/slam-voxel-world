"""voxel_postprocess — quality / cleanup passes over a voxel grid.

Goal: keep the import adapters (uhumans2 / pcd / tsdf / hydra_mesh / ...)
faithful to their raw source by default, and surface the "fix walls, fill
1-cell holes, drop floaters" logic as opt-in calls. Two ways to use:

  1. **Inline from an adapter**, when the adapter still has the per-voxel
     observation count from its accumulation pass:
        from m3_adapter.voxel_postprocess import denoise_by_count, close_holes
        vc, lbl = denoise_by_count(vc, lbl, counts, min_n=3)
        vc, lbl = close_holes(vc, lbl, iterations=1)

  2. **Standalone on any .vxw**, after the fact, with no count signal:
        pixi run python m3_adapter/voxel_postprocess.py \\
            out/14floor.vxw out/14floor_clean.vxw \\
            --close-iters 1

Hydra parity rule: hydra_mesh_to_vxw does NOT call these by default. To
mirror raw Hydra exactly, run it once and stop. To optimise further, run
the standalone pass — that keeps the "Hydra path" reproducible while
letting us pile on Python-side improvements.

The closing pass cannot invent labels for newly-filled voxels; it inherits
the label of the nearest pre-existing voxel via scipy.spatial.cKDTree.
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

log = logging.getLogger("voxel_postprocess")

# Cap the dense grid we allocate during closing so a 100 m world at 1 cm
# voxels doesn't OOM the box. 1 cell = 1 byte (bool), so 512M cells ≈ 512 MB.
_DENSE_GRID_CAP = 512_000_000


def denoise_by_count(
    vc: np.ndarray,
    lbl: np.ndarray,
    counts: np.ndarray,
    min_n: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Drop voxels seen in fewer than `min_n` frames.

    Args:
        vc:      (N, 3) int voxel coordinates.
        lbl:     (N,) uint8 labels parallel to vc.
        counts:  (N,) int observation counts parallel to vc.
        min_n:   threshold; 1 = no-op.

    Returns:
        (kept_vc, kept_lbl, dropped_n)
    """
    if min_n <= 1 or vc.size == 0:
        return vc, lbl, 0
    keep = counts >= min_n
    dropped = int((~keep).sum())
    return vc[keep], lbl[keep], dropped


def close_holes(
    vc: np.ndarray,
    lbl: np.ndarray,
    iterations: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Binary morphological closing on the occupancy grid; fill 1-cell holes.

    Newly-occupied cells inherit the label of their nearest pre-existing
    voxel (KDTree, k=1). Voxels that were already occupied keep their label.

    Args:
        vc:         (N, 3) int voxel coords.
        lbl:        (N,) uint8 labels.
        iterations: number of binary_closing iterations; 0 = no-op.

    Returns:
        (new_vc, new_lbl, filled_n)
    """
    if iterations <= 0 or vc.size == 0:
        return vc, lbl, 0
    try:
        from scipy.ndimage import binary_closing
        from scipy.spatial import cKDTree
    except ImportError as exc:
        log.warning("close_holes skipped: scipy not available (%s)", exc)
        return vc, lbl, 0

    mn = vc.min(axis=0)
    mx = vc.max(axis=0)
    shape = tuple(int(s) for s in (mx - mn + 1))
    cells = int(np.prod(shape))
    if cells > _DENSE_GRID_CAP:
        log.warning("close_holes skipped: dense grid %s exceeds cap %d cells",
                    shape, _DENSE_GRID_CAP)
        return vc, lbl, 0

    grid = np.zeros(shape, dtype=bool)
    local = (vc - mn).astype(np.int32)
    grid[local[:, 0], local[:, 1], local[:, 2]] = True
    closed = binary_closing(grid, iterations=iterations)
    new_mask = closed & ~grid
    new_local = np.argwhere(new_mask).astype(np.int32)
    if new_local.size == 0:
        return vc, lbl, 0
    new_vc = new_local + mn
    tree = cKDTree(vc.astype(np.float32))
    _, nn = tree.query(new_vc.astype(np.float32), k=1)
    new_lbl = lbl[nn]
    out_vc = np.concatenate([vc, new_vc.astype(np.int32)], axis=0)
    out_lbl = np.concatenate([lbl, new_lbl], axis=0)
    return out_vc, out_lbl, int(new_local.shape[0])


# ---------------------------------------------------------------------------
# Standalone CLI: read a .vxw, run close_holes, write a new .vxw.
# (denoise_by_count is not exposed standalone because per-voxel counts are
# discarded by the time we serialise.)
# ---------------------------------------------------------------------------

def _vxw_to_arrays(world: "vxw.World") -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Flatten all chunks' occupied voxels into parallel (vc, lbl, sem) arrays."""
    e = world.manifest.chunk_extent
    coords_list: list[np.ndarray] = []
    lbls_list: list[np.ndarray] = []
    sems_list: list[np.ndarray] = []
    for ck_coord, chunk in world.chunks.items():
        cx, cy, cz = ck_coord
        occ = chunk.voxels["material_id"] != 0
        idx = np.argwhere(occ).astype(np.int32)
        if idx.size == 0:
            continue
        world_coords = idx + np.array([cx * e, cy * e, cz * e], dtype=np.int32)
        coords_list.append(world_coords)
        lbls_list.append(chunk.voxels["material_id"][idx[:, 0], idx[:, 1], idx[:, 2]])
        sems_list.append(chunk.voxels["semantic_id"][idx[:, 0], idx[:, 1], idx[:, 2]])
    if not coords_list:
        return (np.zeros((0, 3), dtype=np.int32),
                np.zeros((0,), dtype=np.uint8),
                np.zeros((0,), dtype=np.uint8))
    return (
        np.concatenate(coords_list, axis=0),
        np.concatenate(lbls_list, axis=0),
        np.concatenate(sems_list, axis=0),
    )


def _arrays_to_chunks(
    vc: np.ndarray,
    lbl: np.ndarray,
    sem: np.ndarray,
    chunk_extent: int,
    compression: "vxw.Compression",
) -> tuple[dict, tuple, tuple]:
    cc = np.floor_divide(vc, chunk_extent).astype(np.int32)
    local = (vc - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)
    chunks: dict = {}
    for ki in range(len(chunk_keys)):
        mask = inverse == ki
        locs = local[mask]
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = lbl[mask]
        arr["semantic_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = sem[mask]
        ck = tuple(int(x) for x in chunk_keys[ki])
        chunks[ck] = vxw.Chunk(coord=ck, voxels=arr,
                                encoding=vxw.Encoding.RLE,
                                compression=compression)
    bmin = tuple(int(x) for x in cc.min(axis=0))
    bmax = tuple(int(x) + 1 for x in cc.max(axis=0))
    return chunks, bmin, bmax


_COMP_MAP = {
    "raw": vxw.Compression.RAW,
    "gzip": vxw.Compression.GZIP,
    "zstd": vxw.Compression.ZSTD,
}


def postprocess_vxw(
    src_path: Path,
    dst_path: Path,
    close_iters: int,
    compression: str = "gzip",
) -> dict:
    """Standalone: read src .vxw, run close_holes, write dst .vxw.

    The entities.json is copied as-is (entities live above the voxel grid).
    Returns a small stats dict for logging."""
    t0 = time.perf_counter()
    world = vxw.read_world(src_path)
    vc, lbl, sem = _vxw_to_arrays(world)
    before = int(vc.shape[0])
    vc, lbl, filled = close_holes(vc, lbl, close_iters)
    # Newly-filled voxels also need a semantic_id; reuse the closest's
    # semantic just like the label inheritance.
    if filled > 0:
        # Recompute semantic for the appended rows: they are at the tail of vc.
        # The cleanest way is to do close_holes once with a stacked (lbl, sem)
        # vector. Re-run a tiny lookup so semantic stays consistent.
        if lbl.shape[0] > sem.shape[0]:
            try:
                from scipy.spatial import cKDTree
                orig_n = sem.shape[0]
                new_vc = vc[orig_n:]
                tree = cKDTree(vc[:orig_n].astype(np.float32))
                _, nn = tree.query(new_vc.astype(np.float32), k=1)
                sem = np.concatenate([sem, sem[nn]], axis=0)
            except ImportError:
                sem = np.concatenate(
                    [sem, np.full(lbl.shape[0] - sem.shape[0], 0, dtype=np.uint8)],
                    axis=0,
                )
    chunks, bmin, bmax = _arrays_to_chunks(
        vc, lbl, sem, world.manifest.chunk_extent, _COMP_MAP[compression],
    )
    new_manifest = vxw.Manifest(
        world_id=world.manifest.world_id,
        voxel_size_meters=world.manifest.voxel_size_meters,
        chunk_extent=world.manifest.chunk_extent,
        bounds_chunks_min=bmin,
        bounds_chunks_max=bmax,
        coord_convention=world.manifest.coord_convention,
        world_up_axis=world.manifest.world_up_axis,
        scene_scale_meters_per_unit=world.manifest.scene_scale_meters_per_unit,
        world_origin_in_slam_frame=world.manifest.world_origin_in_slam_frame,
        lod_levels=world.manifest.lod_levels,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system=world.manifest.source_slam_system + "+postprocess",
        source_sensor=world.manifest.source_sensor,
        raw_data_hash=world.manifest.raw_data_hash,
        format_version=world.manifest.format_version,
        spawn_hint=world.manifest.spawn_hint,
    )
    new_world = vxw.World(manifest=new_manifest, palette=world.palette,
                           chunks=chunks, entities=world.entities)
    vxw.write_world(dst_path, new_world)
    return {
        "before_voxels": before,
        "after_voxels": int(vc.shape[0]),
        "filled": filled,
        "chunks": len(chunks),
        "elapsed_s": round(time.perf_counter() - t0, 2),
    }


def main() -> None:
    logging.basicConfig(level=logging.INFO,
                        format="[%(levelname)s] %(name)s  %(message)s")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("src_vxw", type=Path)
    ap.add_argument("dst_vxw", type=Path)
    ap.add_argument("--close-iters", type=int, default=1,
                    help="binary closing iterations (default 1, 0=off)")
    ap.add_argument("--compression", choices=["raw", "gzip", "zstd"],
                    default="gzip")
    args = ap.parse_args()
    if not args.src_vxw.is_dir():
        log.error("src_vxw not found: %s", args.src_vxw)
        sys.exit(1)
    stats = postprocess_vxw(
        args.src_vxw, args.dst_vxw,
        close_iters=args.close_iters,
        compression=args.compression,
    )
    log.info("done: %s", stats)


if __name__ == "__main__":
    main()
