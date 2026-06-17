"""obsmap_export.py — derive a renderable geometric .vxw from an ObsMap's
occupancy (one concrete material, or semantic-coloured when a semantic_grid is
given). Bridges the log-odds map to the .vxw format the Godot viewer / GVD
pipeline read. (SP-B v1 / v3.)

Also provides ``obsmap_to_world`` — the full exporter that makes the ObsMap the
single source of truth for game-ready .vxw output (semantic voxels + furniture
entities + spawn_hint). See ``obsmap_to_vxw_full`` for the top-level entry point
that resolves Hydra paths and writes a file. The batch ``uhumans2_to_vxw``
adapter is now deprecated for new use; its entity/spawn helpers are imported
here so both adapters share one implementation.
"""
from __future__ import annotations
import sys
import uuid
import time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.common import build_concrete_palette  # noqa: E402


def occupancy_to_vxw(
    occ_mask,
    vmin,
    voxel_size,
    out_path,
    chunk_extent=32,
    semantic_grid=None,
    palette=None,
):
    """Write a geometric .vxw from the occupied cells of *occ_mask*.

    Non-semantic mode (default):
        Every True cell gets material_id=1, semantic_id=1, concrete palette.

    Semantic mode (when *semantic_grid* and *palette* are provided):
        Every True cell gets material_id = semantic_id = the uint8 super_id
        from *semantic_grid* at the same position.  Cells occupied but with
        semantic label 0 (unknown) fall back to material_id=1 so they still
        render.  *palette* must be a vxw.Palette whose material ids cover the
        range of super_ids present (build_palette from uhumans2_to_vxw
        satisfies this).

    Args:
        occ_mask:       bool ndarray (nx, ny, nz) — True = occupied.
        vmin:           world-voxel coord of mask[0,0,0] (int64 (3,)).
        voxel_size:     metres per voxel side (float).
        out_path:       destination .vxw path.
        chunk_extent:   voxels per chunk side (<=255, default 32).
        semantic_grid:  optional uint8 ndarray same shape as occ_mask;
                        super_id per cell (0 = unknown).
        palette:        optional vxw.Palette to use instead of concrete.
    """
    coords = np.argwhere(occ_mask).astype(np.int64) + np.asarray(vmin, dtype=np.int64)
    if len(coords) == 0:
        raise ValueError("occupancy mask is empty")
    if chunk_extent > 255:
        raise ValueError(f"chunk_extent {chunk_extent} exceeds uint8 max (255)")

    # Gather per-occupied-cell semantic labels if requested.
    use_semantic = semantic_grid is not None and palette is not None
    if use_semantic:
        # occ_idx[i] = (ix, iy, iz) index into occ_mask / semantic_grid
        occ_idx = np.argwhere(occ_mask).astype(np.int64)  # (K, 3)
        cell_labels = semantic_grid[occ_idx[:, 0], occ_idx[:, 1], occ_idx[:, 2]]  # (K,) uint8
        # Unknown (label 0) occupied cells: fall back to material_id=1
        mat_ids = cell_labels.copy()
        mat_ids[mat_ids == 0] = 1

    cc = np.floor_divide(coords, chunk_extent)
    local = (coords - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)
    chunks = {}
    for chunk_id in range(len(chunk_keys)):
        mask = inverse == chunk_id
        loc = local[mask]
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
        if use_semantic:
            m_ids = mat_ids[mask]
            s_ids = cell_labels[mask]
            arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = m_ids
            arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = s_ids
        else:
            arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 1
            arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 1
        ckey = tuple(int(x) for x in chunk_keys[chunk_id])
        chunks[ckey] = vxw.Chunk(coord=ckey, voxels=arr,
                                 encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP)
    keys = np.array([tuple(int(x) for x in ck) for ck in chunk_keys])
    man = vxw.Manifest(
        world_id="obsmap_live", voxel_size_meters=float(voxel_size), chunk_extent=chunk_extent,
        bounds_chunks_min=tuple(int(x) for x in keys.min(axis=0)),
        bounds_chunks_max=tuple(int(x) + 1 for x in keys.max(axis=0)))
    chosen_palette = palette if use_semantic else build_concrete_palette()
    vxw.write_world(Path(out_path), vxw.World(manifest=man, palette=chosen_palette, chunks=chunks))


# ---------------------------------------------------------------------------
# Full ObsMap → game-ready .vxw exporter
# ---------------------------------------------------------------------------

def obsmap_to_world(
    obsmap,
    label_names,
    palette,
    voxel_size=None,
    chunk_extent=32,
    extract_entities_fn=None,
    find_spawn_fn=None,
    dbscan_eps_voxels=2.0,
    min_samples=10,
):
    """Derive a complete game-ready vxw_format.World from an ObsMap:
    semantic occupancy voxels + furniture entities + spawn_hint.

    Mirrors what uhumans2_to_vxw.main produces, but sourced from the ObsMap
    (the single source of truth). extract_entities_fn / find_spawn_fn are the
    uhumans2_to_vxw helpers (injected to avoid a hard import cycle).

    Args:
        obsmap:               ObsMap instance.
        label_names:          dict[int, str] — super_id → name.
        palette:              vxw.Palette — covers all super_ids in label_names.
        voxel_size:           override voxel_size (defaults to obsmap.voxel_size).
        chunk_extent:         voxels per chunk side (default 32).
        extract_entities_fn:  callable matching extract_entities signature; if
                              None, no entity extraction is performed.
        find_spawn_fn:        callable matching _find_spawn_hint signature; if
                              None, spawn_hint is left as None.
        dbscan_eps_voxels:    forwarded to extract_entities_fn.
        min_samples:          forwarded to extract_entities_fn.

    Returns:
        (world, entities) — vxw.World and the list of vxw.Entity objects.
    """
    vs = float(voxel_size) if voxel_size is not None else float(obsmap.voxel_size)

    # --- Step 1: pull occupancy + semantics from the map ---
    occ = obsmap.occupancy_mask()          # bool (nx, ny, nz)
    sem = obsmap.semantic_grid()           # uint8 (nx, ny, nz)
    vmin = np.asarray(obsmap.vmin, dtype=np.int64)

    # --- Step 2: build flat (N,3) world voxel coords + labels ---
    occ_idx = np.argwhere(occ)             # (N,3) LOCAL grid indices
    final_vc = occ_idx.astype(np.int64) + vmin   # world voxel coords
    final_lbl = sem[occ_idx[:, 0], occ_idx[:, 1], occ_idx[:, 2]].astype(np.uint8)

    # --- Step 3: drop label-0 (unknown / air) voxels ---
    keep0 = final_lbl != 0
    final_vc = final_vc[keep0]
    final_lbl = final_lbl[keep0]

    if len(final_vc) == 0:
        raise ValueError("ObsMap has no occupied voxels with a known semantic label")

    # --- Step 4: entity extraction ---
    entities: list = []
    keep_mask = np.ones(len(final_vc), dtype=bool)
    if extract_entities_fn is not None:
        entities, keep_mask = extract_entities_fn(
            final_vc, final_lbl, vs, label_names, dbscan_eps_voxels, min_samples,
        )

    # Structure voxels = those NOT claimed by an entity.
    struct_vc = final_vc[keep_mask]
    struct_lbl = final_lbl[keep_mask]

    # --- Step 5: spawn hint (computed on structure voxels; floor voxels survive
    #             entity removal because floor is not an _OBJECT_LABEL) ---
    spawn = None
    if find_spawn_fn is not None:
        spawn = find_spawn_fn(struct_vc, struct_lbl, vs)

    # --- Step 6: build semantic-id → default_material lookup from palette ---
    sem_to_mat: dict[int, int] = {}
    for sc in palette.semantic_classes:
        sem_to_mat[sc.id] = sc.default_material

    # --- Step 7: chunk the structure voxels ---
    cc = np.floor_divide(struct_vc, chunk_extent)
    local = (struct_vc - cc * chunk_extent).astype(np.uint8)
    chunk_keys, inverse = np.unique(cc, axis=0, return_inverse=True)

    chunks: dict = {}
    for ki in range(len(chunk_keys)):
        mask = inverse == ki
        locs = local[mask]
        lbls = struct_lbl[mask]
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
        # material_id = palette's default_material for this semantic class;
        # fall back to 1 (solid) if the super_id isn't in the palette.
        mat_ids = np.fromiter(
            (sem_to_mat.get(int(l), 1) for l in lbls), dtype=np.uint8, count=len(lbls),
        )
        arr["material_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = mat_ids
        arr["semantic_id"][locs[:, 0], locs[:, 1], locs[:, 2]] = lbls
        ck = tuple(int(x) for x in chunk_keys[ki])
        chunks[ck] = vxw.Chunk(
            coord=ck, voxels=arr,
            encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP,
        )

    # --- Step 8: manifest ---
    ck_arr = np.array(list(chunks.keys()), dtype=np.int64)
    bmin = tuple(int(x) for x in ck_arr.min(axis=0))
    bmax = tuple(int(x) + 1 for x in ck_arr.max(axis=0))
    manifest = vxw.Manifest(
        world_id=str(uuid.uuid4()),
        voxel_size_meters=vs,
        chunk_extent=chunk_extent,
        bounds_chunks_min=bmin,
        bounds_chunks_max=bmax,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_slam_system="obsmap",
        spawn_hint=spawn,
    )

    world = vxw.World(manifest=manifest, palette=palette, chunks=chunks, entities=entities)
    return world, entities


def obsmap_to_vxw_full(obsmap_npz_path, out_vxw, hydra_cfg, scene="apartment",
                       chunk_extent=32, dbscan_eps_voxels=2.0, min_samples=10):
    """Load an ObsMap .npz, resolve Hydra label config, export a full .vxw.

    This is the thin entry point used by obsmap_to_vxw.py CLI.  It lazy-imports
    uhumans2_to_vxw helpers so the import cost is borne only when needed.

    Args:
        obsmap_npz_path: path to the .npz file (or a directory containing obsmap.npz).
        out_vxw:         destination .vxw directory path.
        hydra_cfg:       root of the Hydra workspace (passed to _resolve_hydra_paths).
        scene:           scene name, e.g. "apartment".
        chunk_extent:    voxels per chunk side.
        dbscan_eps_voxels: DBSCAN epsilon in voxel units.
        min_samples:     DBSCAN min_samples.

    Returns:
        (world, entities, obsmap)
    """
    from m3_adapter.obsmap import ObsMap
    from m3_adapter.uhumans2_to_vxw import (
        _resolve_hydra_paths, load_label_space, build_palette,
        extract_entities, _find_spawn_hint,
    )

    obsmap_npz_path = Path(obsmap_npz_path)
    if obsmap_npz_path.is_dir():
        obsmap_npz_path = obsmap_npz_path / "obsmap.npz"

    obsmap = ObsMap.load(obsmap_npz_path)

    yaml_path, _csv_path = _resolve_hydra_paths(Path(hydra_cfg), scene)
    label_names = load_label_space(yaml_path)
    palette = build_palette(label_names)

    world, entities = obsmap_to_world(
        obsmap, label_names, palette,
        chunk_extent=chunk_extent,
        extract_entities_fn=extract_entities,
        find_spawn_fn=_find_spawn_hint,
        dbscan_eps_voxels=dbscan_eps_voxels,
        min_samples=min_samples,
    )

    vxw.write_world(Path(out_vxw), world)
    return world, entities, obsmap
