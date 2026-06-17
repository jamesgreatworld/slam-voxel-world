# Live Build Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate rosbag frames in batches and, after each batch, write a renderable .vxw (occupancy + GVD overlay) so a viewer can watch the map grow.

**Architecture:** A new module `m3_adapter/obsmap_export.py` converts an ObsMap occupancy mask into a geometric .vxw, staying cleanly separate from obsmap.py (no vxw dependency there). The `uhumans2_stream.py` driver gains a `--live` flag that calls obsmap_export then remaps the observed-free grid to match the geometric .vxw's densify_occupancy grid (pad=1), then runs the full GVD pipeline in-process per batch. The alignment remap is the critical correctness piece: the occupancy_to_vxw output covers only True cells (a subset of the obsmap grid), so densify_occupancy pads around that tight bbox — producing a different vmin/shape than the obsmap — and we must resample the obsmap's observed_free onto that new grid before calling the pipeline.

**Tech Stack:** Python 3.11, numpy, vxw_format, m3_adapter.gvd.field.densify_occupancy, m3_adapter.gvd.pipeline.GvdConfig/run, pixi, pytest

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `m3_adapter/obsmap_export.py` | Create | Pure function: occupancy bool grid → geometric .vxw |
| `m3_adapter/uhumans2_stream.py` | Modify | Add `--live`, `--batch`, `--out-vxw` args + per-batch snapshot logic |
| `tests/test_obsmap_export.py` | Create | Roundtrip test for occupancy_to_vxw |

---

### Task 1: Create `m3_adapter/obsmap_export.py`

**Files:**
- Create: `m3_adapter/obsmap_export.py`
- Test: `tests/test_obsmap_export.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_obsmap_export.py
import numpy as np
import vxw_format as vxw
from m3_adapter.obsmap_export import occupancy_to_vxw


def test_occupancy_to_vxw_roundtrip(tmp_path):
    occ = np.zeros((40, 40, 40), dtype=bool)
    occ[10:15, 10:15, 10:15] = True   # 125 cells
    out = tmp_path / "g.vxw"
    occupancy_to_vxw(occ, np.array([-5, -5, -5], np.int64), 0.1, out)
    w = vxw.read_world(out)
    total = sum(int((c.voxels["material_id"] != 0).sum()) for c in w.chunks.values())
    assert total == 125
    assert w.manifest.voxel_size_meters == 0.1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pixi run pytest tests/test_obsmap_export.py -q`
Expected: FAIL with ImportError or ModuleNotFoundError

- [ ] **Step 3: Write `m3_adapter/obsmap_export.py`**

```python
"""obsmap_export.py — derive a renderable geometric .vxw from an ObsMap's
occupancy (one concrete material). Bridges the log-odds map to the .vxw format
the Godot viewer / GVD pipeline read. (SP-B v1.)"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vxw_format as vxw  # noqa: E402
from m3_adapter.common import build_concrete_palette  # noqa: E402


def occupancy_to_vxw(occ_mask, vmin, voxel_size, out_path, chunk_extent=32):
    """Write a geometric .vxw: every True cell in occ_mask becomes a concrete
    voxel (material_id=1, semantic_id=1). vmin = world-voxel coord of mask[0,0,0]."""
    coords = np.argwhere(occ_mask).astype(np.int64) + np.asarray(vmin, dtype=np.int64)
    if len(coords) == 0:
        raise ValueError("occupancy mask is empty")
    cc = np.floor_divide(coords, chunk_extent)
    local = (coords - cc * chunk_extent).astype(np.uint8)
    chunks = {}
    for ck in np.unique(cc, axis=0):
        m = np.all(cc == ck, axis=1)
        arr = np.zeros((chunk_extent,) * 3, dtype=vxw.VOXEL_DTYPE)
        loc = local[m]
        arr["material_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 1
        arr["semantic_id"][loc[:, 0], loc[:, 1], loc[:, 2]] = 1
        ckey = tuple(int(x) for x in ck)
        chunks[ckey] = vxw.Chunk(coord=ckey, voxels=arr,
                                 encoding=vxw.Encoding.RLE, compression=vxw.Compression.GZIP)
    keys = np.array(list(chunks.keys()))
    man = vxw.Manifest(
        world_id="obsmap_live", voxel_size_meters=float(voxel_size), chunk_extent=chunk_extent,
        bounds_chunks_min=tuple(int(x) for x in keys.min(axis=0)),
        bounds_chunks_max=tuple(int(x) + 1 for x in keys.max(axis=0)))
    vxw.write_world(Path(out_path), vxw.World(manifest=man, palette=build_concrete_palette(), chunks=chunks))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pixi run pytest tests/test_obsmap_export.py -q`
Expected: 1 passed

- [ ] **Step 5: Run full suite**

Run: `pixi run pytest tests/ -q`
Expected: all pass

---

### Task 2: Add `--live` mode to `uhumans2_stream.py`

**Files:**
- Modify: `m3_adapter/uhumans2_stream.py`

The key correctness challenge: `occupancy_to_vxw` only includes True cells (a subset of the obsmap grid). When the GVD pipeline calls `densify_occupancy(world, pad=1)` on that geometric .vxw, it computes its own `vmin2`/`shape2` from the tight bbox of those True cells ± pad=1. This differs from `obs.vmin`/`obs.logodds.shape`. We must resample the obsmap's `observed_free_mask()` onto this new grid before saving the `observed_free.npz` the pipeline will load.

Remap algorithm:
1. Write temp geometric .vxw from `obs.occupancy_mask()`.
2. Read it back, call `densify_occupancy(world2, pad=1)` → `(occ2, vmin2)`.
3. Compute world coords of the obsmap's free cells: `of_world = np.argwhere(obs.observed_free_mask()) + obs.vmin`.
4. Map into the new grid: `of_idx2 = of_world - vmin2`.
5. Clip to in-bounds, set `of2[of_idx2[inb, 0], of_idx2[inb, 1], of_idx2[inb, 2]] = True`.
6. Save `of2` with `vmin2` — this aligns exactly with what `field.load_observed_free` will compare against.

- [ ] **Step 1: Add args to argparse in `main()`**

After `ap.add_argument("--free-margin", ...)` and before `args = ap.parse_args()`, add:

```python
    ap.add_argument("--live", action="store_true",
                    help="write a renderable .vxw after each --batch frames")
    ap.add_argument("--batch", type=int, default=50,
                    help="frames per live snapshot batch (default 50)")
    ap.add_argument("--out-vxw", type=str, default=None,
                    help="output .vxw for live snapshots (default: <vxw_dir>/live.vxw)")
```

- [ ] **Step 2: Add imports at top of file (after existing imports)**

After `from m3_adapter.obsmap import ObsMap  # noqa: E402`, add:

```python
from m3_adapter.gvd.field import densify_occupancy  # noqa: E402  (already imported above)
```

Note: `densify_occupancy` is already imported. We need to add:

```python
from m3_adapter.gvd.pipeline import GvdConfig, run as gvd_run  # noqa: E402
from m3_adapter.obsmap_export import occupancy_to_vxw  # noqa: E402
```

- [ ] **Step 3: Add `_live_snapshot` helper function before `main()`**

```python
def _live_snapshot(obs, batch_idx, frames_so_far, out_vxw: Path, vxw_dir: Path) -> None:
    """Write a renderable GVD .vxw from the current ObsMap state.

    Grid alignment: occupancy_to_vxw covers only the occupied subset of the obsmap
    grid. densify_occupancy(pad=1) inside the pipeline re-derives its own vmin/shape
    from that tight bbox. We resample observed_free onto that derived grid so shapes
    and vmins align exactly when the pipeline calls load_observed_free.
    """
    import tempfile
    temp_dir = vxw_dir
    temp_geom = temp_dir / "_live_geom.vxw"

    occ_mask = obs.occupancy_mask()
    occ_count = int(occ_mask.sum())
    free_count = int(obs.observed_free_mask().sum())

    if occ_count == 0:
        log.info("  [live] batch=%d  fi=%d  occ=0 — skipping (no occupied voxels yet)",
                 batch_idx, frames_so_far)
        return

    # 1. Write temp geometric .vxw
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
    temp_free = temp_dir / "_live_observed_free.npz"
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
```

- [ ] **Step 4: Wire snapshot into the frame loop**

After `obs.integrate_frame(O_m, P_m, free_margin_m=args.free_margin)` and `frames_integrated += 1`, replace the existing `if frames_integrated % 50 == 0:` block with:

```python
            if frames_integrated % 50 == 0:
                occ_now = int(obs.occupancy_mask().sum())
                free_now = int(obs.observed_free_mask().sum())
                log.info("  fi=%d  integrated=%d skipped=%d  occ=%d free=%d",
                         fi_current, frames_integrated, frames_skipped,
                         occ_now, free_now)

            if args.live and frames_integrated % args.batch == 0:
                _live_snapshot(obs, frames_integrated // args.batch,
                               frames_integrated, live_out_vxw, args.vxw_dir)
```

- [ ] **Step 5: Set `live_out_vxw` before the frame loop**

After `args = ap.parse_args()` and before the `S = args.start_frame` line, add:

```python
    live_out_vxw = Path(args.out_vxw) if args.out_vxw else None
```

Then, after `obs = ObsMap.new(shape, vmin, voxel_size)` block (after `[2/5]`), before `[3/5]`, add:

```python
    if args.live and live_out_vxw is None:
        live_out_vxw = args.vxw_dir / "live.vxw"
```

- [ ] **Step 6: Verify `--help` shows new flags**

Run: `pixi run python m3_adapter/uhumans2_stream.py --help`
Expected output includes: `--live`, `--batch`, `--out-vxw`

- [ ] **Step 7: Run full test suite**

Run: `pixi run pytest tests/ -q`
Expected: all pass

---

### Task 3: Commit

- [ ] **Step 1: Stage and commit**

```bash
git add m3_adapter/obsmap_export.py m3_adapter/uhumans2_stream.py tests/test_obsmap_export.py
git commit -m "feat(map): live build loop — obsmap->vxw export + stream --live (SP-B v1a)"
```
