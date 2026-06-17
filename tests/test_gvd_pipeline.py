import json
import numpy as np
import vxw_format as vxw
from m3_adapter.common import build_concrete_palette
from m3_adapter.gvd.pipeline import GvdConfig, run


def _make_box_vxw(tmp_path, extent=32, vsize=0.5):
    pal = build_concrete_palette()
    arr = np.zeros((extent,) * 3, dtype=vxw.VOXEL_DTYPE)
    occ = np.zeros((21, 21, 21), dtype=bool)
    occ[0] = occ[20] = True
    occ[:, 0] = occ[:, 20] = True
    occ[:, :, 0] = occ[:, :, 20] = True
    ii = np.argwhere(occ)
    arr["material_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    arr["semantic_id"][ii[:, 0], ii[:, 1], ii[:, 2]] = 1
    man = vxw.Manifest(world_id="t", voxel_size_meters=vsize, chunk_extent=extent,
                       bounds_chunks_min=(0, 0, 0), bounds_chunks_max=(1, 1, 1))
    w = vxw.World(manifest=man, palette=pal,
                  chunks={(0, 0, 0): vxw.Chunk(coord=(0, 0, 0), voxels=arr)})
    p = tmp_path / "box.vxw"
    vxw.write_world(p, w)
    return p


def test_pipeline_field_only(tmp_path):
    src = _make_box_vxw(tmp_path)
    out = tmp_path / "o.vxw"
    stats = run(GvdConfig(str(src), str(out), band_max=1.0, thin=True))
    assert stats["gvd_voxels"] > 0
    w = vxw.read_world(out)
    assert "gvd_skeleton" in [m.name for m in w.palette.materials]


def test_pipeline_graph_rooms_cleanup(tmp_path):
    src = _make_box_vxw(tmp_path)
    out = tmp_path / "o.vxw"
    stats = run(GvdConfig(str(src), str(out), band_max=1.0, thin=True,
                          rooms=True, prune_spurs_m=0.3, merge_close_m=0.2,
                          room_resolution=1.0))
    assert stats["graph_nodes"] >= 1
    assert stats["num_rooms"] >= 1
    assert stats["graph_nodes"] <= stats["graph_nodes_raw"]  # cleanup didn't grow it
    g = json.loads((tmp_path / "o.graph.json").read_text())
    assert "num_rooms" in g and all("room" in n for n in g["nodes"])
    w = vxw.read_world(out)
    assert any(m.name.startswith("room_") for m in w.palette.materials)


def test_pipeline_observed_free_source(tmp_path):
    import numpy as np
    import vxw_format as vxw
    from m3_adapter.gvd.field import densify_occupancy
    from m3_adapter.gvd.pipeline import GvdConfig, run
    # reuse the module's _make_box_vxw
    src = _make_box_vxw(tmp_path)
    world = vxw.read_world(src)
    occ, vmin = densify_occupancy(world, pad=1)
    # a mask that marks only a small interior region as observed-free
    mask = np.zeros(occ.shape, dtype=bool)
    mask[8:13, 8:13, 8:13] = True
    np.savez_compressed(tmp_path / "of.npz", mask=mask, vmin=np.asarray(vmin), voxel_size=0.5)
    out = tmp_path / "o.vxw"
    stats = run(GvdConfig(str(src), str(out), band_max=None, thin=True,
                          observed_free_path=str(tmp_path / "of.npz")))
    assert stats["observed_free"] is True
    # free fraction reflects the small mask, NOT a flood
    assert stats["free_fraction"] <= float(mask.sum()) / mask.size + 1e-9
    # mismatched grid raises
    bad = tmp_path / "bad.npz"
    np.savez_compressed(bad, mask=np.zeros((3,3,3), bool), vmin=np.zeros(3, np.int64), voxel_size=0.5)
    try:
        run(GvdConfig(str(src), str(tmp_path/"o2.vxw"), observed_free_path=str(bad)))
        assert False, "expected grid-mismatch ValueError"
    except ValueError:
        pass
