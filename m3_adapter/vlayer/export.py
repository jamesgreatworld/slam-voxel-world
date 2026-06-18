"""把 ObsMap(L0)经 Lc 流水线补全后物化为可渲染 .vxw。
流程:run_pipeline -> compose_structure -> occupancy_to_vxw。
overlay 作为 sidecar(overlay.npz/json)持久化到世界目录。L0 只读。"""
from __future__ import annotations

from pathlib import Path

from m3_adapter.obsmap_export import occupancy_to_vxw
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.pipeline import run_pipeline


def obsmap_to_completed_vxw(obsmap, out_path, generators,
                            palette=None, chunk_extent=32):
    """运行补全流水线并导出 .vxw。返回产生的 Overlay。

    palette 为 None 时按非语义模式导出(全部 material_id=1);
    传入 vxw.Palette 则语义着色。"""
    out_path = Path(out_path)
    overlay = run_pipeline(obsmap, generators)
    occ, sem = compose_structure(obsmap, overlay)
    occupancy_to_vxw(
        occ, obsmap.vmin, obsmap.voxel_size, out_path,
        chunk_extent=chunk_extent,
        semantic_grid=(sem if palette is not None else None),
        palette=palette,
    )
    overlay.save(out_path / "overlay.npz", out_path / "overlay.json")
    return overlay
