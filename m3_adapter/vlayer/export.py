"""把 ObsMap(L0)经 Lc 流水线补全后物化为可渲染 .vxw。
流程:run_pipeline -> compose_structure -> occupancy_to_vxw。
overlay 作为 sidecar(overlay.npz/json)持久化到世界目录。L0 只读。"""
from __future__ import annotations

from pathlib import Path

from m3_adapter.obsmap_export import occupancy_to_vxw
from m3_adapter.vlayer.coarsen import downsample_occupancy
from m3_adapter.vlayer.overlay import compose_structure
from m3_adapter.vlayer.pipeline import run_pipeline


def obsmap_to_completed_vxw(obsmap, out_path, generators,
                            palette=None, chunk_extent=32, coarsen_to_m=None,
                            clean=False, clean_min_component=2, clean_close_radius=1,
                            clean_keep_largest=False, drop_unlabelled=False):
    """运行补全流水线并导出 .vxw。返回产生的 Overlay。

    palette 为 None 时按非语义模式导出(全部 material_id=1);
    传入 vxw.Palette 则语义着色。
    coarsen_to_m: 目标体素尺寸(米);非 None 时按 round(coarsen_to_m/vs) 降采样。
    clean: 是否对(粗)网格做形态学补洞+小连通分量去噪;默认 False 保持原有行为。
    clean_min_component: 小于该体素数的连通分量被删除。
    clean_close_radius: 形态学 closing 迭代次数(半径),0 跳过 closing。"""
    out_path = Path(out_path)
    out_path.mkdir(parents=True, exist_ok=True)
    overlay = run_pipeline(obsmap, generators)
    occ, sem = compose_structure(obsmap, overlay)
    vmin = obsmap.vmin
    vs = obsmap.voxel_size
    if coarsen_to_m:
        factor = max(1, round(coarsen_to_m / vs))
        occ, sem, vmin, vs = downsample_occupancy(occ, sem, vmin, vs, factor)
    if drop_unlabelled:
        # 无语义(label 0)的占据格 = 只有几何、无类别归属的观测残渣,
        # 在语义着色的世界里是无名灰砖——按需剔除(在 clean 之前,
        # 剔除产生的新碎块交给 keep_largest 收拾)。
        occ = occ & (sem != 0)
    if clean:
        from m3_adapter.vlayer.coarsen import clean_coarse
        occ, sem = clean_coarse(occ, sem, clean_min_component, clean_close_radius,
                                keep_largest=clean_keep_largest)
    occupancy_to_vxw(
        occ, vmin, vs, out_path,
        chunk_extent=chunk_extent,
        semantic_grid=(sem if palette is not None else None),
        palette=palette,
    )
    overlay.save(out_path / "overlay.npz", out_path / "overlay.json")
    return overlay
