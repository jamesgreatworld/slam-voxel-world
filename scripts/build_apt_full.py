"""build_apt_full.py — 公寓完整先验层构建(可复现入口,逻辑全在库里)。

四阶段流程(docs/PROGRESS.md):
  ③ vlayer.objects.decouple_objects   — 物体按标签从结构剔除(不粘连)
  ① SlabFill/WallFill/StairsFill/OcclusionFill — 结构平面拟合 + 实心化
  ② OpeningCarve — 门窗开口矩形/拱形拟合后规整挖空
  ④ vlayer.objects.extract_object_models — 物体 → mc_item 模型 entities.json

用法:  python scripts/build_apt_full.py [out_dir]
"""
from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import numpy as np

from m3_adapter.obsmap import ObsMap
from m3_adapter.obsmap_export import SUPER_ID_TO_MC_ITEM
from m3_adapter.uhumans2_to_vxw import (
    _resolve_hydra_paths, load_label_space, build_palette,
)
from m3_adapter.vlayer.export import obsmap_to_completed_vxw
from m3_adapter.vlayer.objects import decouple_objects, extract_object_models
from m3_adapter.vlayer.generators.slab import SlabFill
from m3_adapter.vlayer.generators.wall import WallFill
from m3_adapter.vlayer.generators.stairs import StairsFill
from m3_adapter.vlayer.generators.occlusion import OcclusionFill
from m3_adapter.vlayer.generators.opening import OpeningCarve
from m3_adapter.vlayer.generators.regularize import PlaneRegularize

# 带 RGB 通道的重建图优先(--rgb 全量重跑的产物);没有时退回旧几何图。
OBSMAP_RGB = "out/uhumans2_apt_rgb.vxw/obsmap.npz"
OBSMAP = "out/uhumans2_apt_full.vxw/obsmap.npz"
HYDRA = "F:/hydra_ws"
SCENE = "apartment"


def main(out_dir: str = "out/apt_full.vxw") -> None:
    src = OBSMAP_RGB if pathlib.Path(OBSMAP_RGB).exists() else OBSMAP
    print("[full] obsmap source: %s" % src)
    obsmap = ObsMap.load(src)
    yaml_path, _ = _resolve_hydra_paths(pathlib.Path(HYDRA), SCENE)
    label_names = load_label_space(yaml_path)
    palette = build_palette(label_names)

    # ③ 解耦:物体体素退出结构
    struct, obj_cells = decouple_objects(obsmap)
    print("[full] decoupled %d object voxels from structure" % int(obj_cells.sum()))

    # ①+② 结构补全(平面先验)+ 概率规整(形状先验×观测后验)+ 开口挖空 + 粗化去噪
    generators = [SlabFill(3, "floor"), SlabFill(4, "ceiling"),
                  WallFill(), StairsFill(), OcclusionFill(),
                  PlaneRegularize(), OpeningCarve()]
    obsmap_to_completed_vxw(
        struct, out_dir, generators=generators, palette=palette,
        coarsen_to_m=0.2, clean=True, clean_close_radius=0, clean_keep_largest=True,
    )

    # ④ 模型替换:物体 → mc_item / OBB 实体(带观测色、安装面、bbox 解算)。
    # 小物体类(books/vase)由点级通道实例化,体素通道只解耦、不再出实体,
    # 否则同一只花瓶会两个通道各一份(双重表示)。
    from m3_adapter.small_objects import SMALL_OBJECT_LABELS
    from m3_adapter.vlayer.objects import OBJECT_LABELS
    sp = pathlib.Path(src).parent / "small_objects.json"
    voxel_labels = OBJECT_LABELS - SMALL_OBJECT_LABELS if sp.exists() else OBJECT_LABELS
    # 预制模型的渲染尺寸表:摆放解算必须按渲染尺寸吸附/分离,否则模型沉地/互嵌
    presets = {p["id"]: p.get("overall_extents_m")
               for p in json.loads(pathlib.Path(
                   "m3_adapter/mc_item_pack/_compiled.json").read_text())["presets"]}
    entities = extract_object_models(
        obsmap.occupancy_mask(), obsmap.semantic_grid(),
        np.asarray(obsmap.vmin), obsmap.voxel_size,
        label_names, SUPER_ID_TO_MC_ITEM, min_voxels=30, labels=voxel_labels,
        rgb=obsmap.rgb, rgb_count=obsmap.rgb_count, preset_extents=presets,
        coarse_vs=0.2,
    )
    # 小物体点级实例(--small-objects 流式产物)并入,先做父子支撑吸附:
    # 支撑家具被解算挪动后,书要跟着落在新的家具顶面上,不许悬空。
    from m3_adapter.vlayer.objects import snap_small_to_support
    sp = pathlib.Path(src).parent / "small_objects.json"
    if sp.exists():
        small = json.loads(sp.read_text())["entities"]
        snap_small_to_support(small, entities, preset_extents=presets)
        entities.extend(small)
        print("[full] + %d small-object instances (support-snapped)" % len(small))
    ent_path = pathlib.Path(out_dir) / "entities.json"
    ent_path.write_text(json.dumps(
        {"format_version": "1.0", "entities": entities}, indent=2))
    print("[full] %d mc_item models -> %s" % (len(entities), ent_path))
    print("[full] done -> %s" % out_dir)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out/apt_full.vxw")
