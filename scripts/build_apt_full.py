"""build_apt_full.py — 公寓完整先验层构建(可复现入口,逻辑全在库里)。

四阶段流程(docs/PROGRESS.md):
  ③ vlayer.objects.decouple_objects   — 物体按标签从结构剔除(不粘连)
  ① SlabFill/WallFill/StairsFill/OcclusionFill — 结构平面拟合 + 实心化
  (② OpeningCarve 门窗形状挖空 — 待做)
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

OBSMAP = "out/uhumans2_apt_full.vxw/obsmap.npz"
HYDRA = "F:/hydra_ws"
SCENE = "apartment"


def main(out_dir: str = "out/apt_full.vxw") -> None:
    obsmap = ObsMap.load(OBSMAP)
    yaml_path, _ = _resolve_hydra_paths(pathlib.Path(HYDRA), SCENE)
    label_names = load_label_space(yaml_path)
    palette = build_palette(label_names)

    # ③ 解耦:物体体素退出结构
    struct, obj_cells = decouple_objects(obsmap)
    print("[full] decoupled %d object voxels from structure" % int(obj_cells.sum()))

    # ① 结构补全(平面先验)+ 粗化 + 去噪
    generators = [SlabFill(3, "floor"), SlabFill(4, "ceiling"),
                  WallFill(), StairsFill(), OcclusionFill()]
    obsmap_to_completed_vxw(
        struct, out_dir, generators=generators, palette=palette,
        coarsen_to_m=0.2, clean=True, clean_close_radius=0, clean_keep_largest=True,
    )

    # ④ 模型替换:物体 → mc_item 实体
    entities = extract_object_models(
        obsmap.occupancy_mask(), obsmap.semantic_grid(),
        np.asarray(obsmap.vmin), obsmap.voxel_size,
        label_names, SUPER_ID_TO_MC_ITEM, min_voxels=80,
    )
    ent_path = pathlib.Path(out_dir) / "entities.json"
    ent_path.write_text(json.dumps(
        {"format_version": "1.0", "entities": entities}, indent=2))
    print("[full] %d mc_item models -> %s" % (len(entities), ent_path))
    print("[full] done -> %s" % out_dir)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out/apt_full.vxw")
