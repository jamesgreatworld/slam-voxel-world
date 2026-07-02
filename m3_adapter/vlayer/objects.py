"""vlayer.objects — 先验层的物体解耦(stage ③)与模型替换(stage ④)。

流程定位(docs/PROGRESS.md §先验层四阶段):
  ③ 解耦:按语义标签把物体体素从结构中剔除(墙/地永不与家具粘连)。
    解耦不需要聚类 —— 标签掩码是瞬时的;聚类只服务于 ④ 的模型实例化。
  ④ 模型:对每个物体标签做 scipy 连通分量聚类,一个分量 = 一个 mc_item
    模型实例(entities.json 记录)。不用 sklearn:DBSCAN 的 loky 后端在
    本机两套环境下都崩溃/死锁,而体素网格上 26-邻域连通分量本就是等价聚类。

产物实体是 entity_renderer.gd 消费的 dict(与 entities.json 记录同构),
直立 + 单位旋转:朝向交给 yaw-only 的 mc_item 直立摆放和鼠标旋转编辑。
"""
from __future__ import annotations

import copy
import hashlib

import numpy as np
from scipy import ndimage

# chair / couch / table / lamp / bed / computer / trashcan(与 uhumans2 label space 对齐)
OBJECT_LABELS: frozenset = frozenset({5, 7, 10, 11, 14, 16, 18})

_CC26 = np.ones((3, 3, 3), dtype=int)


def _stable_uuid(seed: str) -> str:
    h = hashlib.md5(seed.encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


def decouple_objects(obsmap, labels=OBJECT_LABELS):
    """返回 (structure_obsmap, obj_cells):物体格从结构中剔除后的 ObsMap 深拷贝
    与被剔除格的布尔掩码。剔除 = log-odds 归零(unknown,既非占据也非 free),
    语义清零;原 obsmap 不变(L0 只读原则)。"""
    occ = obsmap.occupancy_mask()
    sem = obsmap.semantic_grid()
    obj_cells = occ & np.isin(sem, list(labels))
    struct = copy.deepcopy(obsmap)
    struct.logodds[obj_cells] = 0.0
    struct.sem_label[obj_cells] = 0
    return struct, obj_cells


def extract_object_models(occ, sem, vmin, voxel_size, label_names,
                          mc_item_map, min_voxels=80, labels=OBJECT_LABELS):
    """每个物体标签的每个 26-连通分量 → 一个模型实体 dict。

    min_voxels 过滤语义噪声碎块(细 5cm 网格下一件真家具远大于该值)。
    只为 mc_item_map 覆盖的类别产实体;位置=AABB 中心(米),尺寸=AABB(米),
    旋转=单位四元数(直立,朝向留给编辑器)。"""
    ents = []
    vmin = np.asarray(vmin, dtype=np.int64)
    for lab in sorted(labels):
        preset = mc_item_map.get(lab)
        if preset is None:
            continue
        lbl_img, n = ndimage.label(occ & (sem == lab), structure=_CC26)
        for cid in range(1, n + 1):
            cells = np.argwhere(lbl_img == cid)
            if len(cells) < min_voxels:
                continue
            wc = cells + vmin
            cmin = wc.min(axis=0).astype(float)
            cmax = wc.max(axis=0).astype(float)
            centre = (cmin + cmax + 1.0) * 0.5 * voxel_size
            dims = (cmax - cmin + 1.0) * voxel_size
            ents.append({
                "id": _stable_uuid("%d-%d-%.3f-%.3f" % (lab, cid, centre[0], centre[2])),
                "label": int(lab),
                "label_name": label_names.get(int(lab), str(lab)),
                "position": [float(centre[0]), float(centre[1]), float(centre[2])],
                "rotation": [0.0, 0.0, 0.0, 1.0],
                "bbox_dims": [float(dims[0]), float(dims[1]), float(dims[2])],
                "voxel_count": int(len(cells)),
                "custom_meta": {"mc_item": preset},
            })
    return ents
