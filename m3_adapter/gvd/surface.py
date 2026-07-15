# -*- coding: utf-8 -*-
"""surface.py — 2D 地面可通行层(surface places)。

纯函数, 与 3D GVD / 物体层解耦。设计: 输入一张 2D 可通行 bool 栅格(地面投影),
聚类成 surface place 区域(连通分量), 返回区域 + 门口相邻边。nav_frontend(每帧
局部反应式)与后端(累积全局)共用同一份聚类算法 —— 只是喂的栅格 scope 不同。

坐标: 全部在导航平面(ROS map/odom 系, z-up 的水平面 x-y)。不涉及 vxw y-up。
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage


def cluster_surface_places(walkable, origin_xy, res, min_cells=15,
                           door_bridge_m=0.9, blocked=None):
    """把 2D 可通行 bool 栅格聚成 surface place 区域。

    Args:
        walkable: (ny, nx) bool, True = 可走地面格。
        origin_xy: (x0, y0) 栅格 (0,0) 格中心对应的世界米坐标原点(左下角)。
        res: 米/格。
        min_cells: 小于此格数的区域丢弃(碎片)。
        door_bridge_m: 判定两区域"经门相邻"的桥接距离(米)。
        blocked: (ny,nx) bool 障碍格(墙/家具)。给定时, 相邻边只能穿"非障碍"
                 空间(门), 不能穿墙 —— 用测地膨胀实现。

    Returns:
        regions: list[dict] {id, centroid_xy, area_m2, cell_count, label_id}
        edges:   list[(i, j)] 区域间经门相邻(绝不穿墙)
        label_img: (ny,nx) int, 每格的区域 label(0=非可走/被丢弃)
    """
    lab, n = ndimage.label(walkable, structure=np.ones((3, 3), np.uint8))
    regions = []
    for r in range(1, n + 1):
        cells = np.argwhere(lab == r)          # (k, 2) = (row=y, col=x)
        if len(cells) < min_cells:
            lab[lab == r] = 0                  # 丢弃碎片
            continue
        cen = cells.mean(0)                    # (cy, cx) in cells
        wx = origin_xy[0] + (cen[1] + 0.5) * res
        wy = origin_xy[1] + (cen[0] + 0.5) * res
        regions.append({
            "id": len(regions),
            "centroid_xy": (float(wx), float(wy)),
            "area_m2": float(len(cells) * res * res),
            "cell_count": int(len(cells)),
            "label_id": int(r),
        })

    # 门口相邻: 从每个区域"测地膨胀"(只在非障碍格里扩散), 若够到另一区域 => 经门连通。
    # 遇墙(blocked)即止 -> 隔墙的两区域不会连出穿墙边; 只有真门(free 缝)能连通。
    edges = []
    bridge = max(int(round(door_bridge_m / res)), 1)
    passable = np.ones(walkable.shape, bool) if blocked is None else ~blocked
    struct = ndimage.generate_binary_structure(2, 2)
    masks = {}
    for reg in regions:
        m = (lab == reg["label_id"])
        for _ in range(bridge):
            m = ndimage.binary_dilation(m, struct) & passable
        masks[reg["id"]] = m
    ids = [reg["id"] for reg in regions]
    for ai in range(len(ids)):
        for bi in range(ai + 1, len(ids)):
            a, b = ids[ai], ids[bi]
            if (masks[a] & (lab == regions[bi]["label_id"])).any():
                edges.append((a, b))
    return regions, edges, lab


def occupancy_from_counts(surf_hits, obst_hits, surf_min=1, obst_min=2):
    """counts -> (walkable bool, costmap int8)。
    costmap: -1 未知 / 0 可走 / 100 障碍(nav_msgs/OccupancyGrid 约定)。"""
    walkable = (surf_hits >= surf_min) & (obst_hits < obst_min)
    blocked = obst_hits >= obst_min
    cost = np.full(surf_hits.shape, -1, dtype=np.int8)
    cost[walkable] = 0
    cost[blocked] = 100
    return walkable, cost
