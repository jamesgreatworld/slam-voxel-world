"""small_objects.py — 小物体的点级实例通道(水杯/钥匙/花瓶量级)。

问题:5cm 栅格装不下一只 8cm 的杯子(1~2 格,低于任何噪声阈),更别提钥匙
(亚体素);全局加密到 1cm 代价是体素数 ×125,不可行。

解法:小物体**绕过栅格**,走稀疏实例通道——
    流式取帧时,语义属于小物体类的命中点(带观测色)直接进点缓冲
    → finalize 时 1cm 量化去重 + DBSCAN(米制,scipy 实现)聚类
    → 每簇一个实例:AABB 中心/尺寸(可小于一个体素!)+ 观测色均值
    → 写成与家具同构的 entity dict → entities.json → 显示层 OBB 彩盒。

逻辑定位:这是 L0 观测层的**第二条通道**(稠密栅格之外的稀疏实例观测);
先验层无需再解耦(它们从未进栅格),直接实例化;显示层复用实体渲染。
"""
from __future__ import annotations

import hashlib

import numpy as np

from m3_adapter.clustering import dbscan_labels

# uhumans2 apartment:books(2)/vase(6) 是杯壶钥匙量级的小物类。可按数据集配置。
SMALL_OBJECT_LABELS: frozenset = frozenset({2, 6})

_QUANT_M = 0.01          # 累积点 1cm 量化去重(多帧重复观测不膨胀)


def _uuid(seed: str) -> str:
    h = hashlib.md5(seed.encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


class SmallObjectBuffer:
    def __init__(self, labels=SMALL_OBJECT_LABELS):
        self.labels = frozenset(int(x) for x in labels)
        self._pts: dict = {L: [] for L in self.labels}     # label -> [(N,3) m]
        self._cols: dict = {L: [] for L in self.labels}    # label -> [(N,3) u8|None]

    def add_frame(self, points_m, point_labels, point_colors=None) -> int:
        """缓冲本帧中语义属于小物体类的命中点。返回缓冲的点数。"""
        pts = np.asarray(points_m, dtype=np.float64)
        labs = np.asarray(point_labels)
        total = 0
        for L in self.labels:
            m = labs == L
            if not m.any():
                continue
            self._pts[L].append(pts[m])
            self._cols[L].append(
                np.asarray(point_colors)[m].astype(np.uint8)
                if point_colors is not None else None)
            total += int(m.sum())
        return total

    def finalize(self, label_names, eps_m=0.06, min_points=30, min_dim_m=0.03):
        """聚类 → 每簇一个实例 dict(与 entities.json 记录同构)。"""
        ents = []
        for L in sorted(self.labels):
            if not self._pts[L]:
                continue
            pts = np.concatenate(self._pts[L], axis=0)
            cols = None
            col_parts = self._cols[L]
            if all(c is not None for c in col_parts):
                cols = np.concatenate(col_parts, axis=0)
            # 1cm 量化去重:多帧重复观测收敛成稳定点集(颜色取首见)
            q = np.round(pts / _QUANT_M).astype(np.int64)
            _, first = np.unique(q, axis=0, return_index=True)
            pts = pts[first]
            if cols is not None:
                cols = cols[first]
            if len(pts) < min_points:
                continue
            lab = dbscan_labels(pts, eps_m, min_points)
            for cid in range(int(lab.max()) + 1 if lab.size else 0):
                m = lab == cid
                p = pts[m]
                lo, hi = p.min(axis=0), p.max(axis=0)
                centre = (lo + hi) / 2.0
                dims = np.maximum(hi - lo, min_dim_m)
                meta = {"small": True}
                if cols is not None:
                    meta["rgb"] = [int(round(v)) for v in cols[m].mean(axis=0)]
                ents.append({
                    "id": _uuid("small-%d-%d-%.3f-%.3f" % (L, cid, centre[0], centre[2])),
                    "label": int(L),
                    "label_name": label_names.get(int(L), str(L)),
                    "position": [float(v) for v in centre],
                    "rotation": [0.0, 0.0, 0.0, 1.0],
                    "bbox_dims": [float(v) for v in dims],
                    "voxel_count": int(m.sum()),
                    "custom_meta": meta,
                })
        return ents
