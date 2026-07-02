"""clustering.py — sklearn 无关的精确 DBSCAN(scipy cKDTree + 并查集)。

背景:本机两套 python 环境的 sklearn 均不可用(anaconda:numpy ABI 导入错;
pixi:fit_predict 原生崩溃),而仓库对 sklearn 的唯一用途就是 DBSCAN。此实现
语义与 sklearn.DBSCAN.fit_predict 一致:
  core   = eps 邻域内(含自身)点数 >= min_samples;
  簇     = core 点在 eps 图上的连通分量,按首个 core 点出现次序编号 0..k-1;
  border = 非 core 但 eps 内有 core → 并入其 core 邻居的簇(取最小簇号,确定性);
  noise  = -1。
min_samples=1 时退化为纯 eps-连通分量(gvd/graph 的用法),无 noise。
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def _find(parent, i):
    root = i
    while parent[root] != root:
        root = parent[root]
    while parent[i] != root:                 # path compression
        parent[i], i = root, parent[i]
    return root


def dbscan_labels(points, eps, min_samples):
    """返回与 sklearn DBSCAN(eps, min_samples).fit_predict(points) 同语义的标签。"""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts[:, None]
    n = len(pts)
    if n == 0:
        return np.empty(0, dtype=np.int64)
    tree = cKDTree(pts)
    counts = tree.query_ball_point(pts, r=eps, return_length=True)
    core = np.asarray(counts) >= min_samples
    labels = np.full(n, -1, dtype=np.int64)
    if not core.any():
        return labels

    pairs = tree.query_pairs(r=eps, output_type="ndarray")
    parent = np.arange(n)
    if len(pairs):
        both_core = core[pairs[:, 0]] & core[pairs[:, 1]]
        for i, j in pairs[both_core]:
            ri, rj = _find(parent, int(i)), _find(parent, int(j))
            if ri != rj:
                parent[max(ri, rj)] = min(ri, rj)

    # 簇编号:按 core 点扫描序首次出现其根时分配(与 sklearn 编号习惯一致)
    root_to_label: dict = {}
    for i in np.flatnonzero(core):
        r = _find(parent, int(i))
        if r not in root_to_label:
            root_to_label[r] = len(root_to_label)
        labels[i] = root_to_label[r]

    # border:非 core 但 eps 内有 core → 取其 core 邻居的最小簇号(确定性 tie-break)
    if len(pairs):
        a, b = pairs[:, 0], pairs[:, 1]
        for p, q in ((a, b), (b, a)):
            m = ~core[p] & core[q]
            for i, j in zip(p[m], q[m]):
                lj = labels[j]
                if labels[i] == -1 or lj < labels[i]:
                    labels[i] = lj
    return labels
