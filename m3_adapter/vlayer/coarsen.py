"""把细占据/语义网格降采样成粗块(Minecraft 厚块感)。L0 不变;导出后处理。"""
from __future__ import annotations

import numpy as np


def downsample_occupancy(occ, sem, vmin, vs, factor, min_fine: int = 1):
    """细 (occ,sem,vmin,vs) -> 粗 (occ_c,sem_c,vmin_c,vs_c)。
    粗块占据 = 块内占据细格数 >= min_fine;粗块语义 = 块内占据细格 super_id 的众数。
    每轴按 vmin%factor 对齐到 factor 边界后分块,保持 world 对齐。"""
    factor = int(factor)
    vmin = np.asarray(vmin, dtype=np.int64)
    if factor <= 1:
        return occ.copy(), sem.copy(), vmin.copy(), float(vs)
    pad = []
    vmin_c = np.empty(3, np.int64)
    for a in range(3):
        lo = int(vmin[a]) % factor                       # python mod -> 0..factor-1, aligns boundary
        sz = occ.shape[a] + lo
        hi = (-sz) % factor
        pad.append((lo, hi))
        vmin_c[a] = (int(vmin[a]) - lo) // factor
    occ_p = np.pad(occ, pad, mode="constant", constant_values=False)
    sem_p = np.pad(sem, pad, mode="constant", constant_values=0)
    f = factor
    NX, NY, NZ = (np.array(occ_p.shape) // f)
    occ_b = occ_p.reshape(NX, f, NY, f, NZ, f)
    cnt = occ_b.sum(axis=(1, 3, 5))
    occ_c = cnt >= min_fine
    # semantic majority among OCCUPIED fine cells, per block
    sem_b = sem_p.reshape(NX, f, NY, f, NZ, f)
    flat = np.where(occ_b, sem_b, 0).astype(np.int64).transpose(0, 2, 4, 1, 3, 5).reshape(NX, NY, NZ, -1)
    best_cnt = np.zeros((NX, NY, NZ), np.int32)
    best_lab = np.zeros((NX, NY, NZ), np.uint8)
    maxlabel = int(flat.max()) if flat.size else 0
    for L in range(1, maxlabel + 1):
        c = (flat == L).sum(axis=-1).astype(np.int32)
        upd = c > best_cnt
        best_cnt = np.where(upd, c, best_cnt)
        best_lab = np.where(upd, np.uint8(L), best_lab)
    return occ_c, best_lab, vmin_c, float(vs * factor)


from scipy import ndimage


def prune_dangles(occ, sem, iterations: int = 2):
    """去"钟乳石":迭代移除 6-邻域支撑 ≤1 的占据格(单格尖刺/一格细链的
    末端)。keep_largest 只能删断开的孤块;贴着主体的细悬突要靠这个。
    不修改输入。"""
    occ = occ.copy(); sem = sem.copy()
    k = np.zeros((3, 3, 3))
    k[1, 1, 0] = k[1, 1, 2] = k[1, 0, 1] = k[1, 2, 1] = k[0, 1, 1] = k[2, 1, 1] = 1
    for _ in range(int(iterations)):
        nb = ndimage.convolve(occ.astype(np.uint8), k, mode="constant")
        dangle = occ & (nb <= 1)
        if not dangle.any():
            break
        occ &= ~dangle
        sem = np.where(dangle, np.uint8(0), sem)
    return occ, sem


def clean_coarse(occ, sem, min_component: int = 2, close_radius: int = 1,
                 keep_largest: bool = False):
    """Denoise/heal a coarse occupancy: morphological-close small holes (new cells
    inherit nearest occupied cell's sem) and drop connected components smaller than
    min_component voxels. Returns (occ_clean, sem_clean). Does not mutate inputs.

    keep_largest: if True, keep ONLY the largest connected component and drop every
    other blob (floating sensor specks, detached debris) regardless of their size.
    Right for a scan of a single dominant structure (e.g. one building); leave
    False when genuinely-detached objects should survive."""
    occ = occ.copy(); sem = sem.copy()
    st = ndimage.generate_binary_structure(3, 1)
    if close_radius > 0:
        closed = ndimage.binary_closing(occ, structure=st, iterations=int(close_radius))
        new = closed & ~occ
        if new.any():
            # newly-filled cells take the sem of the nearest originally-occupied cell
            inds = ndimage.distance_transform_edt(~occ, return_distances=False, return_indices=True)
            sem_near = sem[tuple(inds)]
            sem = np.where(new, sem_near, sem)
            occ = closed
    if min_component and min_component > 1:
        lbl, n = ndimage.label(occ, structure=st)
        if n > 0:
            sizes = np.bincount(lbl.ravel())
            drop_ids = np.where(sizes[1:] < int(min_component))[0] + 1
            if len(drop_ids):
                drop = np.isin(lbl, drop_ids)
                occ = occ & ~drop
                sem = np.where(drop, np.uint8(0), sem)
    if keep_largest:
        lbl, n = ndimage.label(occ, structure=st)
        if n > 1:
            sizes = np.bincount(lbl.ravel()); sizes[0] = 0
            keep = int(sizes.argmax())
            drop = (lbl != keep) & (lbl != 0)
            occ = occ & ~drop
            sem = np.where(drop, np.uint8(0), sem)
    return occ, sem
