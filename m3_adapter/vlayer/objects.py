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

# 可模型化的物品类全集(uhumans2 apartment label space,21 类):除结构
# (floor 3 / ceiling 4 / stairs 15 / wall 19)与 unknown 0 以外的所有物品——
# appliance/books/chair/vase/couch/plant/furniture/computer/lamp/painting/
# plant/bed/table/screens/trashcan。有 mc_item 预制模型的用模型,其余 OBB 盒
# (带观测色)。"绝大部分物品都能模型化表示"。
OBJECT_LABELS: frozenset = frozenset({1, 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18})
# 动态物(human):从结构剔除,但永不实例化成模型——人不属于地图。
DYNAMIC_LABELS: frozenset = frozenset({20})

_CC26 = np.ones((3, 3, 3), dtype=int)


def _stable_uuid(seed: str) -> str:
    h = hashlib.md5(seed.encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


def decouple_cells(obsmap, mask):
    """按显式格掩码解耦(被实例化的簇 + 点级小物体类 + 动态物):
    掩码格 log-odds 归零(unknown)、语义清零;原 obsmap 不变。
    与 decouple_objects 的区别:未被实例化的超尺寸簇(藤蔓)留在结构里。"""
    struct = copy.deepcopy(obsmap)
    struct.logodds[mask] = 0.0
    struct.sem_label[mask] = 0
    return struct, mask


def decouple_objects(obsmap, labels=None):
    """返回 (structure_obsmap, obj_cells):物体格从结构中剔除后的 ObsMap 深拷贝
    与被剔除格的布尔掩码。剔除 = log-odds 归零(unknown,既非占据也非 free),
    语义清零;原 obsmap 不变(L0 只读原则)。默认剔除全部物品类 + 动态物(human)。"""
    if labels is None:
        labels = OBJECT_LABELS | DYNAMIC_LABELS
    occ = obsmap.occupancy_mask()
    sem = obsmap.semantic_grid()
    obj_cells = occ & np.isin(sem, list(labels))
    struct = copy.deepcopy(obsmap)
    struct.logodds[obj_cells] = 0.0
    struct.sem_label[obj_cells] = 0
    return struct, obj_cells


STRUCTURE_LABELS = (3, 4, 19)      # floor / ceiling / wall — 安装面/吸附判定用


def _merge_overlapping_components(comps, gap_vox=2):
    """同标签簇去碎:AABB 间空隙 ≤ gap_vox 格的簇合并成一件(过分割的桌面+桌腿、
    断成两截的床)。O(n²) 两两合并至收敛;n 是每类簇数,很小。"""
    merged = [np.asarray(c) for c in comps]
    changed = True
    while changed:
        changed = False
        out = []
        while merged:
            cur = merged.pop(0)
            lo1, hi1 = cur.min(axis=0), cur.max(axis=0)
            keep = []
            for other in merged:
                lo2, hi2 = other.min(axis=0), other.max(axis=0)
                # 空隙格数 = lo2 - hi1 - 1;≤ gap_vox 即算相邻(各轴同时满足)
                if (lo1 <= hi2 + gap_vox + 1).all() and (lo2 <= hi1 + gap_vox + 1).all():
                    cur = np.concatenate([cur, other]); changed = True
                    lo1, hi1 = cur.min(axis=0), cur.max(axis=0)
                else:
                    keep.append(other)
            merged = keep
            out.append(cur)
        merged = out
    # 稳定输出:按簇 AABB 最小角排序(与扫描序一致,调用方/测试可预期)
    merged.sort(key=lambda c: tuple(c.min(axis=0)))
    return merged


def _mount_of(cells, occ, sem, reach=8):
    """判断簇的安装方式(数据驱动,任何场景通用):
      ceiling — 上方 reach 格内贴结构、下方悬空(吸顶灯)
      wall    — 下方悬空、侧向 reach 格内贴墙(挂画/挂屏)
      floor   — 其余(家具站地上)。
    非 floor 的实例不做落地吸附、不套直立落地模型。"""
    struct = np.isin(sem, STRUCTURE_LABELS) & occ
    wall = occ & (sem == STRUCTURE_LABELS[2])
    nx, ny, nz = occ.shape
    top = int(cells[:, 1].max()); bot = int(cells[:, 1].min())
    cols_top = {(int(x), int(z)) for x, y, z in cells if y == top}
    cols_bot = {(int(x), int(z)) for x, y, z in cells if y == bot}
    above = any(struct[x, top + 1:min(ny, top + 1 + reach), z].any() for x, z in cols_top)
    below = any(struct[x, max(0, bot - reach):bot, z].any() for x, z in cols_bot)
    if above and not below:
        return "ceiling"
    if not below:
        # 侧向贴墙?任一簇格的 ±x/±z reach 邻域内有墙
        for x, y, z in cells[:: max(1, len(cells) // 64)]:
            if wall[max(0, x - reach):min(nx, x + reach + 1), y, z].any() or \
               wall[x, y, max(0, z - reach):min(nz, z + reach + 1)].any():
                return "wall"
    return "floor"


def extract_object_models(occ, sem, vmin, voxel_size, label_names,
                          mc_item_map, min_voxels=30, labels=OBJECT_LABELS,
                          rgb=None, rgb_count=None, preset_extents=None,
                          coarse_vs=None, max_extent_m=2.6, consumed_out=None):
    """每个物体标签的每个 26-连通分量 → 一个模型实体 dict。

    - 所有物体标签都产实体:有 mc_item 预制模型的用模型,没有的保留 OBB 盒
      (couch 等),渲染器按 bbox_dims + 颜色画盒。
    - min_voxels 过滤语义噪声碎块。
    - 吸顶判定:上贴结构、下不落地的簇标 custom_meta.mount='ceiling';吸顶灯
      不套落地灯模型(直立落地模型挂天上是错的),保留原位彩盒。
    - rgb/rgb_count(ObsMap 颜色通道)提供时,写 custom_meta.rgb=[r,g,b]
      (簇内观测色均值)——同类物体不再共用调色板一个色。
    位置=AABB 中心(米),尺寸=AABB(米),旋转=单位四元数(朝向留给编辑器)。"""
    ents = []
    vmin = np.asarray(vmin, dtype=np.int64)
    for lab in sorted(labels):
        preset = mc_item_map.get(lab)
        lbl_img, n = ndimage.label(occ & (sem == lab), structure=_CC26)
        comps = [np.argwhere(lbl_img == cid) for cid in range(1, n + 1)]
        comps = [c for c in comps if len(c) >= max(4, min_voxels // 4)]
        # 同类去碎:过分割碎片合并成一件,根治"桌子嵌桌子"
        comps = _merge_overlapping_components(comps)
        for cid, cells in enumerate(comps, start=1):
            if len(cells) < min_voxels:
                continue
            wc = cells + vmin
            cmin = wc.min(axis=0).astype(float)
            cmax = wc.max(axis=0).astype(float)
            centre = (cmin + cmax + 1.0) * 0.5 * voxel_size
            dims = (cmax - cmin + 1.0) * voxel_size
            # 模型尺度门控:超尺寸簇(爬满整面墙的藤蔓、屋顶植被)不是"一件
            # 物体",不实例化成盒——留在结构体素里(绿叶砖比灰巨盒诚实得多)。
            if float(dims.max()) > max_extent_m:
                continue
            if consumed_out is not None:
                consumed_out[cells[:, 0], cells[:, 1], cells[:, 2]] = True
            meta = {}
            mount = _mount_of(cells, occ, sem)
            if mount != "floor":
                meta["mount"] = mount          # ceiling / wall:保位置,不套落地模型
            if preset is not None and mount == "floor":
                meta["mc_item"] = preset
            if rgb is not None and rgb_count is not None:
                seen = rgb_count[cells[:, 0], cells[:, 1], cells[:, 2]] > 0
                if seen.any():
                    cc = cells[seen]
                    mean = rgb[cc[:, 0], cc[:, 1], cc[:, 2]].reshape(-1, 3).mean(axis=0)
                    meta["rgb"] = [int(round(v)) for v in mean]
            ents.append({
                "id": _stable_uuid("%d-%d-%.3f-%.3f" % (lab, cid, centre[0], centre[2])),
                "label": int(lab),
                "label_name": label_names.get(int(lab), str(lab)),
                "position": [float(centre[0]), float(centre[1]), float(centre[2])],
                "rotation": [0.0, 0.0, 0.0, 1.0],
                "bbox_dims": [float(dims[0]), float(dims[1]), float(dims[2])],
                "voxel_count": int(len(cells)),
                "custom_meta": meta,
            })
    resolve_placements(ents, occ, sem, vmin, voxel_size,
                       preset_extents=preset_extents, coarse_vs=coarse_vs)
    return ents


# ---------------------------------------------------------------------------
# 摆放解算(先验层物理一致性):落地/贴顶吸附、推出墙体、两两分离。
# 模型实例必须满足:不插进地板、不嵌进墙、互不重叠。数据驱动,任何场景通用。
# ---------------------------------------------------------------------------



def resolve_placements(ents, occ, sem, vmin, vs, max_push_m=0.5,
                       preset_extents=None, coarse_vs=None):
    """摆放解算必须用**渲染尺寸**(mc 预制模型的 overall_extents),而不是观测
    bbox——模型比观测簇高/宽时,按观测尺寸吸附会让模型下半截沉进地板(错位的
    主根源)。preset_extents: mc_item id -> [dx,dy,dz](米)。

    吸附细节(都是踩过的坑):
    · 参考高度用**中心 y**(不是渲染 bottom):模型初始已沉入地板时,自 bottom
      向下的搜索窗整个落在地板之下(未知),吸附不触发;
    · 只认 floor(3)/ceiling(4) 标签:外扩搜索里的墙顶会把"地板面"抬到墙高;
    · coarse_vs 给定时地板顶向上取整到粗格面:显示层画的是粗块,吸到观测细面
      会差最多一格(脚陷进粗砖里)。"""
    floor = occ & (sem == STRUCTURE_LABELS[0])
    ceil_ = occ & (sem == STRUCTURE_LABELS[1])
    wall = occ & (sem == STRUCTURE_LABELS[2])
    nx, ny, nz = occ.shape
    vmin = np.asarray(vmin, dtype=np.int64)
    preset_extents = preset_extents or {}

    def rdims(e):
        mc = e["custom_meta"].get("mc_item")
        ext = preset_extents.get(mc)
        return np.array(ext if ext else e["bbox_dims"], float)

    def to_cell(p_m):
        return np.floor(np.asarray(p_m) / vs).astype(np.int64) - vmin

    margin = int(round(0.4 / vs))       # 横向外扩:自身遮挡地板时用周边地板
    depth = int(round(3.0 / vs))        # 纵深:整层楼高内找支撑面

    # 观测几何存底:父子(支撑)判定必须依据**观测**位置——谁的观测底面贴着
    # 谁的观测顶面,谁就坐在谁上;吸附后再按父件的最终顶面落座。
    for e in ents:
        e["_obs_pos"] = np.array(e["position"], float)
        e["_obs_half"] = np.array(e["bbox_dims"], float) / 2.0

    def find_floor_top(pos, half):
        c = to_cell(pos)
        x0, x1 = max(0, int(c[0] - half[0] / vs) - margin), min(nx, int(c[0] + half[0] / vs) + 1 + margin)
        z0, z1 = max(0, int(c[2] - half[2] / vs) - margin), min(nz, int(c[2] + half[2] / vs) + 1 + margin)
        cy = int(np.clip(c[1], 0, ny - 1))
        lo = max(0, cy - depth)
        col = floor[x0:x1, lo:cy + 2, z0:z1]
        ys = np.where(col.any(axis=(0, 2)))[0]
        if ys.size:
            return (lo + int(ys[-1]) + 1 + vmin[1]) * vs
        # 地板标签缺失(地毯误标/未观测):footprint 内逐列"中心以下最高占据"
        # 的中位数——对墙/邻物污染鲁棒。
        sub = occ[x0:x1, :cy + 1, z0:z1]
        tops = []
        yy = np.arange(sub.shape[1])
        for ix in range(sub.shape[0]):
            for iz in range(sub.shape[2]):
                colm = yy[sub[ix, :, iz]]
                if colm.size:
                    tops.append(int(colm[-1]))
        if tops:
            return (int(np.median(tops)) + 1 + vmin[1]) * vs
        return None

    # --- 1. 支撑感知吸附:按观测底面从低到高放置;支撑候选 = 地板 ∪ 已放
    #        置家具的最终顶面(观测底面贴其观测顶面 ±0.3m 才算父子)。
    #        桌上的电脑/台灯从此坐在桌面上,而不是被地板吸附拽到地上。 ---
    placed = []
    for e in sorted(ents, key=lambda e: e["_obs_pos"][1] - e["_obs_half"][1]):
        meta = e["custom_meta"]
        pos = e["_obs_pos"].copy()
        half = rdims(e) / 2.0
        if meta.get("mount") == "wall":
            placed.append(e)
            continue          # 壁挂物本来就贴墙:不吸附、不推离
        if meta.get("mount") == "ceiling":
            c = to_cell(pos)
            x0, x1 = max(0, int(c[0] - half[0] / vs) - margin), min(nx, int(c[0] + half[0] / vs) + 1 + margin)
            z0, z1 = max(0, int(c[2] - half[2] / vs) - margin), min(nz, int(c[2] + half[2] / vs) + 1 + margin)
            cy = int(np.clip(c[1], 0, ny - 1))
            col = ceil_[x0:x1, cy:min(ny, cy + depth), z0:z1]
            ys = np.where(col.any(axis=(0, 2)))[0]
            if ys.size:
                ceil_under = (cy + int(ys[0]) + vmin[1]) * vs       # 天花板下沿
                if coarse_vs:
                    ceil_under = np.floor(ceil_under / coarse_vs) * coarse_vs
                pos[1] = ceil_under - half[1]
        else:
            obs_bottom = float(e["_obs_pos"][1] - e["_obs_half"][1])
            target = find_floor_top(pos, half)
            if target is not None and coarse_vs:     # 地板对齐显示层粗块顶面
                target = float(np.ceil(target / coarse_vs - 1e-9) * coarse_vs)
            parent = None
            for f in placed:
                if f["custom_meta"].get("mount") == "wall":
                    continue
                fh = rdims(f) / 2.0
                fp = np.array(f["position"], float)
                if abs(pos[0] - fp[0]) > fh[0] + 0.05 or abs(pos[2] - fp[2]) > fh[2] + 0.05:
                    continue
                f_obs_top = float(f["_obs_pos"][1] + f["_obs_half"][1])
                # 观测底面贴着/悬在 f 顶面上方(吊灯悬于餐桌也算)= 坐在 f 上
                if -0.35 <= obs_bottom - f_obs_top <= 0.75:
                    f_top = float(fp[1] + fh[1])
                    if target is None or f_top > target:
                        target = f_top
                        parent = f
            if target is not None:
                pos[1] = target + half[1]
                if parent is not None:
                    meta["support_id"] = parent["id"]
        # --- 2. 推出墙体(沿重叠更薄的水平轴,推向远离墙心一侧) ---
        c = to_cell(pos)
        y0, y1 = max(0, int(c[1] - half[1] / vs) + 1), min(ny, int(c[1] + half[1] / vs))
        x0, x1 = max(0, int(c[0] - half[0] / vs)), min(nx, int(c[0] + half[0] / vs) + 1)
        z0, z1 = max(0, int(c[2] - half[2] / vs)), min(nz, int(c[2] + half[2] / vs) + 1)
        ov = np.argwhere(wall[x0:x1, y0:y1, z0:z1])
        if len(ov):
            spanx = int(ov[:, 0].max() - ov[:, 0].min()) + 1
            spanz = int(ov[:, 2].max() - ov[:, 2].min()) + 1
            axis = 0 if spanx <= spanz else 2      # 重叠薄的轴 = 墙的法向
            lo = ov[:, 0 if axis == 0 else 2].min()
            hi = ov[:, 0 if axis == 0 else 2].max()
            ext = (x1 - x0) if axis == 0 else (z1 - z0)
            # 墙贴近盒的哪一侧,就往反方向推出重叠厚度
            push_cells = (hi + 1) if lo <= ext - 1 - hi else -(ext - lo)
            push = float(np.clip(push_cells * vs, -max_push_m, max_push_m))
            pos[0 if axis == 0 else 2] += push
        e["position"] = [float(pos[0]), float(pos[1]), float(pos[2])]
        placed.append(e)

    # --- 3. 两两分离(渲染尺寸;大的不动,小的沿最小平移轴推开;迭代收敛)。
    #     支撑对跳过(电脑坐在桌上不是互嵌);椅↔桌要分离:椅背(1.125m)
    #     高过桌面(0.875m),"塞桌下"在渲染上必然穿模——宁可摆桌边。 ---
    order = sorted(range(len(ents)),
                   key=lambda i: -float(np.prod(rdims(ents[i]))))

    def _overlaps_any(k, pos_k):
        hk = rdims(ents[k]) / 2
        for m in order:
            if m == k:
                continue
            om = (hk + rdims(ents[m]) / 2) - np.abs(pos_k - np.array(ents[m]["position"]))
            if (om > 1e-6).all():
                return True
        return False

    for _ in range(3):
        moved = False
        for ii, i in enumerate(order):
            for j in order[ii + 1:]:
                a, b = ents[i], ents[j]
                if a["custom_meta"].get("support_id") == b["id"] or \
                   b["custom_meta"].get("support_id") == a["id"]:
                    continue
                pa = np.array(a["position"]); pb = np.array(b["position"])
                ha = rdims(a) / 2; hb = rdims(b) / 2
                overlap = (ha + hb) - np.abs(pa - pb)
                if (overlap > 1e-6).all():
                    # 先试重叠薄的水平轴;若推过去撞上第三件(乒乓),换另一轴
                    axes = [0, 2] if overlap[0] <= overlap[2] else [2, 0]
                    done = False
                    for ax in axes:
                        sign = 1.0 if pb[ax] >= pa[ax] else -1.0
                        cand = pb.copy()
                        cand[ax] += sign * float(min(overlap[ax], max_push_m))
                        if not _overlaps_any(j, cand):
                            b["position"] = [float(v) for v in cand]
                            done = True
                            break
                    if not done:               # 两轴都堵:仍推薄轴(至少解本对)
                        ax = axes[0]
                        sign = 1.0 if pb[ax] >= pa[ax] else -1.0
                        pb[ax] += sign * float(min(overlap[ax], max_push_m))
                        b["position"] = [float(v) for v in pb]
                    moved = True
        if not moved:
            break

    # 子件跟随父件的分离位移(父被推走,坐在其上的物件同步平移)
    by_id = {e["id"]: e for e in ents}
    for e in ents:
        pid = e["custom_meta"].get("support_id")
        if pid and pid in by_id:
            f = by_id[pid]
            fp = np.array(f["position"]); fh = rdims(f) / 2.0
            pos = np.array(e["position"], float)
            pos[1] = fp[1] + fh[1] + rdims(e)[1] / 2.0
            e["position"] = [float(v) for v in pos]

    for e in ents:
        e.pop("_obs_pos", None)
        e.pop("_obs_half", None)


def snap_small_to_support(small_ents, furn_ents, preset_extents=None,
                          max_drop_m=1.5):
    """小物体的父子支撑吸附:书/花瓶应落在家具顶面上,不许悬空。

    小物体保持的是原始观测位置,但其支撑家具在摆放解算中可能被吸附/推移,
    书就悬在原地(父子关系断裂)。此后处理对每个小物体:
      · 若其在某件家具**渲染盒内部**(书柜隔层里的书)→ 不动(在柜内合法);
      · 否则在其 (x,z) 正下方找最近的家具顶面(渲染尺寸、解算后位置),
        1.5m 内有 → 底面吸附上去;找不到 → 保持观测位(可能在地板/窗台上)。
    """
    preset_extents = preset_extents or {}

    def rdims(e):
        mc = e["custom_meta"].get("mc_item")
        ext = preset_extents.get(mc)
        return np.array(ext if ext else e["bbox_dims"], float)

    for s in small_ents:
        pos = np.array(s["position"], float)
        half_h = float(s["bbox_dims"][1]) / 2.0
        bottom = pos[1] - half_h
        best_top = None
        inside = False
        for f in furn_ents:
            fp = np.array(f["position"], float)
            fh = rdims(f) / 2.0
            if abs(pos[0] - fp[0]) > fh[0] or abs(pos[2] - fp[2]) > fh[2]:
                continue
            top = fp[1] + fh[1]
            bot = fp[1] - fh[1]
            if bot - 0.02 < pos[1] < top + 0.02:
                inside = True                      # 柜体/隔层内:合法,不动
                break
            if top <= bottom + 0.05 and (best_top is None or top > best_top):
                best_top = top
        if inside:
            continue
        if best_top is not None and bottom - best_top <= max_drop_m:
            pos[1] = best_top + half_h
            s["position"] = [float(v) for v in pos]
            s["custom_meta"]["support"] = "furniture"
