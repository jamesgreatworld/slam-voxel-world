# SP-B v2:增量 ESDF/GVD(脏区域局部重算)设计

> 日期:2026-06-17。目标:GVD 更新代价 ∝ 每批变化区,不再每批全图 EDT(v1 是 ~17s/批全图)。
> 务实路线:**脏区域局部重算**(不是 Voxblox 完整波前 raise/lower 队列;那是更远期的真常数时间版)。拿到"局部时间"这一大块加速,风险低。

## 1. 核心思想
v1 每批对全 48M 格跑 scipy EDT + 全图 GVD,慢且浪费。v2:
- ObsMap.integrate_frame 时**记录被改动的体素**(occupancy/free 翻转的格)→ 脏集合。
- 每批后,取脏集合的**包围盒 + margin**(margin = 相关最大距离 ≈ band/观测自由的有效半径,默认 ~1m=20体素,保证盒边 ESDF 受外部障碍影响被包含)。
- **只在这个子盒**上:densify 局部 occ + 局部 EDT + 局部 extract_gvd + 局部 thin。
- 把局部 GVD 体素**补丁进持久骨架**(persistent gvd 体素集),再(增量或局部)更新图。

## 2. 数据 / 接口
- `ObsMap` 加脏追踪:`integrate_frame` 累积 `_dirty_min/_dirty_max`(本批改动的世界体素包围盒);`pop_dirty_bbox()` 返回并清空。
  - 改动 = 某格 log-odds 跨过 occ_thr 或 free_thr 边界(状态翻转)。简化版:记所有被 touch 的格的包围盒(更保守但简单)。
- `gvd/field.py` 加 `extract_gvd_local(occ, free, vmin, voxel_size, bbox, margin, d_min, theta_sep)`:
  在 bbox+margin 子区切片,局部 EDT(scipy 对子数组)、局部 GVD,返回**子区内的 GVD 世界体素坐标**。
- 持久骨架 `gvd_voxels: set/3D-bool`:`replace_region(gvd_voxels, bbox, new_local_gvd)` —— 清掉旧 bbox 内的、填入新的。
- 图:v2.0 先**对更新后的持久骨架整体重跑 skeleton_to_graph**(图构建比 EDT 便宜得多,~3s);v2.1 再做局部图打补丁(更难,后续)。

## 3. 流程(每批)
```
integrate batch → bbox = obsmap.pop_dirty_bbox()
local_gvd = extract_gvd_local(occ, free, ..., bbox, margin)
persistent_gvd = replace_region(persistent_gvd, bbox, local_gvd)
graph = skeleton_to_graph(persistent_gvd, ...) + 清洗 + rooms   # v2.0 整体重跑图
render → live.vxw
```
**省的是 EDT**:全图 EDT(~15s)→ 局部 EDT(子盒,~ms-百ms)。每批从 ~25s 降到 ~3-5s(主要剩图构建)。

## 4. 正确性要点
- **margin 充分**:GVD 判据看 parent(最近障碍)。子盒边缘体素的最近障碍可能在盒外。margin ≥ 该区 ESDF 的最大相关距离。观测-自由是薄层(≤几体素离面),margin 1m 足够;若用 band_max,margin=band_max。**margin 不足会在补丁缝处漏判 GVD** —— 合成测试要覆盖。
- **replace_region 边界**:只替换 bbox **内部**(非 margin 区)的 GVD,避免重复/抖动。

## 5. 验证
- `extract_gvd_local` 合成单测:在大网格里造两障碍,全局 extract_gvd vs 局部(给含中线的 bbox)→ bbox 内 GVD 一致。
- `pop_dirty_bbox` 单测:integrate 一帧 → 脏盒覆盖被 touch 的格。
- 真实:stream --live --incremental,每批耗时显著低于 v1(打印 t_esdf 对比),最终 GVD 与全量批处理近似一致。

## 6. 范围外(更远期)
真 Voxblox 波前 raise/lower 队列(逐帧常数时间)、局部图打补丁(v2.1)、并行。本期只做"局部重算 EDT + 整体重跑图",这是性价比最高的一刀。

## 7. v2 达成结果(2026-06-17,commits d0a2d78/f39b5b5)
`field.extract_gvd_local`(脏盒+margin 局部 EDT/GVD,parent 相对性证明无需偏移)+ `field.replace_region`(持久 GVD 打补丁)+ `obsmap.pop_dirty_bbox`(脏盒追踪)+ stream `--incremental`(live 循环用局部更新,持久骨架,跳 rooms)+ `bench_incremental.py`。75 测试绿。
**真实数据 benchmark(out/uhumans2_apt_full.vxw,grid 547×483×185,box 60³,margin 25):full ESDF+GVD 13815ms → local 265ms = 52× 提速。** v2 达成 ✅。
