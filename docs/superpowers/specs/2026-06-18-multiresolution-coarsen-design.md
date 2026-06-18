# 多分辨率粗化 L1 导出(细观测 → 粗块游戏世界)

> 日期:2026-06-18。设计文档(spec)。
> 解决的问题:L0 观测细网格(0.05m)只占据表面格子,渲染成薄片;且小洞/噪声暴露。把 **L1 游戏世界按更粗的块(默认 0.2m)降采样导出** → Minecraft 厚块感、薄层/双层并成实心粗块、小洞被吞、门窗(大开口)因尺寸保留。**L0 不变**;粗化是导出时的后处理,与 SlabFill/WallFill 正交(可叠加,也可单用)。

## 1. 架构
纯后处理:`compose_structure` 出细 `(occ, sem)` → **降采样成粗 `(occ_c, sem_c, vmin_c, vs_c)`** → 现有 `occupancy_to_vxw` 用粗 voxel_size 物化。L0/overlay/compose 不动。

- 新增 `m3_adapter/vlayer/coarsen.py`:`downsample_occupancy(occ, sem, vmin, vs, factor, min_fine=1)`。
- `obsmap_to_completed_vxw` 增参 `coarsen_to_m: float | None = None`(None=细网格,现行为不变;设值=粗化)。

## 2. downsample_occupancy(细 → 粗)
输入:`occ`(bool nx,ny,nz)、`sem`(uint8 同形)、`vmin`(int64 (3,),细体素坐标)、`vs`(float)、`factor`(int,如 4)、`min_fine`(粗块判占据所需的最少占据细格,默认 1=任意)。

1. **对齐 + 补齐**:每轴 `pad_lo = vmin[a] % factor`(把 vmin 对齐到 factor 边界),`pad_hi = (-(shape[a]+pad_lo)) % factor`(补到可整除);对 occ 补 False、sem 补 0。
2. **分块**:reshape 成 `(NX,factor, NY,factor, NZ,factor)`。
3. **粗占据**:`occ_c = (occ_blocks.sum(axis=(1,3,5)) >= min_fine)`。
4. **粗语义**:每粗块在其占据细格里取**众数 super_id**(无占据→0)。向量化:对每个 label 统计块内该 label 的占据数,取 argmax;或退化为"占据细格 sem 的众数"。
5. **坐标**:`vs_c = vs * factor`;`vmin_c = (vmin - pad_lo) // factor`(每轴)。
6. 返回 `(occ_c, sem_c, vmin_c, vs_c)`。

世界对齐不变:细格 world = `(i+vmin)*vs`;粗格 world = `(ic+vmin_c)*vs_c`,因 `vmin-pad_lo` 是 factor 倍数,边界与 world 一致。

## 3. 导出集成
`obsmap_to_completed_vxw(obsmap, out, generators, palette=None, chunk_extent=32, coarsen_to_m=None)`:
```
overlay = run_pipeline(...); occ,sem = compose_structure(...)
if coarsen_to_m:
    factor = max(1, round(coarsen_to_m / obsmap.voxel_size))
    occ, sem, vmin, vs = downsample_occupancy(occ, sem, obsmap.vmin, obsmap.voxel_size, factor)
else:
    vmin, vs = obsmap.vmin, obsmap.voxel_size
occupancy_to_vxw(occ, vmin, vs, out, chunk_extent, semantic_grid=(sem if palette else None), palette=palette)
overlay.save(...)
```

## 4. 默认与取舍
- 默认块 0.2m(factor 4 @ vs 0.05),`min_fine=1`(任意细格占据→粗块实心:最大化"实心 + 吞薄层/小洞")。
- 门/窗(大开口,>> 0.2m)即使 min_fine=1 也只缩小约 1 块/侧,仍开。
- 想保留更多细节 → 调大 min_fine 或减小块。

## 5. 测试(pytest)
- **薄层→实心粗块**:1 细格厚的地板 → factor 2 后该粗层 occ=True、sem=floor。
- **双薄层并块**:相距 1 格的两薄层落入同一粗块 → 一个实心粗块。
- **吞小洞**:floor 中一个细格洞 → 粗块仍实心(块内其他细格占据)。
- **大空区仍空**:整块无占据 → 粗块 occ=False(开口保留)。
- **坐标**:`vs_c=vs*factor`;`vmin_c` 对齐;world 边界一致(抽样校验某粗格 world ≈ 对应细格区间)。
- **集成**:`obsmap_to_completed_vxw(..., coarsen_to_m=0.2)` 产出的 .vxw voxel_size=0.2、体素数远小于细网格;`coarsen_to_m=None` 时与现行一致(回归)。

## 6. 模块边界
- `m3_adapter/vlayer/coarsen.py`:`downsample_occupancy`。
- `m3_adapter/vlayer/export.py`:加 `coarsen_to_m` 参数。
- 不动 compose/pipeline/generators/obsmap。

## 7. 风险 / 待定
- 众数语义向量化性能:大网格逐块众数可能慢;可先用"占据细格 sem 的简单众数"或按 label 分通道 argmax;必要时优化。
- min_fine=1 会把孤立噪声放大成粗块;若噪声明显,调 min_fine 或先去噪。
- vmin 负值的 `%` / `//`:用 Python 的向下取整语义(numpy floor_divide),确保对齐正确(测试覆盖负 vmin)。
- 与 SlabFill/WallFill 叠加顺序:先补全(细)再粗化 → 粗世界既实心又补了大缺口(推荐用法)。
