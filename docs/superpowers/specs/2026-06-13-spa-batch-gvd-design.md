# SP-A 批式 3D GVD 设计(去风险实验)

> 日期:2026-06-13。路线图③(自研 Hydra)第一步。
> **定性:这是一个去风险实验,不是"构建保证正确的骨架"。** 核心未知是:在含噪的真实扫描数据上,
> "种子洪泛界定自由空间 + EDT-GVD"能否出干净的室内骨架。这个问题只有跑一次才有答案。

## 1. 目标与产出边界

**输入**:任意 `.vxw`(主要试验对象 `out/uhumans2_apt_full.vxw`:43.4 万占据体素,
紧致包围盒 545×481×183 体素 ≈ 27.3×24.1×9.2 m,体素 0.05 m,两层公寓含楼梯)。

**输出**(产出边界,已锁):
1. 一个新 `.vxw`:原几何原样 + GVD 骨架体素以新增发光材质叠加 → 用现有 Godot 查看器直接看,零查看器改动;
2. stdout 统计:洪泛体积与占比、ESDF/GVD 体素数、各阶段耗时、漏穿警报。

**不做**(留给后续):稀疏图节点/边(SP-D)、增量(SP-B)、体积层持久化(SP-B)、weight/概率(vision.md §4.1)。
ESDF/parent 全程**瞬态**,只活在适配器内存里,核心 `.vxw` 格式零改动。

## 2. 算法(忠实 Hydra 定义,实现用 scipy 替代增量 brushfire)

Hydra 链路:TSDF 表面为障碍 → ESDF(带 parent/basis point)→ GVD = 到≥2 障碍等距的体素集
(判定:邻居 parent 分叉)。批式场景下,`scipy.ndimage.distance_transform_edt(return_indices=True)`
一次给出距离场 + 每个自由体素的最近障碍索引(= parent),数学上等价,无需自写波前。

管线(全 numpy 向量化,无逐体素 Python 循环):

```
1 读 .vxw → 紧致包围盒 + 1 体素 padding 的稠密 bool 占据网格(~48M 格,48 MB)
2 自由空间 = 种子洪泛:
    scipy.ndimage.label(~occupied, 6 连通) → 取含种子的连通分量为 free_mask
    种子优先级:--seed x,y,z(米)> manifest.spawn_hint > 自动(取最中央 floor 语义体素上方 1m)
    注意:apt_full 的 manifest 无 spawn_hint(生成早于 spawn-search 功能),试验时显式
    --seed 3.875,1.2,5.575(取自 apt_ent 的 hint)
    漏穿警报:free_mask 体积 / 包围盒体积 > 0.5 → 打 WARN(不自动回退,人工判断)
3 ESDF:distance_transform_edt(free_mask, return_indices) → dist(米)+ parent 坐标
4 GVD 判定(对 6 个轴向做数组平移比较,纯向量化):
    v ∈ GVD ⇔ dist(v) ≥ d_min(默认 0.20 m)
             ∧ ∃邻居 n ∈ free: ‖parent(v) − parent(n)‖ ≥ θ_sep(默认 0.40 m)
    d_min 剔除贴墙噪声脊;θ_sep 是"来自不同障碍"的判据(Hydra 用 parent 向量夹角,
    我们用 parent 空间间距,更简单且对体素化噪声更稳)。两参数均 CLI 可调,调参是实验的一部分。
5 写输出:复制 palette 并追加发光材质 "gvd_skeleton"(青色,emission_energy 高),
    GVD 体素(位于自由空间)以该材质写入 chunks,与原几何共存 → write_world 落盘
```

**内存预算**:dist float64 ~384 MB + indices int32×3 ~576 MB,峰值 ~1 GB,桌面机可接受。
**为什么对"室外无限空旷"不灾难**:旷处只有 1 个最近障碍,不满足"≥2 等距",不产生大骨架;
洪泛的作用是省算力 + 清室外凹角伪脊,不是救命。

## 3. 文件结构(遵守 §11 adapter 约定)

| 文件 | 职责 | 备注 |
|---|---|---|
| `m3_adapter/voxel_gvd.py` | **纯算法**,numpy 进/出,无 IO:`flood_free_space()` / `compute_esdf()` / `extract_gvd()` | SP-B 增量化时直接复用 |
| `m3_adapter/gvd_to_vxw.py` | CLI:读 .vxw → 稠密化 → 调算法 → 写叠加 .vxw + 统计 | 不 import 其他 adapter |
| `tests/test_gvd.py` | 合成场景针对性测试(见 §5) | 算法工作,不适用"只冒烟"豁免 |

依赖:`scipy` 当前经 scikit-learn 间接存在,需在 pixi.toml 显式声明(`scipy>=1.11`)。

## 4. 成功 / 失败判据(实验性)

**成功**:① 洪泛占比合理(室内体积量级,无 WARN 或 WARN 经人工确认为误报);
② Godot 截图中走廊/房间中部出现**连续**骨架带,楼梯井骨架**连通两层**;③ 全流程 < 5 分钟。

**失败与回退**:洪泛漏穿楼层/漏到室外,或骨架碎成噪点且调参(d_min/θ_sep)救不回 →
记录证据截图,另立"2.5D 分层 GVD"设计(按楼层切片 + 层间楼梯连接),本设计作废部分仅 §2 第 2/4 步。

## 5. 验证

- **合成测试(pytest)**:① 单个空盒房间 → GVD 出现在中轴附近且 dist 最大值≈半宽;
  ② 两平行墙走廊 → GVD 为走廊中线;③ 种子在封闭房间 → 洪泛不越墙。
- **真实数据**:apt_full 跑通 + headless 截图(默认参数 1 张 + 必要时调参对比),人工判读 §4 判据。
- 总体冒烟:输出 .vxw 能被现有查看器正常加载(无 push_error)。

## 6. 风险登记

| 风险 | 概率 | 缓解 |
|---|---|---|
| 扫描洞致洪泛漏穿楼层/室外 | 高(数据现实) | WARN 指标 + 回退 2.5D 路线已预案 |
| 骨架噪声(体素化锯齿致 parent 分叉误报) | 中 | d_min/θ_sep 可调;失败判据明确 |
| 48M 稠密网格内存 | 低 | ~1 GB 峰值,实测验证;超限再裁 ROI |
| apt_full 无 spawn_hint | 已确认 | --seed 显式传入,三级种子策略 |
