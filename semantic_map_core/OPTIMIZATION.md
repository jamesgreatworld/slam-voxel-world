# semantic_map_core 优化计划

> 记录 field/thin 等模块的提速空间与优先级。**当前所有模块是单线程 naive 实现,
> 优先保证与 Python 逐格 bit 一致;优化留到功能地基铺满后统一做。**

## 背景结论(为什么现在"没快多少"是正常的)

Python→C++ 的提速倍数,与"这段 Python 有多少是解释执行"成正比:

| 模块 | Python 底层 | 实测提速 |
|---|---|---|
| obsmap | 纯 Python 解释循环 | **25×** |
| ESDF | scipy 编译 C | 2.6× |
| GVD | numpy 向量化 SIMD | ~1×(略慢) |
| thin | skimage 编译 Cython | 0.4×(反慢) |

field/thin 的 Python 版本就是编译库,单线程 C++ 追平即正常。**C++ 真正能反超的两招是:多线程 + 增量**(库都不做这两件)。C++ 重写的第一价值是**独立部署(甩掉 Python 运行时,像原版 Hydra)**,速度是附带。

## 三层优化(按优先级)

### 层 2(先做):增量重算 —— 在线最大提速,算法层
- 现状:每帧**全网格重算** ESDF/GVD(House 49-129ms)。
- 方案:照原版 Hydra 的 **brushfire raise/lower 波前**(Lau et al.),只重算**变动区域**。机器人动一点只更新附近几百格,而非 59 万格 → **数量级提速**。
- 依赖:obsmap 已有 dirty-box 追踪可复用;需 GVD 层维护 raise/lower 队列。
- 风险:增量与批量结果需一致(收敛态对拍)。

### 层 1:OpenMP 多线程 —— C++ 反超库的主要手段,易
- **ESDF**:3 趟里每条 1D 线独立 → `#pragma omp parallel for` 摊多核。每格独立算,**bit 一致自动保持**。预期 3-4×。
- **GVD**:三轴循环按格独立 → OpenMP。bit 一致自动保持。预期 3-4×。
- **thin**:`find_candidates`(占 88%)是只读扫描 → 并行收集候选。**注意**:必须按格序拼回候选列表(否则复检删点序变→结果变),才能保持 bit 一致。预期 3-4×,有望从 137ms 干到 ~40ms 反超 skimage。
- CMake 加 `find_package(OpenMP)` + `-fopenmp`。

### 层 3:微优化 —— 锦上添花
- **GVD**:现每格用**除法**解包 parent 扁平索引(慢)。改 parent 存 3 个独立数组(px,py,pz)免除法 → 2-3×,可反超 numpy。
- **SIMD**:EDT/GVD 内层手工向量化,边际收益。
- 内存布局/cache 友好。

## 执行顺序

1. **先移植完剩余模块**(objects / scene_graph / surface),全部 bit 对齐 —— 铺满地基。
2. 再统一上 **层2 增量 GVD**(在线痛点)。
3. 顺手加 **层1 OpenMP**(ESDF/GVD 免费,thin 需保候选序)。
4. 最后 **层3 微优化**。

原则:先功能对齐、再优化,不边移植边调优。每步优化后仍需与 Python 对拍验证不破坏一致性。
