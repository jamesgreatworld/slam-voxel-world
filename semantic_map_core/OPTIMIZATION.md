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

1. **先移植完剩余模块**(objects / scene_graph / surface),全部 bit 对齐 —— 铺满地基。✅ 已完成(含 features/merge/consolidate 在线层)
2. 再统一上 **层2 增量 GVD**(在线痛点)。
3. 顺手加 **层1 OpenMP**(ESDF/GVD 免费,thin 需保候选序)。✅ 已完成
4. 最后 **层3 微优化**。

原则:先功能对齐、再优化,不边移植边调优。每步优化后仍需与 Python 对拍验证不破坏一致性。

---

## 已实测(2026-07-16/17, VM 2核)

### 各模块 C++ vs 自研 Python 提速(单核 pin)
- features(41 物体 shape): 2.06→0.039ms(**~53×**);merge_observation: 20.3→0.80ms(**~25×**);
  consolidate: 0.55→0.079ms(~6.9×)。在线层已全部 <1ms,非瓶颈。

### 2026-07-17 优化落地(全部对拍零误差保持: gvd/thin 逐字节一致, scene/merge/consolidate/rooms PASS)
1. **thin border-worklist**:`find_candidates` 全网格扫描 → 活跃集(img==1 且 6-邻域有 0;升序=栅格序,
   候选序列 bit-exact;完备性:border 判定只依赖 6-邻域,删点后补被删点的 6-邻居即可)。thin 110→**77ms**。
2. **OpenMP**:ESDF 每条 1D 线独立(线程私有 scratch)、GVD 幂等写 1、objects DBSCAN 邻居扫描只读。
   2 核:esdf 36→26、gvd 29→18。泰山派 4 核收益更大。
3. **fullpipe(House 全图产出场景图)228 → 170ms(2核)/ 196ms(单核)** → 全图重算 ~5.9Hz。
4. obsmap 每帧积分(C++, 13.2k 点合成帧): **31.7ms/帧** ≈ 31fps 能力。

### 端到端(House 完整数据集 356 帧/35.6s, VM 2核)
| | MIT Hydra | 自研 Python | 自研 C++ |
|---|---|---|---|
| 总耗时 | **281s**(83 关键帧, 丢半) | **40s** 原速跟播(积分 100/356) | 在线可 35.6s 实时零丢帧(投影);离线纯算 ~17s |
| 每帧 | frontend 637ms/kf(max 2s, 越跑越慢) | 积分 ~0.4s/帧 + cycle 0.6-0.9s | 积分 31.7ms + 全图 DSG 170ms |
| 峰值 RSS | 2979 MB | 488 MB | 22 MB(bench) |

### 剩余优化空间(按收益)
- **objects 43ms**:DBSCAN 逐类 cells 收集是全网格扫描 → 一次扫描按类分桶;合并簇 bbox 并查集 O(n²) 可 R-tree。
- **thin 77ms 再降**:剩余大头是 recheck 的 is_simple_point(octree 递归)→ 迭代化/查表;find 部分可 OpenMP(按序拼回)。
- **增量(层2)**:obsmap 已有 dirty-box(`pop_dirty_bbox`);场景图层=周期全图重算+merge 时序增量。室内规模
  全图 170ms 已够;真 dirty-box 窗口化(pad≥距离影响半径,边界正确性要单独对拍)留给大场景。
- **GVD parent 三数组免除法**(层3)。
