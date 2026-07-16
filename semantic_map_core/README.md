# semantic_map_core

自研 Hydra 语义建图核心的**独立 C++ 移植**——占据图 → ESDF → GVD → places → rooms →
(objects / scene graph / surface)。**无 Python / numpy / scipy / skimage / networkx 依赖**,
单一原生二进制,面向机器人(泰山派/ARM)部署(原版 Hydra 即 C++,目标机不跑 Python)。

## 目标

- **独立部署**:甩掉整套 Python 科学栈运行时,像原版 Hydra 一样可上机器人。
- **与现 Python 管线(m3_adapter)逐格 bit 一致**:每个模块都对 Python 实现做逐格/逐帧对拍,零误差或极小才算通过。

## 已移植模块与对拍状态

| 模块 | 文件 | 对拍(vs Python) |
|---|---|---|
| ObsMap(log-odds 占据+语义积分) | `src/obsmap.cpp` | 逐帧占据完全一致;25× 快 |
| ESDF(可分离 Felzenszwalb EDT + 最近障碍索引) | `src/field.cpp` | 距离逐格 0 差(vs scipy) |
| GVD 提取 + denoise | `src/field.cpp` | 不一致格 0 |
| 3D 骨架化(Lee 1994,移植自 skimage 0.26 源码) | `src/skeletonize.cpp` | 逐格 0 差(vs skimage) |
| skeleton_to_graph(places) | `src/graph.cpp` | 边 100% 一致 + 819/820 节点一致(1 节点位置浮点平局差 0.1m) |
| rooms(clearance 切门口) | `src/graph.cpp` | 同图逐节点 820/820 一致 |
| GVD 压缩抽图(Hydra 式,备选) | `src/graph.cpp` | 备选,未启用(实测比 skeleton 密) |

**待移植**:objects(DBSCAN 聚类)· scene_graph(DSG + 查询)· surface(2D 地面)。

## 构建

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/bench_obsmap    # 各 bench = 对拍 + 计时
./build/bench_field
./build/bench_skel
./build/bench_rooms
```

## 对拍方法

C++ 结果写二进制 → Python 读取,与 numpy/scipy/skimage 逐格比较。见 `bench/` + 仓库外的
对拍脚本。合成场景注意点落 voxel 中心以避开 floor 在体素边界的 float 摇摆。

## 优化

见 `OPTIMIZATION.md`。当前为单线程 naive,优先 bit 一致;提速(增量 GVD + OpenMP)留待功能铺满后统一做。

## 算法出处 / 致谢

- ESDF:Felzenszwalb & Huttenlocher 2004 可分离距离变换。
- 3D 骨架化:Lee et al. 1994,移植自 scikit-image 0.26 的 Cython 实现(BSD-3)。
- GVD / GraphExtractor 思路参考 MIT-SPARK Hydra(Hughes et al.)与 voxblox 稀疏骨架(Oleynikova 2018)。
- 上游 Python 实现:自研 slam-voxel-world / m3_adapter。
