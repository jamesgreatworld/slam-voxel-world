# vlayer 用户操作层 C++ 化(阶段 A 收尾)实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 m3_adapter/vlayer 剩余模块(OpeningCarve/StairsFill/RansacPlaneFill/PlaneRegularize/coarsen/objects)零误差移植进 semantic_map_core,并给 smc_live_node 加体素编辑 RPC,使编辑语义层完全 C++ 化。

**Architecture:** 沿用本工程已验证 15+ 模块的对拍范式:C++ 实现写 `F:\Shared\claude_jobs\smc`(权威源)→ rsync 到 VM `/home/james/semantic_map_core` 用 `build_algo`(-DBUILD_ROS_NODE=OFF)编译 → bench_vlayer dump 二进制 → cmp_vlayer.py 用真实 Python vlayer 同数据跑 → 逐元素比对零误差 → robocopy /MIR 回 `F:\slam-voxel-world` 提交推送。

**Tech Stack:** C++17(semantic_map_core,零第三方依赖)、Python 参考实现(m3_adapter/vlayer)、VM Ubuntu24 对拍。

## Global Constraints

- **每模块必须零误差(或极小且逐条解释)才算完成**;从真实源码逐行移植,不许猜。
- vmrun 一律走 **PowerShell**(Bash/MSYS 改写 guest 路径参数);脚本放 `F:\Shared\claude_jobs\`,vmrun 跑 `/mnt/hgfs/Shared/claude_jobs/xxx.sh`。
- 算法 bench 用 **build_algo** 目录 + `-DBUILD_ROS_NODE=OFF`(build 目录被 ROS 构建缓存污染)。
- 代码/文档只改 **smc 权威源**,再 robocopy /MIR 到 git 镜像(直接改镜像会被冲掉)。
- 提交信息不含英文双引号(PS5.1 native 参数会碎);结尾 `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`。
- Python vlayer 在全部对拍通过前**不许删**(对拍 oracle);删除放后续计划(连同 export/vxw writer、GDExtension)。

## 对拍数据与一键脚本(已存在,直接复用)

- House 张量:`/mnt/hgfs/Shared/claude_jobs/house_{dims,occ,sem,free}.bin`(nx,ny,nz int32 + uint8 行主序)。
- `F:\Shared\claude_jobs\run_vlayer.sh`:rsync→cmake(build_algo)→make bench_vlayer→跑→python3 cmp_vlayer.py→cat 结果到 `vlayer_cmp.txt`。
- bench dump 约定:`int n; 每条 {x,y,z,op,sem} 5×int32`,集合比对;compose 网格逐字节比对。

---

### Task 1: OpeningCarve 合成门洞对拍(C++ 实现已写好,未验证)

**Files:**
- 已有: `smc/src/vlayer_generators.cpp`(OpeningCarve 实现,本次会话已写)
- Modify: `smc/bench/bench_vlayer.cpp`、`F:\Shared\claude_jobs\cmp_vlayer.py`

**Interfaces:**
- Consumes: `OpeningCarve(min_area_m2=0.35, max_area_m2=4.0, max_extent_m=2.6, enclosure_min=0.5, thickness_m=0.10, min_wall_cells=20, min_height=4, min_run=4, trim_fill=0.3)`,`run(MapView, const Overlay& acc)`
- Produces: REMOVE deltas(sem=0, generator="opening_carve")

- [ ] **Step 1: bench 加合成门洞场景**(复用 Task 已有的合成墙 so/ss/sf,先跑 WallFill 得 acc,再在墙上开一个 free 门洞穿墙)

```cpp
  // ---- 合成门洞: 墙平面上 free 连通域 -> OpeningCarve 挖 remove ----
  // 复用合成墙(so/ss/sf), 在墙面开 4x7 的 free 门(带 1 格毛边), 穿透墙平面
  for (int y = 4; y <= 10; ++y)
    for (int z = 20; z <= 23; ++z) {
      so[sid(12, y, z)] = 0; ss[sid(12, y, z)] = 0;
      sf[sid(12, y, z)] = 1;             // 门洞在墙平面上是 observed-free
    }
  sf[sid(12, 11, 21)] = 1; so[sid(12, 11, 21)] = 0; ss[sid(12, 11, 21)] = 0;  // 毛边
  Overlay acc2; { auto wds = wall.run(sm, empty); acc2.voxels = wds; }
  OpeningCarve oc;
  auto od = oc.run(sm, acc2);
  std::printf("C++ synth opening=%zu\n", od.size());
  dumpv(O + "vlayer_open_cpp.bin", od);
```

- [ ] **Step 2: cmp 加 Python 同场景**

```python
# 合成门洞: 同一面墙 + free 门, WallFill 先跑得 acc, OpeningCarve 挖
so[12, 4:11, 20:24] = 0; ssem[12, 4:11, 20:24] = 0; sfr[12, 4:11, 20:24] = 1
sfr[12, 11, 21] = 1; so[12, 11, 21] = 0; ssem[12, 11, 21] = 0
from m3_adapter.vlayer.generators.opening import OpeningCarve
_wd = WallFill(0.10, 20, 4, 4, 2).run(synctx(SynObs()))
_ctx2 = synctx(SynObs()); _ctx2.overlay.voxels = _wd
osyn = OpeningCarve().run(_ctx2)
py_open = set((d.idx[0], d.idx[1], d.idx[2], OPS.index(d.op), int(d.sem)) for d in osyn)
cpp_open = loadset("vlayer_open_cpp.bin")
print("synth opening: Py=%d C++=%d  对称差=%d" % (len(py_open), len(cpp_open), len(py_open ^ cpp_open)))
# 并把 py_open 条件并入 ok 判定, 且要求 len(py_open) > 0
```
注意:合成墙同时被 wall.run 用于 House 对拍段之后——**门洞修改必须放在 synth wall dump 之后**,或复制一份数组;两侧(bench/cmp)顺序保持一致。

- [ ] **Step 3: PowerShell 跑 run_vlayer.sh,期望 `synth opening: 对称差=0` 且 >0;若门洞被 enclosure/面积门槛拒掉,按两侧一致原则调合成场景(不是调算法)**
- [ ] **Step 4: robocopy + git commit `vlayer: port OpeningCarve (bit-exact on synthetic doorway)` + push**

### Task 2: StairsFill

**Files:** Modify `smc/include/semantic_map_core/vlayer_generators.hpp`、`smc/src/vlayer_generators.cpp`、bench/cmp

**Interfaces:** `StairsFill(double max_depth_m=1.5)`,STAIRS=15/FLOOR=3 常量与 Python 一致。

- [ ] **Step 1: 移植(完整代码,stairs.py 1:1)**

```cpp
// hpp:
class StairsFill : public Generator {
 public:
  explicit StairsFill(double max_depth_m = 1.5);
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;
 private:
  double max_depth_m_;
};
// cpp:
StairsFill::StairsFill(double max_depth_m) : max_depth_m_(max_depth_m) {
  id = "stairs_fill"; stage = 1; default_binding = "persistent";
}
std::vector<VoxelDelta> StairsFill::run(const MapView& m, const Overlay&) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  const int STAIRS = 15, FLOOR = 3;
  std::vector<VoxelDelta> out;
  bool any = false;
  int ground_y = 0; bool any_floor = false;
  for (long i = 0; i < (long)nx * ny * nz; ++i) {
    if (m.occ[i] && m.sem[i] == STAIRS) any = true;
    if (m.occ[i] && m.sem[i] == FLOOR) {
      int y = (int)((i / nz) % ny);
      if (!any_floor || y < ground_y) ground_y = y;
      any_floor = true;
    }
  }
  if (!any) return out;
  if (!any_floor) ground_y = 0;
  int depth = std::max(1, (int)std::lround(max_depth_m_ / m.voxel_size));
  for (int x = 0; x < nx; ++x)          // argwhere(any(axis=1)) 序 = (x,z) 升序
    for (int z = 0; z < nz; ++z) {
      int top = -1;
      for (int y = ny - 1; y >= 0; --y)
        if (m.occ[m.id(x, y, z)] && m.sem[m.id(x, y, z)] == STAIRS) { top = y; break; }
      if (top < 0) continue;
      int lo = std::max(ground_y, top - depth);
      for (int y = top - 1; y >= lo; --y) {
        long id = m.id(x, y, z);
        if (m.occ[id] || m.free[id]) break;
        VoxelDelta d; d.idx[0] = x; d.idx[1] = y; d.idx[2] = z;
        d.op = DeltaOp::ADD; d.sem = STAIRS;
        d.generator = "stairs_fill"; d.binding = default_binding;
        out.push_back(d);
      }
    }
  return out;
}
```

- [ ] **Step 2: 合成阶梯对拍**(House 无 label15):bench 造 3 级阶梯(15)+ 地面(3),cmp 同造;比 delta 集合,要求 >0 且对称差 0

```cpp
  // bench: 合成阶梯(x=5..7 各高 y=5/7/9), 地面 y=2
  std::vector<uint8_t> to(so.size(), 0), ts(so.size(), 0), tf(so.size(), 0);
  for (int x = 3; x <= 9; ++x) for (int z = 3; z <= 9; ++z) { to[sid(x, 2, z)] = 1; ts[sid(x, 2, z)] = 3; }
  for (int k = 0; k < 3; ++k) for (int z = 4; z <= 8; ++z) {
    to[sid(5 + k, 5 + 2 * k, z)] = 1; ts[sid(5 + k, 5 + 2 * k, z)] = 15; }
  MapView tm{to.data(), ts.data(), tf.data(), SX, SY, SZ, 0.1f};
  StairsFill st(1.5);
  dumpv(O + "vlayer_stairs_cpp.bin", st.run(tm, empty));
```
```python
to = np.zeros((SX, SY, SZ), np.uint8); tsm = np.zeros_like(to); tfr = np.zeros_like(to)
to[3:10, 2, 3:10] = 1; tsm[3:10, 2, 3:10] = 3
for k in range(3): to[5 + k, 5 + 2 * k, 4:9] = 1; tsm[5 + k, 5 + 2 * k, 4:9] = 15
from m3_adapter.vlayer.generators.stairs import StairsFill as PyStairs
class StObs:
    voxel_size = 0.1
    def __init__(s): s.sem_label = tsm
    def occupancy_mask(s): return to.astype(bool)
    def observed_free_mask(s): return tfr.astype(bool)
ssyn = PyStairs(1.5).run(synctx(StObs()))
```
- [ ] **Step 3: 跑对拍,期望对称差 0;Step 4: commit `vlayer: port StairsFill (bit-exact)` + push**

### Task 3: RansacPlaneFill(RNG 注入式对拍)

**Files:** hpp/cpp/bench/cmp 同上。

**Interfaces:** `RansacPlaneFill(labels={4}, dist_thresh=1.5, min_inliers=80, max_planes=4, iters=200, seed=0)`;新增可选 `const std::vector<std::array<int,3>>* trials`(每次迭代抽的 3 个 remaining 下标)。**对拍策略沿用 fixed-obj_ids 先例**:numpy PCG64 的 choice 不可移植 → Python 侧包一层记录每次 `rng.choice` 结果,C++ 注入同序列 → 采样之后的全部逻辑 bit 对拍;生产路径 C++ 用自带 splitmix64(文档注明与 numpy 不逐位一致,语义等价)。

- [ ] **Step 1: 移植 plane.py 25-70 行**。关键映射:`remaining @ n - d` 用 double;`np.abs(...) < dist_thresh`;`argmax(|n|)` 平局取第一个;`int(round(w))` 用 `std::lround`(Python round 半偶?—— `round()` 是 numpy float64 的 Python round → **银行家舍入**,用 `std::nearbyint` + `FE_TONEAREST` 或 rint 保持一致,工程既有 obj 质心用 lrint 的先例是 np.round 同源,这里 Python 用内建 round(float) 同为半偶 → 用 `std::rint`);remaining 收缩 `remaining[~inl_sel]` 保持原顺序。
- [ ] **Step 2: 对拍**。cmp 侧:
```python
class RecRng:
    def __init__(s, seed): s.r = np.random.default_rng(seed); s.rec = []
    def choice(s, n, k, replace): v = s.r.choice(n, k, replace=replace); s.rec.append([int(x) for x in v]); return v
# monkeypatch: plane.np.random.default_rng = lambda seed: RecRng(seed)
```
跑 Python 得 deltas+rec;把 rec 写 `ransac_trials.bin`(int32 n, 3×n);bench 读入注入 C++;比 delta 集合对称差 0。合成场景:斜面点云(label 4,y=x/2 平面 200 点)+ 若干噪声点。
- [ ] **Step 3: 跑对拍零误差;Step 4: commit `vlayer: port RansacPlaneFill (trial-injected bit-exact)` + push**

### Task 4: PlaneRegularize

**Files:** `smc/include/semantic_map_core/vlayer.hpp`(MapView 加 `const float* logodds`,默认可空)、hpp/cpp/bench/cmp。

**Interfaces:** `PlaneRegularize(k_per_cell=0.3, prior_cap=3.0, trim_fill=0.3, min_wall_cells=20, min_height=4, min_run=4, support_m=0.4, keep_comp_m2=0.09)`;依赖 `wall_fill`。

- [ ] **Step 1: MapView 加 logodds 字段**;bench 的 House MapView 传 nullptr(House 无 label19 墙,主对拍走合成);合成场景构造 logodds(墙格 +3.5、free -2.0、其余 0)。
- [ ] **Step 2: 移植 regularize.py 49-128**。关键映射:
  - `projection_peaks` 复用已有;`_robust_rect` 与 OpeningCarve 的 `robust_rect` 同一实现(共享 static)。
  - 先验场:`d_in = min(iv-rv0, rv1-iv, ih-rh0, rh1-ih)+1`,`prior = inside? min(d_in*k,cap) : -min(d_out*k,cap)`(double)。
  - `post = lo2d + prior`:lo2d 为 float32 → double 提升,与 numpy 一致。
  - `near_wall = EDT2D(~wall2d) <= support_cells`:**用平方距离整数比较** `d2 <= support_cells^2`(与 float sqrt 比较等价,免边界抖动);2D EDT 用两趟 Felzenszwalb(照抄 field.cpp edt1d 模式,平方域)。
  - cut 的大分量保护:8 连通 label(复用 label8_2d)+ 分量尺寸 >= keep_comp_cells 则不删。
  - emitted 集合语义:fill 先于 cut 遍历,`np.argwhere` 行主序。
- [ ] **Step 3: 合成对拍**:Task1 的墙(去掉门洞副本)+ 矩形外 2 格飘砖(logodds 0.9)+ 矩形内深处挖 2 格未观测洞;要求 fill/cut 均 >0 且对称差 0。
- [ ] **Step 4: commit `vlayer: port PlaneRegularize (bit-exact)` + push**

### Task 5: coarsen(downsample + clean_coarse + prune_dangles)

**Files:** Create `smc/include/semantic_map_core/coarsen.hpp`、`smc/src/coarsen.cpp`;CMake 加入 lib;bench/cmp。

**Interfaces:**
```cpp
void downsample_occupancy(const uint8_t* occ, const uint8_t* sem, int nx, int ny, int nz,
                          const long vmin[3], float vs, int factor, int min_fine,
                          std::vector<uint8_t>& occ_c, std::vector<uint8_t>& sem_c,
                          long vmin_c[3], float& vs_c, int dims_c[3]);
void prune_dangles(std::vector<uint8_t>& occ, std::vector<uint8_t>& sem,
                   int nx, int ny, int nz, int iterations = 2);
void clean_coarse(std::vector<uint8_t>& occ, std::vector<uint8_t>& sem,
                  int nx, int ny, int nz, int min_component = 2, int close_radius = 1,
                  bool keep_largest = false);
```

- [ ] **Step 1: 移植 coarsen.py**。关键映射:
  - downsample:pad 对齐 `lo = vmin[a] mod factor`(**Python mod,负数也非负**:`((v % f) + f) % f`);块内众数**严格 >**(平局取先出现的小 label,因 L 从 1 升序扫描);label 0 不参与(`where(occ_b, sem_b, 0)` 后从 1 起)。
  - prune_dangles:6 邻计数 `<=1` 迭代删,直到无变化或迭代耗尽。
  - clean_coarse:3D 6-连通 closing(dilate×r 后 erode×r,border=0);新格 sem = **最近原占据格**的 sem —— 用已有 `compute_esdf`(它带 parent 站点传播)对原 occ 求每格最近站点,新格取 `sem[parent[id]]`。**风险**:scipy EDT(Maurer)与 Felzenszwalb 等距站点平局可能不同 → 对拍时 House 上统计 tie 差异格数,0 则 PASS;非 0 则逐格验证"差异格均为等距平局"并记录数量(先例:skeleton_to_graph 819/820)。
  - 小分量删除(6-连通)与 keep_largest(最大分量,`sizes.argmax()` 平局取小 id)。
- [ ] **Step 2: House 真数据对拍**(occ/sem @0.1 → factor2):downsample/prune/clean 三个函数分别 dump (occ_c,sem_c) 逐字节比。
- [ ] **Step 3: commit `vlayer: port coarsen (downsample/clean/prune)` + push**

### Task 6: vlayer objects(解耦/簇合并/mount/实体化/摆放解算)

**Files:** Create `smc/include/semantic_map_core/vlayer_objects.hpp`、`smc/src/vlayer_objects.cpp`(+ 内置 md5,RFC1321 公式自实现);bench/cmp。

**Interfaces:**
```cpp
struct EntityModel {   // entities.json 同构
  std::string id;                    // _stable_uuid(md5)
  int label; std::string label_name;
  double position[3]; double rotation[4];  // 单位四元数
  double bbox_dims[3]; int voxel_count;
  std::string mount;                 // ""|ceiling|wall
  std::string mc_item;               // 预制模型 id, 可空
  int rgb[3]; bool has_rgb;
  std::string support_id;
};
std::vector<std::array<int,3>> decouple_objects_mask(const MapView& m,
    const std::set<int>& labels);    // 返回被剔除格
std::vector<EntityModel> extract_object_models(const MapView& m, const long vmin[3],
    const std::map<int,std::string>& label_names,
    const std::map<int,std::string>& mc_item_map, int min_voxels = 30,
    const std::set<int>& labels = ..., double max_extent_m = 2.6,
    const std::map<std::string,std::array<double,3>>& preset_extents = {},
    double coarse_vs = 0);
```

- [ ] **Step 1: 移植 objects.py 全文**(38-421 行,逐函数):
  - `_stable_uuid`:自带 md5(输入是短 ASCII,实现 RFC1321 单块/多块);格式化 `"%d-%d-%.3f-%.3f"`(printf 语义与 Python % 一致)。
  - 26-连通 3D label(光栅首遇序,新写 label26_3d,模式同 label8_2d)。
  - `_merge_overlapping_components`:O(n²) AABB gap 合并到收敛,输出按 AABB min 角排序(tuple 比较)。
  - `_mount_of`:cells 采样步长 `max(1, len//64)`,any() 逻辑逐行照抄。
  - `extract_object_models`:labels 升序;`comps >= max(4, min_voxels//4)` 预滤;centre/dims 公式照抄;`max_extent_m` 门控;rgb 通道 House 侧没有 → 对拍传 None/nullptr。
  - `resolve_placements`:吸附顺序 `sorted(key=观测底面)`(**stable sort**);`find_floor_top` 的中位数回退(`np.median` 偶数取均值→double);推墙、两两分离(3 轮迭代,order 按 -体积)、子随父。所有 `np.clip/round` 逐点对应(`round` 半偶 → std::rint)。
  - `snap_small_to_support` 照抄。
- [ ] **Step 2: House 对拍**:labels 用 House 的非结构类(复用 gvd_live 的 structure 集合取反,cmp 与 bench 同集合);mc_item_map 空 map、preset_extents 空 → 走 OBB 路径;比 EntityModel 全字段(id 字符串、position/dims 1e-9、mount/support)。
- [ ] **Step 3: commit `vlayer: port objects (decouple/models/placement) bit-exact` + push**

### Task 7: smc_live_node 体素编辑 RPC(用户操作在线入口)

**Files:** Modify `smc/src/smc_live_node.cpp`(RPC op + Overlay 集成 + 持久化)、`smc/mcp/smc_mcp_server.py`(工具透传)。

**Interfaces(RPC 新 op,一行 JSON)**:
- `edit_add_voxel {x,y,z,sem}` / `edit_remove_voxel {x,y,z}` / `edit_replace_voxel {x,y,z,sem}`(单位:ROS 米,节点内转 vxw 格;generator="manual", binding="independent")
- `edit_list {}` → 全部 VoxelDelta;`edit_clear {}`
- 语义:节点持有 `Overlay edits_`(edit_mtx_);**每个 cycle 的裁剪 occ/sem 在 denoise 之前经 compose_structure(edits_) 叠加**(manual 强制写/删,与 Python 优先级一致);`edits_` 存 `maps/house_live/overlay.json`(Overlay::save,启动 load)。
- MCP server TOOLS 增加同名 5 个工具(薄转发,无参数处理逻辑)。

- [ ] **Step 1: 节点实现 + 持久化**(compose 只作用于 cycle 快照,L0 obsmap 不动 —— Python 铁律);ROS 米→格:`gx = floor(-y/vs) - vmin[0]` 等(yup2ros 逆变换,与 publish_voxels 正变换互逆)。
- [ ] **Step 2: 在线验证(我当 agent)**:起节点(载 House 存档)→ `edit_add_voxel` 在某房间中央加 1×1×5 柱(sem=box 类)→ 下个 cycle `find_object` 应出现新物体;`edit_remove_voxel` 删掉某物体一半体素 → 若干 cycle 后该物体 voxel_count 变小/消失(misses 途径);`edit_list` 返回全部;重启节点 → 编辑仍在(overlay.json 恢复)、compose 后地图一致。
- [ ] **Step 3: commit `node: manual voxel-edit RPC backed by vlayer Overlay (persisted)` + push**

### Task 8: 文档与记忆收尾

- [ ] PIPELINE.md 加"用户编辑层"节(Overlay/binding 语义表 + RPC op 清单);OPTIMIZATION.md 无需动。
- [ ] 更新记忆 `semantic_map_core_cpp.md` 阶段 A 状态;commit `docs: vlayer edit layer` + push。

## 后续计划(本计划不含,防 scope 蔓延)

1. **export/vxw writer C++ 化 + 删除 Python vlayer**(需先读 vxw_format.py/obsmap_export.py,删除前 `git tag python-reference`)。
2. **阶段 B GDExtension**:28 个 .gd → C++(需 VM 装 godot-cpp 工具链,单独 brainstorm+plan)。

## Self-Review 记录

- 覆盖:vlayer 剩余 = opening(T1)/stairs(T2)/plane(T3)/regularize(T4)/coarsen(T5)/objects(T6)+编辑入口(T7);floor.py 是 SlabFill 别名无需移植(T2 注)。generators/__init__.py 为空文件,无内容可移植。
- 类型一致:Generator::run(MapView, const Overlay&) 全计划统一;MapView.logodds 在 T4 引入,T4 之前的模块不读它(nullptr 安全)。
- 无占位:各 task 含完整代码或"逐行移植 + 全部陷阱枚举"(RNG/舍入/平局/遍历序),数据与命令齐备。
