# observed-free ray-carve(正道第一步,批式)设计

> 日期:2026-06-17。根治四轮泄漏:用 rosbag 射线碰撞出"观测-自由"3D 掩码,替换 field 阶段的 flood。
> 单逻辑图、两通道(vision.md §4):占据 = .vxw(渲染);observed-free = sidecar(建图)。**瞬态可不存,但本次落盘**(算一次贵,复用)。

## 1. 数据契约:sidecar
`<vxw_dir>/observed_free.npz` = `np.savez_compressed(mask=bool[nx,ny,nz], vmin=int[3], voxel_size=float)`
- `mask[i,j,k]=True` ⇔ 该格被射线穿过(观测-自由)。
- **网格对齐**:carve 工具读 .vxw,用 `gvd.field.densify_occupancy(world, pad=1)` 拿到**与 GVD 完全相同**的 `(occ, vmin)` 和 shape,把 free carve 进这张网格。单一网格真值源,carve 与 field 永不错位。

## 2. carve 工具 `m3_adapter/uhumans2_carve.py`
复用 uhumans2_to_vxw 的 helper(unproject_depth / collect_odom / collect_tf_static / _interp_T_world_body / _chain_tf / ros_zup_to_vxw_yup)。流程:
1. 读 .vxw → densify_occupancy(pad=1) → free_grid=zeros(shape, bool), vmin, voxel_size。
2. 重扫 rosbag,逐帧:unproject → 世界表面点 `P_m`(米)+ 相机原点 `O_m`(=T_world_cam[:3,3],同样过 ros_zup_to_vxw_yup)。
3. `carve_frame(free_grid, vmin, voxel_size, O_m, P_m, free_margin_m)`:沿每条 O→P 射线按体素步长采样,落在 bbox 内的格标 free,**终点前 free_margin 截止**(不把表面标成 free)。
4. 存 observed_free.npz。
CLI:`uhumans2_carve.py <bag_dir> <vxw_dir> [--pixel-stride 4] [--max-frames N] [--free-margin 0.10] [--depth-min/max]`。首验用 stride4 + max-frames 提速。

## 3. field/pipeline 接入
- `gvd/field.py`:`load_observed_free(path) -> (mask, vmin, voxel_size)`(np.load)。
- `GvdConfig` 加 `observed_free_path: str|None = None`。
- pipeline run:densify 后,若 `observed_free_path`:加载 mask,**断言 shape==occ.shape 且 vmin 一致**(不一致报错),`free = mask & ~occ`(交占据补集稳妥),**跳过 flood + resolve_seed**;否则照旧 flood。band-clip 仍可叠加(默认关,observed-free 已有界)。
- CLI `gvd_to_vxw.py` 加 `--observed-free <npz路径>`。

## 4. 验证
- carve_frame 合成单测:单射线 → 中间格 free、终点 margin 内不 free、bbox 外不越界。
- load_observed_free + pipeline 合成单测:造个小 mask npz,pipeline 用它,free 来自 mask 而非 flood。
- 真实:carve apt → GVD `--observed-free`:**flood/free fraction 应远低于 99.1%(墙外不再连通)**;GVD 不再实心雾;渲染对比。

## 5. 成功判定
泄漏指标从 99.1% 降到"室内体积量级";GVD 骨架明显比 flood 干净(墙外无骨架)。→ 正道第一步达成,SP-B 增量化才有意义。

## 6. 首次真实结果(2026-06-17)—— 正道第一步达成 ✅

carve apt(stride4, 400 帧,269s)→ `observed_free.npz` 181KB:**free 2,335,067 / 48,877,185 = 4.78%**(对比 flood 99.1%)。网格完美对齐(shape 547×483×185,vmin=[-345,-262,-3])。

GVD `--observed-free`(band 关 + denoise30 + thin + 清洗 + room-res0.3):

| 指标 | flood(旧) | **observed-free** |
|---|---|---|
| 自由空间 | 99.1% | **4.78%** |
| GVD 体素 | 38,165 | **7,991** |
| 图节点 | 3,486 | **597** |
| 图边 | 3,754 | 589 |
| 房间 | 234 | 118 |

**判读:四轮泄漏被根治。** 骨架只在观测到的室内,墙外无骨架,图小 6 倍(渲染 out/apt_of.png)。

**残留(已归因)**:118 房仍偏高,但**原因变了**——不再是泄漏,而是 **400 帧部分覆盖**:相机没扫到的室内角落留空洞,把自由空间打成碎片,每个碎片→一个社区。**更多帧(更全的 carve)会把碎片连起来**,房数会再降。这是覆盖度问题,不是方法问题。

**下一步**:① 跑全量 carve(所有帧 / stride2)得更完整掩码 → 房数应落到合理量级;② 然后 SP-B 增量化(逐帧实时)。
