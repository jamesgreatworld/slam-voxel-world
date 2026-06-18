# 实心化结构补全(SlabFill 多层 + WallFill 曼哈顿)

> 日期:2026-06-18。设计文档(spec)。
> 解决的问题:重建数据**只把"测到的表面格子"标成占据**,所以楼板/墙/顶渲染成 1 块厚的薄片(渲染器是对的,MC 实心方块;数据是薄的)。本设计在 **L1 虚拟层**加结构 generator,把表面**补洞 + 增厚成实心方块**,并**逐楼层**处理、**保留门窗开口(observed_free)**,得到 Minecraft 风格的实心结构。
> 接续 `2026-06-18-layered-map-architecture-design.md`(L0 观测 / Lc 流水线 / L1 覆盖)与已落地的 `vlayer`(overlay/compose/pipeline/floor_fill)。

## 0. 关键事实(已用数据核实)
- 语义 super_id:**floor=3、ceiling=4、wall=19**。
- 公寓 `obsmap` 实测:floor 体素分两簇——地面 ≈ 0.0±0.2m、抬高层 ≈ +0.95m;ceiling ≈ +3.8~4.3m。→ **多层楼面真实存在**,现 `floor_fill` 只补主峰(地面),二楼漏。
- "薄层"成因 = 只占据表面格子、内部空;**实心化 = 填内部格子成块**。门/窗/通道 = `observed_free`,**不填**。

## 1. 架构(沿用现有 vlayer 契约)
两个 stage-1 generator,产 `binding=persistent`、`generator=...` 的 `add` VoxelDelta,经现有 `compose_structure` 进 `.vxw`。ObsMap(L0)只读。

- **`SlabFill(label, side, ...)`**:水平板(floor/ceiling)补全 + 实心化,多层。`floor_fill` 退役为 `SlabFill(label=3, side="floor")`(保留旧测试通过或迁移)。
- **`WallFill(...)`**:竖直墙(label=19)曼哈顿补全 + 增厚,保留开口。
- 都注册到 stage 1,`depends_on=[]`,可与现有 floor 并存。

## 2. SlabFill(floor + ceiling,多层 + 实心化)

参数:`label`(3 或 4)、`side`("floor"=向下增厚 / "ceiling"=向上增厚)、`thickness_m=0.15`(实心化厚度,~3 体素)、`level_gap_m=0.5`(分峰最小间隔)、`close_radius=2`。

`run(ctx)`(只读 L0):
1. `surf = occ & (sem==label)`;无 → `[]`。
2. **分层**:对 `surf` 的 Y 索引做直方图,按 `level_gap_m`(/voxel_size 取整)做 1D 峰聚类 → 各层代表高度 `Y_k`(该层 surf 的众数/中位 Y)。
3. **逐层补洞**:对每层 `Y_k`:
   - 足迹 = 该层附近垂直窗口内的 `observed_free` 投影到 XZ(`free[:, Y_k-W : Y_k+W, :].any(axis=1)`,W 由 level_gap 推) ∪ 该层 surf 的 XZ;形态学闭(close_radius)。
   - 对足迹每个 (x,z):若 `(x,Y_k,z)` 空 → `add`(occupied + sem=label)。
4. **实心化(增厚)**:对该层已占据 + 刚补的每个 (x,Y_k,z):
   - side="floor":向下填 `(x, Y_k-1..Y_k-T+1, z)` 中为空且**非 observed_free** 的格子(T = thickness/voxel)。
   - side="ceiling":向上填 `(x, Y_k+1..Y_k+T-1, z)` 同理。
   - 不覆盖观测、不填 observed_free(防穿到下层空间/门洞)。
5. 产出全部 `add`(generator=`slab_fill`,binding=persistent)。

> 多层自动覆盖二楼;实心化把薄片变 T 厚实心。`floor_fill` 调用点改用 `SlabFill(3,"floor")`。

## 3. WallFill(曼哈顿,保留开口,增厚)

参数:`thickness_m=0.10`(墙厚,~2 体素)、`min_wall_cells=20`(忽略小碎墙)、`close_radius=2`。

`run(ctx)`(只读 L0):
1. `wall = occ & (sem==19)`;无 → `[]`。
2. **曼哈顿分面**:墙近似轴对齐竖直面。分别按 **X 切片**(每个 x:wall 在该 x 的 (z,y) 集)和 **Z 切片**(每个 z:wall 在该 z 的 (x,y) 集)统计;一个 x(或 z)切片若 wall 格数 ≥ min_wall_cells 且在 (z,y)(或 (x,y)) 上铺成面 → 认定为一段"沿 Z(或沿 X)的墙",法向 = ±X(或 ±Z)。
3. **面内补洞(保留开口)**:对每段墙面(固定 x、在 (z,y) 平面):
   - 取该面 wall 的 (z,y) 包围盒;盒内每个 (z,y) cell,若 `(x,y,z)` 空 **且非 observed_free** → 候选补。
   - 形态学闭(2D,在 (z,y) 面上)补小洞;**observed_free 的格子永不补**(门/窗/通道)。
   - 命中候选 → `add`(occupied + sem=19)。
4. **增厚**:对该面每个(已占据+补的)cell,沿法向填 `±1..±(T-1)` 体素中为空且非 observed_free 的格子,使墙有 T 厚。
5. 产出 `add`(generator=`wall_fill`,binding=persistent)。

> 保守:只在**已观测墙面的包围盒内**补,不向外凭空造墙;observed_free 一律跳过 → 门窗/通道保留。

## 4. compose / 导出(不变)
两 generator 产的 `add` 经现有 `compose_structure`(completed 只填未占据格,不覆盖观测)→ `occupancy_to_vxw`。导出调用:`obsmap_to_completed_vxw(obsmap, out, generators=[SlabFill(3,"floor"), SlabFill(4,"ceiling"), WallFill()], palette=...)`。

## 5. 测试策略(Python/pytest,可靠)
沿用 floor_fill 风格,构造小 ObsMap:
- **SlabFill 多层**:两个 floor 高度(如 y=2 与 y=6)+ 各自 free 上方 → 断言两层都补、各按 thickness 增厚、observed 不变、observed_free 不被填、未观测格被填。
- **SlabFill ceiling**:label=4、side=ceiling、向上增厚。
- **WallFill 保留开口**:一面墙(固定 x、(z,y) 矩形)中挖一个 observed_free 的"门洞"+ 一个未观测小洞 → 断言小洞被补、门洞(free)不补、沿法向增厚到 T。
- **L0 只读**:跑完 `obsmap` 字节不变。
- 回归:现有 `test_floor_fill`(floor_fill→SlabFill 迁移后仍绿)。

## 6. 模块边界
- `m3_adapter/vlayer/generators/slab.py`:`SlabFill`(取代/泛化 floor.py;`floor.py` 可留薄包装 `FloorFill = SlabFill(3,"floor")` 以免破坏现有导入,或直接迁移测试)。
- `m3_adapter/vlayer/generators/wall.py`:`WallFill`。
- 不动 overlay/pipeline/compose/export(契约已够)。
- 导出脚本(若有 settled 命令)更新 generators 列表。

## 7. 范围 / 推迟
**本期**:SlabFill(floor+ceiling、多层、实心化)+ WallFill(曼哈顿、保留开口、增厚)+ 测试 + floor_fill 迁移。
**推迟**:斜面/非曼哈顿墙(RANSAC)、楼板填到"下一层之间"的精确 slab(现用固定 thickness 增厚)、窗台/门框语义细节、Lc 增量(dirty 区域)重跑。

## 8. 风险 / 待定
- **分峰**:地面簇厚(±0.2m, 6 体素)+ 噪声 → 峰聚类用 level_gap_m 合并近邻层,避免把噪声当多层。
- **曼哈顿假设**:斜墙/弧墙不适用(本期不处理,RANSAC 推迟)。
- **增厚方向穿层**:floor 向下增厚 T 不应穿进下层房间顶部 → T 取小(0.15m)+ 不填 observed_free 已基本防住;多层间距(0.95m)远大于 T。
- **observed_free 边界毛糙**:close_radius 过大可能糊掉小开口 → 保守取 2,且 free 永不被覆盖(硬规则)。
