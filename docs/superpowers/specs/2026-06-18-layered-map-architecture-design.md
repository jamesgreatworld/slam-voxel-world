# 分层地图架构:不可变观测 + 先验推理 + 虚拟覆盖

> 日期:2026-06-18。设计文档(spec)。
> 解决的问题:真实观测(测量)与虚拟设计数据(自动补全 + 手动创作)如何在同一空间网格上**共存、良好结合、各自可持久化、并被房间/物体树查询**,同时保持观测纯净。
> 这是 vision.md **Phase ④ 先验引导建图**作为一个**数据层**的落地。触发条件已满足:③ 场景图(DSG)本会话完成 + 出现"屋子悬浮 / 人穿墙掉地"的直接痛点。

## 0. 一句话架构

**2 个存储 + 1 个引擎 + 1 个身份索引:**

```
compose(L0, L1):
  结构 = L0 ⊕ L1.structure_diffs          # 基底读 L0,贴差异(稀疏 add/remove/replace)
  物体 = L1 已提交物体集(observed/completed/authored)   # 用户拥有,Lc 在观测更新时调和
                  ▲ 用 DSG 的 stable id 作唯一连接键
            materialize → .vxw   ← Godot 只加载/编辑这个产物
```

| | 角色 | 持久化 | 现有代码 |
|---|---|---|---|
| **L0 观测**(存储) | `logodds`(几何 float)+ `sem_label`(语义 u8),不可变 | ✅ `obsmap.npz` | `m3_adapter/obsmap.py` |
| **Lc 先验推理**(引擎,非存储) | 跑 generator(聚类/平面拟合/补洞/模板/跟踪/GVD-rooms);**引擎本身不持久化**,但它把派生的 observed 物体**提交进 L1**;观测更新时对 L1 已提交物体做数据关联式调和(`merge_observation`:匹配则更新、新增则添加、不再观测到则按生死规则待删) | ❌ 引擎无状态;产物归 L1 | `uhumans2_to_vxw.extract_entities` + `gvd/` + `scene_graph.merge_observation` |
| **L1 覆盖**(存储) | **结构体素**:只存差异(add/remove/replace,基底读 L0);**物体**:提交全量持久条目(observed/completed/authored,带 provenance/ref),由用户拥有——观测不更新即永久保存 | ✅ 🆕 `overlay.*`(结构差异)+ `entities.json`(物体) | 🆕 `m3_adapter/vlayer/` |
| **DSG 身份/拓扑索引** | Building>Room>Place/Object 树 + 稳定 id(L1↔Lc 的 join key)+ 查询 | ✅ scene_graph 持久化 | `m3_adapter/gvd/scene_graph.py` |

## 1. 四条不变量(架构铁律)

1. **观测只读**:L0 永不被 Lc/L1 修改。观测错了也不改,只在 L1 覆盖以纠正输出(错误底片保留,可追溯,可被未来更好观测自然取代)。
2. **虚拟层是唯一输出真相**:`.vxw = compose(L0, L1)` 的物化产物,可随时重算;Godot 只加载/编辑它,编辑只回写 L1,永不触及 L0。
3. **万物带出身**:每个派生项 / 覆盖项带 `generator` + `binding` + 可选 `ref`(指向它引用/取代的观测派生 id)。
4. **结构存差异,物体存全量(均不复制 L0 稠密栅格)**:对**结构体素**(墙/地),L1 只存差异(add/remove/replace),基底从 L0 取;对**物体**(水瓶/家具),L1 提交全量持久条目(observed/completed/authored),由用户拥有——稀疏、廉价。两者都不复制 L0 的稠密观测栅格,故仍合"不重复观测"的本意。物体一旦提交进 L1 即归用户所有:观测不更新就永久存在,只有观测真的更新且矛盾时才由 Lc 触发生死规则。

## 2. 来源模型:generator + binding(先验强度 = λ)

凡"解释"观测的操作都是**先验**(聚类=空间连贯性先验、平面拟合=建筑/曼哈顿先验、补洞/遮挡恢复=几何连续性先验、模板=形状尺寸先验)。L0 是唯一"采信为测量"的东西(`logodds`+`sem_label`)。

每个虚拟项带两字段:

- **`generator`** —— 哪条先验产生:`cluster | plane_fit | occlusion | hole_fill | template | manual | …`
- **`binding`** —— **观测更新时的调和策略**(= vision.md 的 λ,可调)。进入 L1 即由用户拥有,**观测不更新就永久存在**;binding 只决定"当观测真的更新且矛盾时"怎么处理:
  - `live` —— observed 物体:观测更新冲突时(如水瓶被移走)默认删除(+提示);但观测没更新就持久
  - `persistent` —— 补全:观测更新也保留(观测缺失处仍在)
  - `independent` —— 纯创作:与观测无关,观测怎么变都在

默认由 generator 推出(cluster→live、completion→persistent、manual→independent),但**显式存**,以便先验可被观测推翻、λ 可调(vision §6"先验不能霸道")。

| generator | 先验强度 | binding 默认 | 行为 |
|---|---|---|---|
| cluster / OBB / tracking | 弱 | live | 观测更新时调和;观测移除且矛盾→默认删(+提示) |
| hole_fill / plane_fit(补全) | 中 | persistent | 观测缺失处保留 |
| occlusion(遮挡恢复) | 中–强 | persistent | 多靠推理 |
| manual(创作/覆盖) | 极强(人) | independent | 与观测无关 |

## 3. L1 对观测内容的覆盖操作集

L1 = `已提交物体集(observed/completed/authored,用户拥有)` + `结构差异(diffs)`。全部只改输出、不动 L0,按 stable id / 体素区域键:

| op | 含义 | overlay 表示 |
|---|---|---|
| **adjust** | 微调/摆正:挪位、转向、缩放、扶正 | `{ref: obj_id, op: adjust, transform: Δpose}` |
| **hide** | 不显示某观测(噪声/错误/不想要) | `{ref: obj_id, op: hide}` |
| **replace** | 换模型(mc_item)/换材质/换语义 | `{ref: obj_id, op: replace, model|material|sem}` |
| **add** | 凭空加内容(补全 / 创作) | `{op: add, geom|model, generator, binding}` |
| **remove** | 抹掉某片观测体素(结构层纠错) | `{ref: region, op: remove}` |

- `hide`/`remove` = 处理"观测错了"的正道(不改 L0)。
- `adjust(摆正)` 与自动 yaw-only 修复是同一目标两条路:自动= `generator=plane/template` 让模型默认直立;手动= `generator=manual` 摆正个别;手动优先级更高。

## 4. 合成(compose / materialize)

```
output_world = materialize( compose(L0, L1) )

compose(L0, L1):
  1. 结构 = L0 ⊕ L1.structure_diffs   # 基底读当前 L0,贴结构差异(add/remove/replace)
  2. 物体 = L1 已提交物体集            # observed/completed/authored,用户拥有;
                                       #   观测更新时由 Lc 调和(merge_observation),非整体重生
  3. 按优先级叠加:authored > completed > observed
     - completed(add/补全):只填结构中"未知"的格,绝不覆盖观测
     - authored(adjust/hide/replace/add/remove):最高优先,直接改输出
  4. extract/replace 物体:带 mc_item 的物体抠出体素、以模型渲染(现有 keep_mask 机制)
materialize → .vxw chunks(4B/体素);Godot 加载它,并读 entities.json(L1 物体真相源)+ DSG(身份/拓扑索引)
```

`.vxw` 是**渲染派生产物**,不是真相源,可从 `L0 + L1` 随时重算;`entities.json`(L1 物体)与 `overlay.*`(结构差异)才是 L1 持久真相源。

## 5. 更新策略:两种制式,按可视/变化区域局部级联

**A. 建图制式(有传感器输入,流式构建)**
```
传感器帧 → L0 在可视范围 R 内更新(integrate_frame)
        → R 标脏(dirty-box 追踪)
        → Lc 只对 R 增量重派生(局部 GVD/ESDF、重聚类、merge_observation + 删除传播)
        → L1 只对 R 调和(matched 物体更新、新物体添加、不再观测到且矛盾的按生死规则待删;persistent 补全保留、authored 保留)
```
只动可视范围,不全量重算(复用现有 `extract_gvd_local` 局部重算 / dirty-box)。

**B. 游戏制式(无传感器输入)**
L0/Lc 冻结;**只有用户操作更新 L1**;每次编辑只重合成被改的小区域。

两制式共用同一份 L1 数据结构,区别仅在"谁触发更新"。

## 6. 生死/冲突规则(observed 物体服从观测)

**前提:观测不更新 → 物体永久存在;用户可随时删除任意 L1 物体(用户拥有)。以下规则仅在观测真的更新且矛盾时触发。**

- `observed` 基底物体默认 `binding=live`——**服从观测**;微调/替换**不自动升级持久**。
- 当 Lc 删除传播判定某 `observed` 物体消失、且它带 authored 覆盖(用户动过):
  - **交互 + 在线建图**:提示"该物体已不再被观测到,保留吗?",**默认删除**;选"保留"→ 升级 `authored/independent`。
  - **无人值守 / 批处理**:无人可问 → 默认删除,但**记录日志**列出被丢弃的编辑(不静默截断)。
- 纯 `authored` 恒持久,不提示。
- `hide` 的物体若观测也消失 → 覆盖项成孤儿,静默丢弃。

## 7. 持久化 / 文件落地

**持久真相源(source of truth):**

| 文件 | 内容 | 状态 |
|---|---|---|
| `<world>/obsmap.npz` | L0:logodds + sem_label(+ vmin/voxel_size) | 已有 |
| `<world>/entities.json` | 🔑 L1 **物体真相源**(已存在):已提交物体集(observed/completed/authored),每项带 `provenance{generator,binding}`/`ref`/`op`;用户拥有,观测不更新即永久保存 | 扩展(扩 provenance/ref/op) |
| `<world>/overlay.npz` + `<world>/overlay.json` | 🆕 L1 **结构差异真相源**:体素差异(稀疏 add/remove/replace)+ 可选的纯结构创作(generator/binding/ref/op) | 🆕 |
| `<world>/scene_graph.*`(现有格式) | DSG = 身份/拓扑索引:树 + stable id(L1↔Lc join key);Lc 增量维护、持久化以跨 run 稳定 id | 已有 |

**materialize 渲染产物(派生,可从 L0+L1 随时重算,非真相源):**
| 文件 | 内容 | 状态 |
|---|---|---|
| `<world>/`(chunks/manifest/palette) | `.vxw` 体素世界(Godot 加载) | 已有 |

存储 contract:**L0 与 L1 两个持久真相源永不互相污染;L1 物体存 `entities.json`、结构差异存 `overlay.*`;Lc 是其间的纯变换(引擎不持久化,DSG 仅持久 id/拓扑);`.vxw` 是 materialize 渲染产物,可重算。**

## 8. Lc 先验推理:插件化流水线

Lc 不是一坨代码,而是一条**有序、模块化的插件流水线**:每个先验是一个 generator 插件,注册到某个**阶段(stage)**,声明依赖;运行器按 `(stage, depends_on)` 拓扑序执行,在阶段间传递一个共享上下文(渐增的 L1 + DSG)。扩展 = 加一个插件、声明它的 stage/deps,流程不改。

### 8.1 流水线阶段(先做什么后做什么)

| stage | 名称 | 输入 | 产出 | 现有代码 | 本期 |
|---|---|---|---|---|---|
| 0 | **READOUT** 直读 | L0 | 占据/材质(近乎直读,基础步) | `obsmap` 阈值 | ✅ 既有 |
| 1 | **STRUCTURE** 结构先验 | L0 | 平面拟合/补洞/遮挡恢复 → **结构差异**(overlay) | 🆕 | 🆕 **仅 floor_fill** |
| 2 | **SEGMENT** 物体分割 | L0(+S1) | 聚类+语义投票+OBB → 物体实例(observed) | `extract_entities` | ✅ 归位 |
| 3 | **REFINE** 物体精炼 | S2 | 去碎/跟踪/时序合并 → 稳定 id 物体 | `objects.py`+`scene_graph` | ✅ 归位 |
| 4 | **TEMPLATE** 模型拟合 | S3 | 类→mc_item 模型 + 位姿(yaw-only) | `SUPER_ID_TO_MC_ITEM`+`_yaw_only_quat` | ✅ 归位 |
| 5 | **TOPOLOGY** 拓扑 | L0+S2 | GVD→places→rooms→DSG 树 | `gvd/`+`scene_graph` | ✅ 归位 |
| 6 | **VALIDATE** 物理验证 | S1/4/5 | 悬空/穿插检测 → 反馈回 S1/S4 | 🆕(Phase ④c) | ⏸ 推迟 |

数据流:`L0 →[S0..S5 依序]→ L1 项(结构差异 + 物体,带 provenance)→ DSG 索引`。S6 是闭环反馈(推迟)。
**本期只新增 stage 1 的 floor_fill;stage 2–5 是把现有散落步骤"归位"到流水线契约下(行为不变,后续可平滑替换/增插件);stage 6 推迟。**

### 8.2 Generator 插件契约

```python
class Generator(Protocol):
    id: str               # 写入 provenance.generator,如 "floor_fill"
    stage: int            # 注册到哪个阶段(0..6)
    depends_on: list[str] # 依赖的 generator id(同阶段/更早),运行器据此排序
    default_binding: str  # "persistent" | "live" | "independent"
    def run(self, ctx: LcContext) -> list[L1Item]:
        """只读 ctx.obs(L0 ObsMapView)+ ctx 中已累积的 L1/DSG;产出 L1 项
        (结构体素 add→overlay.*;物体→entities.json)。绝不改 L0。"""
```
- `LcContext` = 只读 L0 视图 + 已累积 L1 项 + DSG(让后阶段能看前阶段产物)。
- **增量模式**:每个 generator 声明是否"区域可裁剪";建图制式下只对 dirty 区域 R 运行(§5)。
- 运行器:按 `(stage, depends_on)` 拓扑序跑,累积 L1,冲突按 §4 优先级。

### 8.3 floor_fill 插件(stage 1 首个,解决"悬浮 / 掉地")
`id="floor_fill"`, `stage=1`, `generator="hole_fill"`, `binding=persistent`,只读 L0,产出地板体素 add 项:
1. **估地板高度 Y₀**:取 `sem_label==floor(=3)` 的占据体素,对 Y 做直方图取主峰;多峰→按峰分区(防糊穿楼梯)。
2. **求房间足迹**:`observed_free_mask` 投影到 XZ(有空气列即室内)→ 2D 足迹;并上已有 floor 体素 XZ;形态学闭运算补锯齿/内部小洞(核大小为参数)。
3. **盖地板**:对足迹每个 (x,z),若 `(x, Y₀, z)` 当前空/未知 → 产出 `add` 项(occupied + sem=floor)。已有真实观测的格子不动。
4. 走 §4 compose 进 `.vxw` → 实心连续地板。

复用:`_find_spawn_hint`(已有 floor 检测,floor_label=3)、`gvd/field` 已用 scipy 形态学。

### 8.4 本期实现顺序(先后)
1. `LcContext` + `Generator` 契约 + 流水线运行器(空管线能跑通)。
2. `overlay.*` 数据结构 + 读写 + §4 compose(先支持"结构差异"叠加)。
3. `floor_fill` 插件(stage 1)接入流水线 → 实心地板可见。
4. 把 stage 2–5 现有步骤**适配到契约**(薄封装,行为不变)。
5. `entities.json` 扩 `provenance/ref/op`;导出器 `obsmap_to_world` 改走 compose。
6. 测试(§10)。

## 9. 模块边界

- `m3_adapter/obsmap.py` —— L0;加"只读"契约说明(集成帧是唯一写者)。
- 🆕 `m3_adapter/vlayer/` —— L1:`overlay.py`(数据结构 + 读写 + 合成)、`generators/`(插件框架)、`generators/floor.py`(首插件)。
- `m3_adapter/obsmap_export.py` —— 改为消费 `compose(L0,L1)`(现 `obsmap_to_world` 扩成走 overlay)。
- Python 导出端 —— 在 L1 物体真相源 `entities.json` 上扩 `provenance/ref/op`。
- Godot `entity_edit_controller`/`entity_inspector` —— **物体编辑(spawn/adjust/hide/replace/delete)写回 `entities.json`**(L1 物体存储,已有路径,扩 `provenance/ref/op` 字段);**结构体素编辑写回 `overlay.*`**。
- `m3_adapter/gvd/scene_graph.py` —— DSG;L1 add 的物体也接入树(统一查询)。

## 10. 测试策略

- `vlayer/overlay`:add/remove/replace/adjust/hide 的合成正确性 + 优先级(authored>completed>observed);observed 物体提交进 L1(`entities.json`),结构差异落 `overlay.*`,observed 物体不混入结构差异栅格。
- `floor_fill`:构造带洞 floor 的小 ObsMap → 断言足迹正确、洞被补、已观测格不变、多峰分区不糊穿。
- 生死规则:观测不更新时 observed 物体持久存在(回归守护);仅当观测更新且矛盾(物体不再被观测到)时默认删除,带 authored 覆盖时产出"待确认删除"事件(headless 默认删 + 日志)。
- 更新制式:dirty 区域内 L0→Lc→L1 局部级联;区域外不变。
- L0 只读:任何 generator/compose 路径跑完后 `obsmap.npz` 字节不变(回归守护)。

## 11. 范围(本期 vs 推迟)

**本期(本 spec → 一个实现计划):**
- `LcContext` + `Generator` 契约 + 流水线运行器(§8.2)。
- `vlayer/overlay` 数据结构 + 读写 + compose(含优先级)。
- stage 1 `floor_fill` 插件(§8.3);stage 2–5 现有步骤**适配到契约**(薄封装,行为不变)。
- `entities.json` 扩 provenance/ref/op(L1 物体真相源);导出器走 compose(结构贴 overlay 差异、物体取 entities.json)。
- 生死/冲突规则的"产出事件 + headless 默认删 + 日志"(Godot 提示 UI 可后续接)。

**推迟(后续各自 spec):**
- 墙面平面拟合 / 遮挡恢复 / 模板拟合(Phase ④a/b 其余 generator)。
- L2 结构编辑的分层化(体素级手动编辑接 overlay,目前靠整世界 snapshot)。
- Godot 端"保留?"提示 UI 与在线流式重派生触发的工程化。
- λ(binding 强度)学习/自动整定(vision §6,最远期)。

## 12. 待定 / 风险

- 足迹闭运算核大小:太大糊到墙外,需经验值 + 上限(不静默)。
- 多层/错层:Y 直方图多峰策略需在真实公寓验证。
- `.vxw` 既是 materialize 产物又被 Godot 编辑:编辑必须回写 L1(物体→`entities.json`、结构→`overlay.*`),而非直接改 chunks(否则破坏"L0+L1 可重算")。
- stable id 跨 run 稳定性依赖 DSG 现有跟踪;大幅重观测下 id 漂移需观察。
