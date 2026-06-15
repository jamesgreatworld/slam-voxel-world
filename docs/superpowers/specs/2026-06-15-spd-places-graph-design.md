# SP-D 批式 Places 图设计(骨架 → 拓扑图)

> 日期:2026-06-15。路线图③第二步,上游 = SP-A 的细化骨架(`--band-max 1.0 --min-component 30 --thin`,38K 体素)。
> 目标:把 1 体素宽的 GVD 骨架**稀疏化成拓扑图**(节点+边)= Hydra 的 Places 层。Rooms 层(社区检测)留 SP-D2。

## 1. 目标与产出

**输入**:SP-A 管线里的细化骨架 `gvd`(bool)+ 距离场 `dist_m` + `vmin` + `voxel_size`(全部已在 `run_gvd` 内存中,不重算)。

**产出**:
1. `<output>.graph.json`:`{nodes:[{id, pos_m:[x,y,z], clearance_m, degree, type}], edges:[{a, b, length_m}]}`
2. 可视化 `.vxw`:原几何 + 骨架(青,= 边)+ **节点标记**(品红 3×3×3 发光方块,= places 节点)→ 现有 Godot 查看器直接看。
3. stdout 统计:骨架体素 / key 体素 / 合并后节点 / 边 / 节点类型分布。

**不做**:Rooms 聚类(SP-D2)、增量(SP-B)、图优化/回环。

## 2. 算法(标准 skeleton→graph,用现成库)

```
1 度数:kernel=3³ 全 1(中心 0),ndimage.convolve(gvd) → 每骨架体素的 26 邻居数 = degree
2 key 体素:gvd & (degree != 2)   端点(deg1)/交叉(deg≥3)/孤立(deg0)
3 合并 key 簇:DBSCAN(eps=merge_radius_vox, min_samples=1) on key 坐标 → 每簇一个 place 节点
      节点 = 簇质心(贴到最近骨架体素);clearance = 簇内 max dist_m;
      type: 有 deg≥3 → junction;否则有 deg1 → endpoint;deg0 → isolated
4 边来源两类(去重):
   a 链:chain = gvd & ~key;label(chain, 26 连通);每段膨胀 1 ∩ key → 触及的簇;
        恰好 2 簇 → 一条边,length = 段体素数 × voxel_size;非 2 簇记 warn 跳过
   b 直连:不同簇的 key 体素若 26 相邻 → 簇间一条边,length = voxel_size
```

**为什么用 DBSCAN 合并**:细化后噪声处仍有局部"毛球"(一坨 deg≥3),不合并会炸出几十个伪节点。DBSCAN(eps≈3 体素=0.15m)把一坨塌成一个 place,和 Hydra 的 node-merge 同义。sklearn 已在依赖里。

## 3. 模块结构

| 文件 | 职责 | CLI |
|---|---|---|
| `m3_adapter/gvd_graph.py` | **纯算法**:`skeleton_to_graph(gvd, dist_m, voxel_size, merge_radius_m=0.15) -> (nodes, edges)`。节点用 dense-idx,长度/clearance 用米。无 IO。 | 否 |
| `m3_adapter/gvd_to_vxw.py`(扩展) | `--graph` flag:thin 后调 skeleton_to_graph → 写 `.graph.json`(idx→米用 vmin)+ 节点标记叠进可视化 .vxw | 是 |

`skeleton_to_graph` 返回:
- `nodes`: list of `{"idx":(i,j,k), "clearance_m":float, "degree":int, "type":str}`(dense idx;调用方加 vmin 转米)
- `edges`: list of `(node_a_id, node_b_id, length_m)`

## 4. 可视化约定

复用 SP-A 的发光叠加机制(overlay_gvd_into_world 已有 append-material 模式):
- 骨架体素:沿用青色 `gvd_skeleton`(= 边的视觉)
- 节点:新材质 `place_node`,品红 (255,0,255) 发光,每节点画 3×3×3 实心块(放大以可见)

## 5. 验证(算法工作,合成测试)

`tests/test_gvd_graph.py`:
- **十字骨架**(一条横线 + 一条竖线交于中心):→ 1 junction(deg4)+ 4 endpoints + 4 edges。
- **直线骨架**:→ 2 endpoints + 1 edge,length ≈ 线长。
- **毛球合并**:中心一坨 3×3×3 全 True + 四条伸出的线 → DBSCAN 把坨合成 1 junction(不是 27 个),4 edges。
- 真实数据:apt 骨架跑通,节点/边数量级合理(节点几百量级,非几万),`.graph.json` 合法,可视化 .vxw 能加载。

## 6. 风险

| 风险 | 缓解 |
|---|---|
| 毛球炸出伪节点 | DBSCAN 合并;merge_radius 可调 |
| 链触及 ≠2 簇(对角邻接歧义) | 跳过+warn;不影响主图 |
| 边长用体素数近似(对角偏短) | v1 接受;length 仅作权重粗值 |
| 节点几百但仍杂 | 是质量信号——告诉我们骨架够不够干净,正道 TSDF 是否必要 |

## 7. 完成判定

节点/边图写出 + 品红节点叠在青骨架上、Godot 截图可见拓扑结构 → SP-D v1 达成。图质量(节点是否落在房间/路口的合理位置)是对 SP-A 骨架质量的检验,也决定要不要上正道。

## 8. SP-D v1 结果(2026-06-15)

apt 骨架(band1.0+denoise30+thin,38165 体素)抽图:**3486 节点 / 3754 边**,30.6s。
类型:junction 1914 / endpoint 1564 / isolated 8。`out/apt_graph.graph.json` 合法。
可视化 `out/apt_graph.png`:品红节点标记叠在青骨架上,拓扑结构清晰可见。

**踩坑(已修)**:首版 chain-edge 检测对每个连通分量做全数组 `binary_dilation`,O(分量数×4800万)在真实数据上挂死。改为按分量分组 + 局部 26 邻居查表(O(链体素数)),30s 跑通。合成测试语义不变仍过。

**判读:管线端到端跑通 ✅,但图过密(3486 节点,理应几十)。** 1564 个 endpoint = 骨架噪声毛刺的死端,1914 junction = 缠结分叉点。**节点数 = SP-A 骨架质量的直接信号**:图只能和上游骨架一样干净。

**SP-D v1 收口:✅ 达成**(图+可视化交付)。**下一步候选:**
- **图后处理剪枝**:剪掉短死端边(endpoint 边长 < 阈值的毛刺)+ 更激进合并 → 节点数量级直降,廉价。
- **正道 TSDF**:从根上消噪声毛刺(要 rosbag)。
- **SP-D2 Rooms**:在(剪枝后)places 图上社区检测分房间。
