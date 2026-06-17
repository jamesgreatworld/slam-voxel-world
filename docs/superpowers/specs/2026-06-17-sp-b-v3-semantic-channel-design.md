# SP-B v3:ObsMap 语义通道设计

> 日期:2026-06-17。目标:让增量观测地图不只几何(占据/自由),还**每格带语义类**(墙/地/椅/桌…),使 live 地图能像原 apt 一样按语义着色,并为未来"语义 places 节点"铺路。

## 1. 思想
ObsMap 当前只有 log-odds 几何。v3 加**并行语义通道**:每占据格记录被观测到的语义类(多数表决)。复用 uhumans2_to_vxw 已有机械:`load_color_to_super_id`(seg RGB→super_id)、`seg_pixels_to_labels`、`load_label_space`、`build_palette`。

## 2. 数据 / 接口
ObsMap 加两个 uint16 通道(与 logodds 同形状,稀疏占据所以压缩后小):
- `sem_label`:当前胜出标签(uint8,0=unknown)。
- `sem_count`:该胜出标签的累计观测次数(uint16,做多数表决用)。
简化多数表决(避免存所有标签计数):**胜者计数法**——hit 时若新标签==当前胜者则 count+1;否则 count-1,若 count 归 0 则换标签、count=1。这是经典 Boyer-Moore majority 流式近似,O(1)/格/帧,够用。

接口:
- `integrate_frame(origin_m, points_m, point_labels=None, free_margin_m)`:沿用几何更新;若给 `point_labels`(每 surface 点的 super_id),对 hit 格做上面的胜者计数更新 `sem_label/sem_count`。
- `semantic_grid()` -> uint8 label per cell(=sem_label,仅 occupancy_mask 内有意义)。

## 3. 驱动 / 导出
- `uhumans2_stream`:加 `--semantic`(默认开?需读 seg 主题,慢一点)。读 seg + camera_info,`seg_pixels_to_labels` 得每点 super_id,传给 `integrate_frame(point_labels=...)`。复用 uhumans2_to_vxw 的 seg 读取(同 bag,topic `/tesse/seg_cam/rgb/image_raw`)。
- `obsmap_export.occupancy_to_vxw`:加可选 `semantic_grid` + `palette`:每占据格写 `semantic_id = sem_label`,material 用语义调色板(复用 uhumans2 `build_palette`),而非纯 concrete。这样 live.vxw 渲染成语义色。

## 4. 验证
- ObsMap 语义合成单测:同格多帧给标签 A,A,B,A → 胜者=A;给 A,B,C,B,B → 胜者=B(Boyer-Moore 行为)。
- save/load round-trip 含语义通道不变。
- 真实:stream --semantic 跑一段 → 导出 live.vxw 渲染应见墙/地/家具语义色(非全灰)。

## 5. 范围外
语义 places 节点(图节点带 dominant 语义)、语义先验约束(Phase ④)、实例分割——后续。本期只把"每格语义"加进增量地图 + 语义着色导出。

## 6. v3 达成结果(2026-06-17,commits 2efc1a2/c232dbb)
- ObsMap 加 `sem_label`(uint8)+`sem_count`(uint16),`integrate_frame(point_labels=)` 做 Boyer-Moore 流式多数表决,`semantic_grid()`,save/load 含语义(旧 npz 向后兼容)。
- `obsmap_export.occupancy_to_vxw(semantic_grid=, palette=)` 语义着色导出;`uhumans2_stream --semantic` 读 seg(复用 uhumans2_to_vxw 全套 seg 机械)→ 传 labels → 导出语义色 live.vxw。
- 82 测试绿。注:hydra cfg yaml/csv 在外部工作区,单测用内联 palette;真实 `--semantic` 跑用 uhumans2 原配置(已存在)。
**v3 达成 ✅。SP-B 1-2-3 全部完成。**
