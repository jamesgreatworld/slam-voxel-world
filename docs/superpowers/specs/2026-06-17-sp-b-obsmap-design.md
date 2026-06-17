# SP-B:持久化增量观测地图 ObsMap 设计

> 日期:2026-06-17。用户核心需求(memory: incremental-map-requirement):**可保存/恢复、在已存地图上持续增量增删改观测**。
> 技术 = log-odds 占据图(OctoMap/Voxblox 模型)。这是 SP-B 地基,v1 流式 / v2 实时 ESDF 都建其上。

## 1. ObsMap(`m3_adapter/obsmap.py`,几何 log-odds 地图)
每格存 log-odds 置信度(float16 网格,默认 dense над bbox)。hit(表面)增、miss(射线穿过)减,clamp。三态由阈值分:occupied(l≥occ_thr)/ observed-free(l≤free_thr)/ unknown(中间,含从未观测的 0)。

```python
class ObsMap:
    # 字段: logodds float16 (nx,ny,nz); vmin int64[3]; voxel_size float; 参数
    # 参数(OctoMap 风格): l_hit=+0.85, l_miss=-0.4, l_min=-2.0, l_max=3.5,
    #                      occ_thr=+0.85(~prob0.7), free_thr=-0.4
    @classmethod
    def new(cls, shape, vmin, voxel_size): ...        # 全 unknown(0)
    @classmethod
    def load(cls, path): ...                           # np.load obsmap.npz
    def save(self, path): ...                          # savez_compressed(logodds,vmin,voxel_size,params)
    def integrate_frame(self, origin_m, points_m, free_margin_m=0.10):
        # 沿每条 origin->point 射线:途经格 += l_miss(clamp);终点格 += l_hit(clamp)
        # 复用 uhumans2_carve 的采样,但 miss 更新内部、hit 更新端点
    def occupancy_mask(self): return self.logodds >= self.occ_thr   # bool
    def observed_free_mask(self): return self.logodds <= self.free_thr  # bool
```
**增删改的实现**:integrate_frame 用 += / clamp。某格原是占据(l 高),后续多帧 miss 把它压到 ≤free_thr → occupancy_mask 不再含它(删除);新表面 hit 抬高 → 新增;反复观测 → 修正。**这是二值做不到的。**

**持久化**:`<vxw_dir>/obsmap.npz`。logodds 大部分=0(未观测),压缩后小。

## 2. 驱动 `m3_adapter/uhumans2_stream.py`(续建)
```
uhumans2_stream.py <bag_dir> <vxw_dir> [--start-frame S] [--end-frame E] [--pixel-stride] ...
  obsmap = ObsMap.load(<vxw_dir>/obsmap.npz) if exists else ObsMap.new(网格来自 densify(.vxw))
  逐帧[S:E]: integrate_frame(origin, points)
  obsmap.save(...)
  打印: 本次帧数、occupancy/free 格数(对比加载时,体现"长大")
```
跑两次(0-500、再 500-1000)→ 地图累积。**这就是"存档恢复续建"。**

## 3. GVD 消费
ObsMap 派生 occ+free 喂 GVD。最省:`uhumans2_stream` 结束时把 `occupancy_mask`/`observed_free_mask` 各导出一份(observed_free.npz 复用现路径;occupancy 可导一份几何 .vxw 供渲染,语义暂缺)。或 pipeline 直接读 obsmap.npz(后续)。本期先导出复用现路径。

## 4. 验证
- ObsMap 合成单测:integrate 一条射线 → 终点格 occupied、途经格 free;**再 integrate 多帧 miss 同一占据格 → 翻成 free(删除验证)**;save/load round-trip 不变。
- 真实:stream 帧 0-500 存;load 续 500-1000 存;occupancy/free 格数随帧增长;GVD 复跑结果合理。

## 5. 范围外
v1 流式实时循环、v2 波前增量 ESDF、语义通道、推流 Godot——都建在 ObsMap 上,后续。

## 6. SP-B 地基达成结果(2026-06-17,commits be92a94/0653a36)

`m3_adapter/obsmap.py` ObsMap + `m3_adapter/uhumans2_stream.py` 驱动,70+ 测试绿(含**删除验证**:占据格被反复 carve free → log-odds 翻负 → 从 occupied 变 free)。

**真实数据续建演示(存档恢复):**
| | seg1 帧0-300 | seg2 **resume** 帧300-600 |
|---|---|---|
| occupied | 0→73,417 | 73,417→**115,137** |
| observed-free | 0→1,910,128 | 1,910,128→**2,456,345** |

seg2 加载 seg1 存的 obsmap.npz(761KB,baseline 精确=73417),续 integrate,两量均增长 → **存盘/恢复/续建/增量增删改全部验证**。GVD 消费 obsmap 派生的 observed-free:8002 体素 / 481 节点 / **20 房**,与直接 carve 一致干净(out/apt_sb.png)。

**SP-B 地基达成 ✅**:可持久化、可恢复、可增量增删改的 log-odds 观测地图。一份 log-odds 网格统一了占据+自由(用户"二为一"),per-cell 置信度(用户"weight 属性"),支持删除(二值做不到)。

**下一步**:v1 流式实时循环(边收边推 Godot,§12)| v2 波前增量 ESDF(逐帧实时,FIESTA/Voxblox)| 语义通道(ObsMap 加 semantic 层)。
