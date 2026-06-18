# 多分辨率粗化 L1 导出 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or executing-plans. Checkbox steps.

**Goal:** 导出时把 compose 出的细占据降采样成粗块(默认 0.2m),得 Minecraft 厚块感 + 并薄层/双层 + 吞小洞,门窗保留;L0 不变。

**Architecture:** 新增 `m3_adapter/vlayer/coarsen.py::downsample_occupancy`;`obsmap_to_completed_vxw` 加 `coarsen_to_m` 参数。纯后处理,不动 compose/generators。

**Tech Stack:** numpy、pytest。fine vs=0.05;factor=round(coarsen_to_m/vs)。

**Setup:** main。commit 结尾 `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`。

---

### Task 1: downsample_occupancy + 导出集成(单任务,TDD)

**Files:** Create `m3_adapter/vlayer/coarsen.py`; Modify `m3_adapter/vlayer/export.py`; Test `tests/test_coarsen.py`。

完整参考实现 + 测试见对应 subagent 指令(本计划由控制器派发 subagent 实现)。要点:
1. 写 `tests/test_coarsen.py`(薄层→实心粗块、空区→空、语义众数、vs_c/vmin_c 坐标、负 vmin 对齐、集成 coarsen_to_m=0.2 体素数骤减 + coarsen_to_m=None 回归)。
2. 跑 → FAIL。
3. 实现 `downsample_occupancy(occ, sem, vmin, vs, factor, min_fine=1)`:每轴按 `vmin%factor` 对齐 + 补齐 → reshape 分块 → `occ_c = blockcount >= min_fine` → 块内占据细格 sem 众数(按 label argmax,~21 类)→ `vs_c=vs*factor`,`vmin_c=(vmin-pad_lo)//factor`。
4. `obsmap_to_completed_vxw` 加 `coarsen_to_m=None`:设值则 compose 后 downsample 再 occupancy_to_vxw(粗 vmin/vs);None 时现行为不变。
5. 跑 → PASS;全量 vlayer 回归绿。
6. commit `feat(vlayer): multi-resolution coarsen export (downsample_occupancy + coarsen_to_m)`。

### Task 2: 公寓粗化冒烟 + 目视
- `obsmap_to_completed_vxw(m, 'out/apt_coarse.vxw', generators=[SlabFill(3,'floor'),SlabFill(4,'ceiling'),WallFill()], palette=pal, coarsen_to_m=0.2)` → 打印体素数(应远小于细网格 ~30万)。
- Godot 打开 `apt_coarse.vxw` 目视:厚块 MC 感、薄层消失、门窗仍开。

---

## 后续
- min_fine / 去噪调参;众数性能优化;粗化与碰撞网格、mc_item 模型的尺度协调。
