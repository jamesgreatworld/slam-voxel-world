# main.gd 拆分设计(违规 #8 还债)

> 日期:2026-06-11。纯结构重构,无任何行为变化、不加新功能。
> 背景:architecture.md §10.7 违规 #8 —— main.gd 涨到 822 行,混 7 类职责。
> 后续工作(增量建图 SP-B、热重载 SP-C)都要动 world 加载路径,先拆干净。

## 目标

main.gd 收缩为 ~120 行编排瘦壳,只做:@onready 引用、`_ready` 按序 init 各 controller、全局快捷键 `_input` 路由、pause 开/关。

## 新文件(均在 `godot_viewer/`)

| 新文件 | 类型 | 职责 | 迁出内容 |
|---|---|---|---|
| `boot_config.gd` | RefCounted | 纯解析 CLI user args → 配置字段,无副作用 | `_ready` 里 ~50 行 arg 循环 + 全部 `_test_*` 字段 |
| `test_hooks_controller.gd` | Node | 持有 boot_config,世界就绪后执行全部 `--test-*` 钩子 | delete/rotate/grab/undo/duplicate/snapshot/toggle-behavior/hide-voxel/day-night 钩子块(~160 行) |
| `world_session.gd` | Node | 世界加载/原地切换/重载/备份/世界快照;完成后发 `world_loaded(world, path)` 信号 | `_load_world_in_place`、`_on_reload_world`、`_on_load_world_requested`、`_on_save_world_backup`、`_save_world_snapshot`、`_copy_dir_recursive`、`_resolve` |
| `edit_session.gd` | Node | 统一撤销栈(voxel + entity 条目路由)、磁盘补丁、material 选择、编辑模式开关 | `_undo_stack`/`_push_undo`/`undo_last_edit`、`_on_voxel_destroyed/_placed`、`_patch_voxel_on_disk`、`_on_material_*`、`_on_toggle_edit_mode` |
| `environment_controller.gd` | Node | Day/Night 全部常量与切换 | `_DAY_*`/`_NIGHT_*` 常量、`_apply_day/night_mode`、`_toggle_day_night`、`_get_sky_material` |
| `entity_subsystem.gd` | Node | 实体子系统装配:8 个动态节点创建+互连+context bar wiring | `_ready` 里 entity_renderer/item_picker/placer/selector/edit/inspector/toolbar/context_bar 装配块(~110 行) |

## 设计规则

- 依赖方向:main → controllers;controllers 之间**不互相 import**,跨模块协作经信号或由 main 注入引用(遵守 architecture.md §8.5)。
- `world_session.world_loaded` 信号是世界切换后的统一再绑定入口:edit_session、entity_subsystem、hud 等监听它刷新自身,替代现在 `_load_world_in_place` 里手工逐个 re-init 的写法 —— 这是 SP-C 热重载的接口预留(热重载 = 同一信号的增量版)。
- 撤销栈保留"单一栈、条目带 op 前缀路由"的现状语义,只搬家不改造。
- 暂停菜单 wiring:菜单信号在 main 连接,但 handler 转发到对应 controller。

## 验证(从简,用户确认)

最后做一次总体冒烟:headless 跑 baseline_20cm + apt 世界各一次(带 `--snapshot`),外加一次 `--test-toggle-day-night`;PNG 生成、无 push_error、`_editor_selftest` / `_writer_selftest` 通过即算过。

## 范围外

不加新功能;违规 #10(硬编码 item pack 路径)、#11(重复常量)另开小提交;不动 voxel_renderer / entity_* 各文件内部。

## 完成后

回填 architecture.md §10.7(#8 标记已修),然后进入路线图下一项(建图质量 / 自研 Hydra SP-A)。
