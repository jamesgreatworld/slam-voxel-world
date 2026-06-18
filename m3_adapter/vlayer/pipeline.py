"""Lc 先验推理流水线:按 (stage, depends_on) 拓扑序运行 generator 插件,
把各插件产出的结构差异累积进一个 Overlay。引擎本身无持久状态。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from m3_adapter.vlayer.overlay import Overlay


class Generator(Protocol):
    id: str
    stage: int
    depends_on: list
    default_binding: str
    def run(self, ctx: "LcContext") -> list: ...   # -> list[VoxelDelta]


@dataclass
class LcContext:
    obsmap: object        # 只读 L0(约定:generator 不得修改)
    overlay: Overlay


def _toposort(generators):
    """先按 stage 升序,再在依赖约束下做稳定 Kahn 排序。"""
    by_id = {g.id: g for g in generators}
    ordered = []
    remaining = list(generators)
    while remaining:
        ready = [g for g in remaining
                 if all(dep not in by_id or by_id[dep] not in remaining
                        for dep in g.depends_on)]
        if not ready:
            raise ValueError("generator dependency cycle: "
                             + ",".join(g.id for g in remaining))
        ready.sort(key=lambda g: g.stage)      # 同就绪集里低 stage 先
        nxt = ready[0]
        ordered.append(nxt)
        remaining.remove(nxt)
    return ordered


def run_pipeline(obsmap, generators) -> Overlay:
    ctx = LcContext(obsmap=obsmap, overlay=Overlay())
    for g in _toposort(generators):
        ctx.overlay.voxels.extend(g.run(ctx))
    return ctx.overlay
