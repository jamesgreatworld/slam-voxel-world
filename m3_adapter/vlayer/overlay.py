"""L1 结构覆盖层:相对 L0 观测的稀疏体素差异(add/remove/replace),
带 provenance(generator/binding)。永不修改 L0。

持久化:overlay.npz 存并列数组(idx/op/sem/gen_id/binding_id),
overlay.json 存 id→字符串图例。"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np

_OPS = ("add", "remove", "replace")


@dataclass
class VoxelDelta:
    idx: tuple          # (x, y, z) 整数网格索引(同 ObsMap 数组索引)
    op: str             # "add" | "remove" | "replace"
    sem: int = 0        # super_id(add/replace 用)
    generator: str = "manual"
    binding: str = "persistent"   # live | persistent | independent


@dataclass
class Overlay:
    voxels: list = field(default_factory=list)   # list[VoxelDelta]

    def add_voxel(self, idx, sem=0, generator="manual",
                  binding="persistent", op="add") -> None:
        assert op in _OPS, f"bad op {op}"
        self.voxels.append(VoxelDelta(
            tuple(int(v) for v in idx), op, int(sem), str(generator), str(binding)))

    def save(self, npz_path, json_path) -> None:
        n = len(self.voxels)
        idx = np.zeros((n, 3), np.int32)
        op = np.zeros(n, np.uint8)
        sem = np.zeros(n, np.uint8)
        gen_legend, bind_legend = {}, {}
        gen_id = np.zeros(n, np.uint16)
        bind_id = np.zeros(n, np.uint8)
        for i, d in enumerate(self.voxels):
            idx[i] = d.idx
            op[i] = _OPS.index(d.op)
            sem[i] = d.sem
            gen_id[i] = gen_legend.setdefault(d.generator, len(gen_legend))
            bind_id[i] = bind_legend.setdefault(d.binding, len(bind_legend))
        np.savez_compressed(npz_path, idx=idx, op=op, sem=sem,
                            gen_id=gen_id, bind_id=bind_id)
        with open(json_path, "w") as f:
            json.dump({"gen_legend": {v: k for k, v in gen_legend.items()},
                       "bind_legend": {v: k for k, v in bind_legend.items()}}, f)

    @classmethod
    def load(cls, npz_path, json_path) -> "Overlay":
        d = np.load(npz_path)
        with open(json_path) as f:
            leg = json.load(f)
        gl = {int(k): v for k, v in leg["gen_legend"].items()}
        bl = {int(k): v for k, v in leg["bind_legend"].items()}
        ov = cls()
        for i in range(len(d["op"])):
            ov.voxels.append(VoxelDelta(
                tuple(int(v) for v in d["idx"][i]),
                _OPS[int(d["op"][i])],
                int(d["sem"][i]),
                gl[int(d["gen_id"][i])],
                bl[int(d["bind_id"][i])]))
        return ov
