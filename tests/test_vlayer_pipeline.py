import numpy as np
from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.overlay import VoxelDelta
from m3_adapter.vlayer.pipeline import LcContext, run_pipeline


class _Gen:
    def __init__(self, gid, stage, deps, coord):
        self.id = gid; self.stage = stage; self.depends_on = deps
        self.default_binding = "persistent"; self._coord = coord
    def run(self, ctx):
        return [VoxelDelta(self._coord, "add", 3, self.id, "persistent")]


def test_runs_in_stage_then_dep_order():
    m = ObsMap.new((4, 4, 4), np.zeros(3, np.int64), 0.1)
    g_late = _Gen("late", 2, ["early"], (2, 0, 0))
    g_early = _Gen("early", 1, [], (1, 0, 0))
    ov = run_pipeline(m, [g_late, g_early])
    order = [d.generator for d in ov.voxels]
    assert order == ["early", "late"]


def test_dep_within_same_stage():
    m = ObsMap.new((4, 4, 4), np.zeros(3, np.int64), 0.1)
    a = _Gen("a", 1, ["b"], (0, 0, 0))   # a depends on b -> b first
    b = _Gen("b", 1, [], (1, 0, 0))
    ov = run_pipeline(m, [a, b])
    assert [d.generator for d in ov.voxels] == ["b", "a"]
