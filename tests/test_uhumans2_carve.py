"""Unit tests for uhumans2_carve.carve_frame — no rosbag required."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))
sys.path.insert(0, str(_PROJECT_ROOT / "m3_adapter"))

from uhumans2_carve import carve_frame  # noqa: E402


def test_single_ray_marks_interior_not_surface():
    grid = np.zeros((20, 5, 5), dtype=bool)
    vmin = np.zeros(3, dtype=np.int64)
    vs = 1.0
    # ray from (0.5,2.5,2.5) to (15.5,2.5,2.5) along +x, margin 1.0
    carve_frame(grid, vmin, vs, np.array([0.5, 2.5, 2.5]),
                np.array([[15.5, 2.5, 2.5]]), free_margin_m=1.0)
    # cells along the ray interior are free
    assert grid[3, 2, 2] and grid[10, 2, 2]
    # the surface end (x~15) within margin is NOT free
    assert not grid[15, 2, 2]
    # off-ray cells untouched
    assert not grid[3, 0, 0]


def test_out_of_bounds_safe():
    grid = np.zeros((5, 5, 5), dtype=bool)
    # ray pointing far outside the small grid: must not crash, must not mark oob
    carve_frame(grid, np.zeros(3, np.int64), 1.0, np.array([2.5, 2.5, 2.5]),
                np.array([[100.0, 2.5, 2.5]]), free_margin_m=0.5)
    assert grid[3, 2, 2]  # in-bounds part still marked
