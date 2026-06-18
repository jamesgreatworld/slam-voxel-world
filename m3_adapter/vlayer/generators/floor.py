"""floor_fill 已并入 SlabFill;保留 FloorFill 名以兼容现有调用/测试。"""
from m3_adapter.vlayer.generators.slab import SlabFill


def FloorFill(close_radius: int = 2):
    return SlabFill(3, "floor", close_radius=close_radius)
