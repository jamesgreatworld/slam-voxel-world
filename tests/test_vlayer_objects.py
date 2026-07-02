"""vlayer.objects:物体解耦(③)与 scipy-CC 模型抽取(④)。"""
import numpy as np
import pytest

from m3_adapter.obsmap import ObsMap
from m3_adapter.vlayer.objects import decouple_objects, extract_object_models


def _make_map():
    """20x10x20 场景:y=0 地板(label 3)+ 两把分离的椅子(label 5)。"""
    m = ObsMap.new((20, 10, 20), vmin=(0, 0, 0), voxel_size=0.05)
    m.logodds[:, 0, :] = m.l_max
    m.sem_label[:, 0, :] = 3
    for x0, z0 in [(2, 2), (12, 12)]:          # 3x3x3 椅子 ×2
        m.logodds[x0:x0 + 3, 1:4, z0:z0 + 3] = m.l_max
        m.sem_label[x0:x0 + 3, 1:4, z0:z0 + 3] = 5
    return m


def test_decouple_removes_objects_keeps_structure():
    m = _make_map()
    struct, obj = decouple_objects(m)
    assert int(obj.sum()) == 2 * 27
    # 结构图中不再有物体标签,地板仍在;原图未被修改(L0 只读)
    assert not (struct.sem_label == 5).any()
    assert (struct.occupancy_mask()[:, 0, :]).all()
    assert (m.sem_label == 5).sum() == 2 * 27


def test_extract_models_one_per_component():
    m = _make_map()
    ents = extract_object_models(
        m.occupancy_mask(), m.semantic_grid(), m.vmin, m.voxel_size,
        {5: "chair"}, {5: "chair"}, min_voxels=10)
    assert len(ents) == 2
    e = ents[0]
    assert e["custom_meta"]["mc_item"] == "chair"
    assert e["rotation"] == [0.0, 0.0, 0.0, 1.0]
    assert e["bbox_dims"] == pytest.approx([0.15, 0.15, 0.15])
    # 位置 = AABB 中心(米):x0=2,3 格宽 → (2+4+1)/2*0.05
    assert ents[0]["position"][0] == pytest.approx((2 + 4 + 1) * 0.5 * 0.05)


def test_min_voxels_filters_specks():
    m = _make_map()
    m.logodds[18, 1, 18] = m.l_max          # 1 格语义噪声
    m.sem_label[18, 1, 18] = 5
    ents = extract_object_models(
        m.occupancy_mask(), m.semantic_grid(), m.vmin, m.voxel_size,
        {5: "chair"}, {5: "chair"}, min_voxels=10)
    assert len(ents) == 2                    # 噪声被 min_voxels 滤除


def test_unmapped_labels_skipped():
    m = _make_map()
    ents = extract_object_models(
        m.occupancy_mask(), m.semantic_grid(), m.vmin, m.voxel_size,
        {5: "chair"}, {}, min_voxels=10)     # 空 mc_item map
    assert ents == []
