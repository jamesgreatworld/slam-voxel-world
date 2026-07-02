"""vlayer.objects:物体解耦(③)、模型抽取(④)、摆放解算(物理一致性)。"""
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


def _extract(m, mc_map, **kw):
    return extract_object_models(
        m.occupancy_mask(), m.semantic_grid(), m.vmin, m.voxel_size,
        {5: "chair", 11: "lamp", 7: "couch"}, mc_map, **kw)


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
    ents = _extract(m, {5: "chair"}, min_voxels=10)
    assert len(ents) == 2
    e = ents[0]
    assert e["custom_meta"]["mc_item"] == "chair"
    assert e["rotation"] == [0.0, 0.0, 0.0, 1.0]
    assert e["bbox_dims"] == pytest.approx([0.15, 0.15, 0.15])
    assert e["position"][0] == pytest.approx((2 + 4 + 1) * 0.5 * 0.05)


def test_min_voxels_filters_specks():
    m = _make_map()
    m.logodds[18, 1, 18] = m.l_max          # 1 格语义噪声
    m.sem_label[18, 1, 18] = 5
    ents = _extract(m, {5: "chair"}, min_voxels=10)
    assert len(ents) == 2                    # 噪声被 min_voxels 滤除


def test_unmapped_labels_get_obb_entities():
    """没有预制模型的类也产实体(OBB 盒渲染),只是不带 mc_item。"""
    m = _make_map()
    ents = _extract(m, {}, min_voxels=10)    # 空 mc_item map
    assert len(ents) == 2
    assert all("mc_item" not in e["custom_meta"] for e in ents)


def test_ceiling_lamp_detected_and_not_floor_lamp_model():
    """贴天花板、不落地的灯 → mount=ceiling,且不套落地灯模型。"""
    m = ObsMap.new((20, 24, 20), vmin=(0, 0, 0), voxel_size=0.05)
    m.logodds[:, 0, :] = m.l_max;  m.sem_label[:, 0, :] = 3      # 地板
    m.logodds[:, 20, :] = m.l_max; m.sem_label[:, 20, :] = 4     # 天花板
    m.logodds[8:11, 17:20, 8:11] = m.l_max                        # 吸顶灯(贴顶)
    m.sem_label[8:11, 17:20, 8:11] = 11
    ents = _extract(m, {11: "lamp"}, min_voxels=10)
    assert len(ents) == 1
    assert ents[0]["custom_meta"].get("mount") == "ceiling"
    assert "mc_item" not in ents[0]["custom_meta"]


def test_fragmented_object_merged_into_one():
    """同标签、AABB 相隔 ≤2 格的两个碎片 → 合并成一件(不再互相嵌套)。"""
    m = ObsMap.new((20, 10, 20), vmin=(0, 0, 0), voxel_size=0.05)
    m.logodds[:, 0, :] = m.l_max; m.sem_label[:, 0, :] = 3
    m.logodds[2:5, 1:4, 2:5] = m.l_max;  m.sem_label[2:5, 1:4, 2:5] = 5
    m.logodds[7:10, 1:4, 2:5] = m.l_max; m.sem_label[7:10, 1:4, 2:5] = 5  # 相隔2格
    ents = _extract(m, {5: "chair"}, min_voxels=10)
    assert len(ents) == 1                    # 合并为一件
    assert ents[0]["bbox_dims"][0] == pytest.approx(0.40)   # 跨两碎片的宽度


def test_floor_snap_lifts_embedded_object():
    """陷进地板的物体被吸附抬起:底面 = 地板顶面。"""
    m = _make_map()
    ents = _extract(m, {5: "chair"}, min_voxels=10)
    for e in ents:
        bottom = e["position"][1] - e["bbox_dims"][1] / 2
        assert bottom == pytest.approx(0.05, abs=1e-6)   # 地板顶面 y=1格*0.05


def test_rgb_mean_color_attached():
    m = _make_map()
    rgb = np.zeros(m.logodds.shape + (3,), np.uint8)
    cnt = np.zeros(m.logodds.shape, np.uint8)
    rgb[2:5, 1:4, 2:5] = (200, 40, 40); cnt[2:5, 1:4, 2:5] = 3
    ents = extract_object_models(
        m.occupancy_mask(), m.semantic_grid(), m.vmin, m.voxel_size,
        {5: "chair"}, {5: "chair"}, min_voxels=10, rgb=rgb, rgb_count=cnt)
    with_color = [e for e in ents if "rgb" in e["custom_meta"]]
    assert len(with_color) == 1
    assert with_color[0]["custom_meta"]["rgb"] == [200, 40, 40]


def test_overlapping_different_objects_separated():
    """两件不同物体的盒重叠 → 小的被推开,不再互相嵌入。"""
    m = ObsMap.new((30, 10, 30), vmin=(0, 0, 0), voxel_size=0.05)
    m.logodds[:, 0, :] = m.l_max; m.sem_label[:, 0, :] = 3
    m.logodds[5:13, 1:4, 5:13] = m.l_max; m.sem_label[5:13, 1:4, 5:13] = 5    # 大椅
    m.logodds[11:14, 1:4, 6:9] = m.l_max; m.sem_label[11:14, 1:4, 6:9] = 11   # 灯嵌进椅子
    ents = _extract(m, {5: "chair", 11: "lamp"}, min_voxels=8)
    assert len(ents) == 2
    a, b = ents
    pa, ha = np.array(a["position"]), np.array(a["bbox_dims"]) / 2
    pb, hb = np.array(b["position"]), np.array(b["bbox_dims"]) / 2
    overlap = (ha + hb) - np.abs(pa - pb)
    assert not (overlap > 1e-6).all(), "解算后不应再有 AABB 互嵌"
