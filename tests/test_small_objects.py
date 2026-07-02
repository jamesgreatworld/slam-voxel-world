"""小物体点级实例通道:点云(不经栅格)→ 聚类 → OBB → 实体。"""
import numpy as np

from m3_adapter.small_objects import SmallObjectBuffer


def _cluster_points(centre, n=200, spread=0.03, seed=0):
    rng = np.random.default_rng(seed)
    return np.asarray(centre) + rng.normal(0, spread, size=(n, 3))


def test_two_vases_become_two_entities():
    buf = SmallObjectBuffer(labels=frozenset({6}))
    pts_a = _cluster_points([1.0, 0.8, 2.0])
    pts_b = _cluster_points([3.0, 0.8, 2.0], seed=1)
    pts = np.vstack([pts_a, pts_b])
    labs = np.full(len(pts), 6, np.uint8)
    cols = np.tile([200, 60, 40], (len(pts), 1)).astype(np.uint8)
    buf.add_frame(pts, labs, cols)
    ents = buf.finalize({6: "vase"})
    assert len(ents) == 2
    e = sorted(ents, key=lambda x: x["position"][0])[0]
    assert abs(e["position"][0] - 1.0) < 0.05
    assert e["label_name"] == "vase"
    assert e["custom_meta"]["small"] is True
    assert e["custom_meta"]["rgb"] == [200, 60, 40]
    assert 0.03 <= e["bbox_dims"][0] <= 0.5          # 杯子量级,远小于一个粗体素


def test_sub_voxel_object_still_captured():
    """3cm 的钥匙:栅格(5cm)根本装不下,点级通道照样出实体。"""
    buf = SmallObjectBuffer(labels=frozenset({6}))
    pts = _cluster_points([0.5, 0.75, 0.5], n=400, spread=0.008)   # ~3cm 物体
    buf.add_frame(pts, np.full(len(pts), 6, np.uint8))
    ents = buf.finalize({6: "key"}, min_points=30)
    assert len(ents) == 1
    assert max(ents[0]["bbox_dims"]) < 0.08


def test_sparse_noise_dropped_and_big_labels_ignored():
    buf = SmallObjectBuffer(labels=frozenset({6}))
    rng = np.random.default_rng(2)
    noise = rng.uniform(0, 5, size=(10, 3))                        # 零星错标
    buf.add_frame(noise, np.full(10, 6, np.uint8))
    wall_pts = _cluster_points([2, 1, 2], n=300)
    buf.add_frame(wall_pts, np.full(300, 19, np.uint8))            # 墙:非小物体类
    assert buf.finalize({6: "vase"}, min_points=50) == []


def test_multi_frame_accumulation_dedup():
    """同一只杯子被 10 帧反复观测:去重后仍是一件。"""
    buf = SmallObjectBuffer(labels=frozenset({6}))
    for i in range(10):
        buf.add_frame(_cluster_points([1, 1, 1], n=80, seed=i),
                      np.full(80, 6, np.uint8))
    ents = buf.finalize({6: "vase"})
    assert len(ents) == 1
