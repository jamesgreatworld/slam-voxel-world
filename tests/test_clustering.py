"""m3_adapter.clustering.dbscan_labels — sklearn DBSCAN 的 scipy 等价实现。"""
import numpy as np

from m3_adapter.clustering import dbscan_labels


def test_two_separated_blobs():
    a = np.array([[0, 0], [1, 0], [0, 1]], float)
    b = a + 100.0
    lab = dbscan_labels(np.vstack([a, b]), eps=2.0, min_samples=3)
    assert lab[0] == lab[1] == lab[2] == 0
    assert lab[3] == lab[4] == lab[5] == 1


def test_noise_is_minus_one():
    pts = np.array([[0, 0], [1, 0], [0, 1], [50, 50]], float)
    lab = dbscan_labels(pts, eps=2.0, min_samples=3)
    assert lab[3] == -1
    assert set(lab[:3]) == {0}


def test_min_samples_1_is_connected_components():
    pts = np.array([[0.0], [1.0], [2.1], [10.0]])
    lab = dbscan_labels(pts, eps=1.05, min_samples=1)
    # 0-1 相连;2.1 与 1 相距 1.1 > eps 断开;10 独立
    assert lab[0] == lab[1]
    assert lab[2] != lab[0] and lab[3] != lab[2]
    assert -1 not in lab                     # min_samples=1:无噪声

def test_border_point_joins_cluster():
    # 密核 3 点 + 1 个只挨着核的边界点(自身邻居数不足)
    pts = np.array([[0, 0], [0.5, 0], [0, 0.5], [1.3, 0]], float)
    lab = dbscan_labels(pts, eps=1.0, min_samples=3)
    assert lab[3] == lab[0] != -1            # border 并入邻核簇

def test_empty_and_single():
    assert dbscan_labels(np.empty((0, 3)), 1.0, 2).size == 0
    assert dbscan_labels(np.array([[1.0, 2.0]]), 1.0, 1).tolist() == [0]
    assert dbscan_labels(np.array([[1.0, 2.0]]), 1.0, 2).tolist() == [-1]


def test_integer_voxel_coords():
    """体素整型坐标(gvd/uhumans2 的实际用法)。eps=2 → 桥接 ≤2 格间隙。"""
    cells = np.array([[0, 0, 0], [1, 0, 0], [3, 0, 0], [9, 9, 9], [9, 9, 10]])
    lab = dbscan_labels(cells, eps=2.0, min_samples=2)
    assert lab[0] == lab[1] == lab[2]
    assert lab[3] == lab[4] != lab[0]
