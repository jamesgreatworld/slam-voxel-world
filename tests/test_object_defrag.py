import numpy as np
from m3_adapter.gvd.objects import extract_objects


def test_dbscan_bridges_small_gap_into_one_object():
    occ = np.zeros((40, 40, 40), dtype=bool); sem = np.zeros((40,40,40), np.uint8)
    # one chair split by a 1-voxel gap (seat at x 10..14, legs at x 16..18) — NOT
    # touching (gap at x=15), so strict connected-components would give 2 objects
    occ[10:15, 10, 10] = True; sem[10:15, 10, 10] = 5
    occ[16:19, 10, 10] = True; sem[16:19, 10, 10] = 5
    # strict adjacency: 2 components. DBSCAN eps=3 bridges the gap -> 1 object
    objs = extract_objects(occ, sem, 0.1, np.zeros(3, np.int64),
                           label_names={5: "chair"}, min_voxels=3,
                           cluster_eps_voxels=3.0, min_samples=1, merge_gap_voxels=2)
    assert len(objs) == 1
    assert objs[0].voxel_count == 8   # 5 + 3 voxels merged


def test_bbox_merge_joins_close_fragments():
    occ = np.zeros((50, 50, 50), dtype=bool); sem = np.zeros((50,50,50), np.uint8)
    # two table fragments 2 voxels apart (beyond DBSCAN eps=1.5 but within merge_gap=3)
    occ[10:14, 10, 10] = True; sem[10:14, 10, 10] = 16
    occ[17:21, 10, 10] = True; sem[17:21, 10, 10] = 16   # gap x14..16 = 3 voxels
    objs = extract_objects(occ, sem, 0.1, np.zeros(3, np.int64),
                           label_names={16: "table"}, min_voxels=3,
                           cluster_eps_voxels=1.5, min_samples=1, merge_gap_voxels=3)
    assert len(objs) == 1    # bbox merge joins them even though DBSCAN didn't


def test_distinct_objects_stay_separate():
    occ = np.zeros((60, 60, 60), dtype=bool); sem = np.zeros((60,60,60), np.uint8)
    occ[10:14, 10, 10] = True; sem[10:14, 10, 10] = 5    # chair A
    occ[45:49, 45, 45] = True; sem[45:49, 45, 45] = 5    # chair B far away
    objs = extract_objects(occ, sem, 0.1, np.zeros(3, np.int64),
                           label_names={5: "chair"}, min_voxels=3,
                           cluster_eps_voxels=3.0, min_samples=1, merge_gap_voxels=2)
    assert len(objs) == 2    # far-apart objects NOT merged
