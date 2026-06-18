import numpy as np
from m3_adapter.uhumans2_to_vxw import extract_entities


def test_extracted_entity_has_observed_provenance():
    n = 60
    rng = np.random.default_rng(0)
    vc = (rng.random((n, 3)) * 3).astype(np.int64) + np.array([10, 10, 10])
    lbl = np.full(n, 5, dtype=np.int64)          # 5 = chair, in _OBJECT_LABELS
    names = {5: "chair"}
    ents, _keep = extract_entities(vc, lbl, 0.1, names,
                                   dbscan_eps_voxels=3.0, min_samples=5)
    assert len(ents) >= 1
    prov = ents[0].custom_meta.get("provenance")
    assert prov == {"generator": "cluster", "binding": "live"}
