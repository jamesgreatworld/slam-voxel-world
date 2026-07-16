// objects.hpp — 语义物体层: 逐类 DBSCAN 聚类 + bbox 邻近合并 + shape 特征。
// 移植自 m3_adapter/gvd/objects.py。
#pragma once
#include <array>
#include <cstdint>
#include <set>
#include <vector>

namespace smc {

struct ObjectNode {
  int ci, cj, ck;      // 质心 dense voxel idx(np.round 后取整)
  int label;           // super_id
  int voxel_count;
  int bmin[3], bmax[3];
  std::array<double, 3> feat;  // shape 描述子(features.py::extract_features["shape"])
};

// occ/sem: 行主序占据/语义栅格。structure_labels: 不算物体的结构类。
// voxel_size: 用于 shape 特征(米)。
std::vector<ObjectNode> extract_objects(const uint8_t* occ, const uint8_t* sem, int nx,
                                        int ny, int nz,
                                        const std::set<int>& structure_labels,
                                        int min_voxels = 20,
                                        double cluster_eps_voxels = 3.0,
                                        int min_samples = 5, int merge_gap_voxels = 2,
                                        double voxel_size = 0.1);

// 通用 DBSCAN(整数 3D 点), 语义与 sklearn.DBSCAN.fit_predict 一致(移植 dbscan_labels)。
std::vector<int> dbscan_labels_int3(const std::vector<std::array<int, 3>>& pts,
                                    double eps, int min_samples);

}  // namespace smc
