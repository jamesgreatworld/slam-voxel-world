// graph.hpp — Places 图: Hydra 式压缩抽图(替代 thin+skeleton_to_graph)。
// 把稠密 GVD 体素量化到 compression 粗网格 -> 每格一个 place; 相邻成员格连边。
// 不做形态学细化。参考 Hydra CompressionGraphExtractor。
#pragma once
#include <cstdint>
#include <vector>

namespace smc {

struct PlaceNode {
  double x, y, z;      // 世界米坐标(粗格内最大 clearance 成员的位置)
  float clearance_m;   // 该 place 的净空(粗格内 max dist)
  int members;         // 粗格内 GVD 体素数
  int degree = 0;      // 图度数(构图后填)
};

struct PlaceEdge {
  int a, b;            // 节点索引
  float length_m;      // 两节点间距
};

struct PlacesGraph {
  std::vector<PlaceNode> nodes;
  std::vector<PlaceEdge> edges;
};

// gvd/dist: extract_gvd/compute_esdf 的输出(行主序)。vmin: 世界体素偏移。
// compression_m: 稀疏图分辨率(Hydra 默认 0.5m)。min_node_dist_m: 净空低于此的 GVD
// 体素不参与(门口太窄不当 place 中心, Hydra min_node_distance_m=0.3)。
PlacesGraph compress_gvd_graph(const uint8_t* gvd, const float* dist, int nx, int ny,
                               int nz, const long vmin[3], float voxel_size,
                               float compression_m = 0.5f,
                               float min_node_dist_m = 0.3f,
                               float node_merge_m = 0.3f);

// ---- skeleton_to_graph: 逐格移植自 m3_adapter/gvd/graph.py(bit 对齐目标)----
struct SkelNode {
  int i, j, k;         // dense voxel idx(簇代表 = 最近质心的成员)
  float clearance_m;   // 簇内 max dist
  int degree;          // 图度数
  int type;            // 0=junction 1=passthrough 2=endpoint 3=isolated 4=loop
};
struct SkelEdge { int a, b; float length_m; };
struct SkelGraph {
  std::vector<SkelNode> nodes;
  std::vector<SkelEdge> edges;
};

SkelGraph skeleton_to_graph(const uint8_t* gvd, const float* dist, int nx, int ny,
                            int nz, float voxel_size, float merge_radius_m = 0.15f);

// rooms: clearance 切门口(移植自 gvd_live._partition_rooms_clearance)。
// 返回每节点的房间号(0..k-1)。纯确定性。
std::vector<int> partition_rooms_clearance(const SkelGraph& g,
                                           float door_clearance_m = 0.85f,
                                           int min_room_nodes = 8);

}  // namespace smc
