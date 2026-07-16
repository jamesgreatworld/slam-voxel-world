// surface.hpp — 2D 地面可通行层(surface places)。移植自 m3_adapter/gvd/surface.py。
// 纯函数: 输入 2D 可通行 bool 栅格 -> 8 连通聚类成区域 + 经门相邻边(测地膨胀, 不穿墙)。
#pragma once
#include <cstdint>
#include <utility>
#include <vector>

namespace smc {

struct SurfRegion {
  int id;             // 0-based, 保留顺序
  double cx, cy;      // 世界米质心
  double area_m2;
  int cell_count;
  int label_id;       // 原始连通分量 label(1-based, 与 scipy 一致)
};

struct SurfaceResult {
  std::vector<SurfRegion> regions;
  std::vector<std::pair<int, int>> edges;  // (id_a, id_b), a<b, 经门相邻
  std::vector<int> label_img;              // (ny*nx) 每格区域 label(0=非可走/丢弃)
};

// walkable/blocked: 行优先 (ny行, nx列), blocked 可为 nullptr(全可穿)。
SurfaceResult cluster_surface_places(const uint8_t* walkable, int ny, int nx,
                                     double origin_x, double origin_y, double res,
                                     int min_cells = 15, double door_bridge_m = 0.9,
                                     const uint8_t* blocked = nullptr);

// counts -> (walkable bool, costmap int8: -1 未知/0 可走/100 障碍)。
void occupancy_from_counts(const int* surf_hits, const int* obst_hits, int n,
                           int surf_min, int obst_min,
                           std::vector<uint8_t>& walkable, std::vector<int8_t>& cost);

}  // namespace smc
