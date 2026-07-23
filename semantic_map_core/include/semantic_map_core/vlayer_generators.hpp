// vlayer_generators.hpp — 结构先验 generator(移植自 m3_adapter/vlayer/generators/*)。
#pragma once
#include <string>

#include "semantic_map_core/vlayer.hpp"

namespace smc {

// stage-1: 水平板(floor/ceiling)补洞 + 实心化, 多层逐峰。移植自 slab.py::SlabFill。
class SlabFill : public Generator {
 public:
  // side: "floor" | "ceiling"
  SlabFill(int label, const std::string& side, double thickness_m = 0.6,
           double level_gap_m = 0.5, int close_radius = 2);
  std::vector<VoxelDelta> run(const MapView& m) const override;

 private:
  int label_;
  bool floor_;          // true=floor, false=ceiling
  double thickness_m_, level_gap_m_;
  int close_radius_;
};

// stage-1: 封堵被占据近乎包围的未知针孔。移植自 occlusion.py::OcclusionFill。
class OcclusionFill : public Generator {
 public:
  OcclusionFill(int min_neighbors = 5, int fill_label = 19);
  std::vector<VoxelDelta> run(const MapView& m) const override;

 private:
  int min_neighbors_, fill_label_;
};

// stage-1: 墙(label=19)当轴对齐竖直平面, 投影峰认领 + 面内闭运算 + 朝墙背增厚。
// 移植自 wall.py::WallFill。
class WallFill : public Generator {
 public:
  WallFill(double thickness_m = 0.10, int min_wall_cells = 20, int min_height = 4,
           int min_run = 4, int close_radius = 2);
  std::vector<VoxelDelta> run(const MapView& m) const override;

 private:
  double thickness_m_;
  int min_wall_cells_, min_height_, min_run_, close_radius_;
};

// 墙平面检测(WallFill / OpeningCarve 共用): 投影计数的局部峰(>=min_cells, 紧邻合并取最大)。
std::vector<int> projection_peaks(const std::vector<long>& count, int min_cells);

}  // namespace smc
