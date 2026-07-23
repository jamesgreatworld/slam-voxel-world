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
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;

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
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;

 private:
  int min_neighbors_, fill_label_;
};

// stage-1: 墙(label=19)当轴对齐竖直平面, 投影峰认领 + 面内闭运算 + 朝墙背增厚。
// 移植自 wall.py::WallFill。
class WallFill : public Generator {
 public:
  WallFill(double thickness_m = 0.10, int min_wall_cells = 20, int min_height = 4,
           int min_run = 4, int close_radius = 2);
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;

 private:
  double thickness_m_;
  int min_wall_cells_, min_height_, min_run_, close_radius_;
};

// 墙平面检测(WallFill / OpeningCarve 共用): 投影计数的局部峰(>=min_cells, 紧邻合并取最大)。
std::vector<int> projection_peaks(const std::vector<long>& count, int min_cells);

// stage-2: RoofCap —— 天花板(label=4)层的观测缺口 binary_fill_holes 补洞(不填 free 开口)。
// 移植自 roof.py::RoofCap。depends_on slab_fill_ceiling。
class RoofCap : public Generator {
 public:
  RoofCap(double level_gap_m = 0.5, int band_cells = 2);
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;

 private:
  double level_gap_m_;
  int band_cells_;
};

// stage-2: OpeningCarve —— 门/窗开口(墙平面上的 free 连通域)拟合成矩形/拱形并挖穿(remove)。
// 读 acc 拿 wall_fill 补墙格一并挖。移植自 opening.py::OpeningCarve。
class OpeningCarve : public Generator {
 public:
  OpeningCarve(double min_area_m2 = 0.35, double max_area_m2 = 4.0, double max_extent_m = 2.6,
               double enclosure_min = 0.5, double thickness_m = 0.10, int min_wall_cells = 20,
               int min_height = 4, int min_run = 4, double trim_fill = 0.3);
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;

 private:
  double min_area_m2_, max_area_m2_, max_extent_m_, enclosure_min_, thickness_m_, trim_fill_;
  int min_wall_cells_, min_height_, min_run_;
};

// stage-2: PlaneRegularize —— 形状先验(墙=矩形)× 观测 log-odds 的概率规整:
// 矩形内深处未观测缺格补上, 矩形外孤立小碎块删掉; observed-free 永不覆盖。
// 移植自 regularize.py::PlaneRegularize。需 MapView.logodds。
class PlaneRegularize : public Generator {
 public:
  PlaneRegularize(double k_per_cell = 0.3, double prior_cap = 3.0, double trim_fill = 0.3,
                  int min_wall_cells = 20, int min_height = 4, int min_run = 4,
                  double support_m = 0.4, double keep_comp_m2 = 0.09);
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;

 private:
  double k_, cap_, trim_fill_, support_m_, keep_comp_m2_;
  int min_wall_cells_, min_height_, min_run_;
};

// stage-1: 楼梯(label 15)向下实心化, 停在观测面/free/max_depth/地面下界(floor 最低 y)。
// 移植自 stairs.py::StairsFill。
class StairsFill : public Generator {
 public:
  explicit StairsFill(double max_depth_m = 1.5);
  std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const override;

 private:
  double max_depth_m_;
};

}  // namespace smc
