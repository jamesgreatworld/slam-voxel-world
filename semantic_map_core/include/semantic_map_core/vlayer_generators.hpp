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

}  // namespace smc
