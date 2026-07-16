// features.hpp — 物体形状描述子(数据关联用)。移植自 m3_adapter/gvd/features.py。
// 当前只含 ShapeFeature(旋转不变: PCA 主轴标准差, 米, 降序)+ feature_cost。
#pragma once
#include <array>
#include <vector>

namespace smc {

// ShapeFeature.extract: cells (M,3) dense voxel 坐标 -> (σ0,σ1,σ2) 降序主轴标准差*voxel_size。
// M<3 返回 {0,0,0}。
std::array<double, 3> shape_feature(const std::vector<std::array<int, 3>>& cells,
                                    double voxel_size);

// feature_cost: DEFAULT_FEATURES 只有 shape(weight=2.0), distance=L2。
// = 2.0 * ||a-b||。两侧 shape 向量恒长度 3 且恒存在。
double feature_cost(const std::array<double, 3>& a, const std::array<double, 3>& b);

}  // namespace smc
