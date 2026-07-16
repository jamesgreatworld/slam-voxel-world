// field.hpp — ESDF / GVD 体素场(C++ 移植自 m3_adapter/gvd/field.py)。
// 3D 欧氏距离变换用 Felzenszwalb 可分离精确算法 + 最近障碍索引传播(feature transform)。
#pragma once
#include <cstdint>
#include <vector>

namespace smc {

// occ: uint8 占据栅格, 行主序 (i*ny+j)*nz+k。
// 输出 dist: 到最近障碍的欧氏距离(米); parent: 最近障碍的扁平索引(feature transform)。
void compute_esdf(const uint8_t* occ, int nx, int ny, int nz, float voxel_size,
                  std::vector<float>& dist, std::vector<long>& parent);

// GVD 体素 = 到 >=2 个不同障碍等距的 free 格(父间距判据, 移植自 extract_gvd)。
std::vector<uint8_t> extract_gvd(const uint8_t* free, const float* dist,
                                 const long* parent, int nx, int ny, int nz,
                                 float voxel_size, float d_min = 0.20f,
                                 float theta_sep = 0.40f);

// 丢弃小于 min_component 体素的连通障碍块(26 连通)。
std::vector<uint8_t> denoise_occupancy(const uint8_t* occ, int nx, int ny, int nz,
                                       int min_component);

// 3D 骨架细化(Lee 1994), 逐行移植 skimage.skeletonize。thin_gvd。
std::vector<uint8_t> thin_gvd(const uint8_t* gvd, int nx, int ny, int nz);

// thin_gvd 上一次运行的内部剖析(找瓶颈用)。
struct ThinProfile {
  long iters = 0;         // 收敛所需的 border-pass 总次数
  long candidates = 0;    // 累计候选点数
  double find_ms = 0;     // 全网格扫描 find_candidates 累计耗时
  double recheck_ms = 0;  // 复检删点累计耗时
};
ThinProfile last_thin_profile();

}  // namespace smc
