// obsmap.hpp — 3D log-odds 占据+语义栅格(C++ 移植自 m3_adapter/obsmap.py)。
// 纯 C++, 无 ROS/Eigen 依赖。每帧 integrate_frame 是热点路径。
#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace smc {

class ObsMap {
 public:
  // 与 Python 默认一致
  static constexpr float L_HIT = 0.85f;
  static constexpr float L_MISS = -0.40f;
  static constexpr float L_MIN = -2.0f;
  static constexpr float L_MAX = 3.5f;
  static constexpr float OCC_THR = 0.85f;   // >= -> 占据(与 Python 一致)
  static constexpr float FREE_THR = -0.4f;  // <= -> 观测自由

  ObsMap(int nx, int ny, int nz, std::array<long, 3> vmin, float voxel_size);

  // origin/pts: 世界米坐标(与 Python 同系)。pts 为 n*3 连续 float。
  // labels: 每点 super_id(可空); 0=未知跳过。free_margin: 短于此的射线丢弃。
  void integrate_frame(const float origin[3], const float* pts, int n,
                       const uint8_t* labels = nullptr, float free_margin = 0.10f);

  // 占据格 bool(logodds > OCC_THR), 长度 nx*ny*nz, 行主序 (i*ny+j)*nz+k。
  std::vector<uint8_t> occupancy_mask() const;
  std::vector<uint8_t> observed_free_mask() const;  // logodds < FREE_THR

  long last_hit_cells = 0;   // 上一帧去重后命中格数(调试)
  long last_miss_cells = 0;  // 上一帧去重后 miss 格数(调试)
  long size() const { return static_cast<long>(nx_) * ny_ * nz_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  const std::array<long, 3>& vmin() const { return vmin_; }
  float voxel_size() const { return voxel_size_; }
  const std::vector<uint8_t>& sem_label() const { return sem_label_; }

 private:
  inline long idx(int i, int j, int k) const {
    return (static_cast<long>(i) * ny_ + j) * nz_ + k;
  }
  inline bool in_bounds(int i, int j, int k) const {
    return i >= 0 && i < nx_ && j >= 0 && j < ny_ && k >= 0 && k < nz_;
  }
  void hit_logodds(long id);
  void miss_logodds(long id);
  void vote_label(long id, uint8_t label);

  int nx_, ny_, nz_;
  std::array<long, 3> vmin_;
  float voxel_size_;
  std::vector<float> logodds_;
  std::vector<uint8_t> sem_label_;   // Boyer-Moore 胜出 super_id(0=未知)
  std::vector<uint16_t> sem_count_;  // Boyer-Moore 计数(与 Python uint16 一致)
};

}  // namespace smc
