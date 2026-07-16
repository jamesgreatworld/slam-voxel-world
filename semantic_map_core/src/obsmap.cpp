// obsmap.cpp — 忠实移植自 m3_adapter/obsmap.py::integrate_frame。
// 定步射线采样(step=s*vs), miss/hit 每帧各去重加一次, 之后统一 clamp;
// 语义 Boyer-Moore 逐命中点投票(按点序, 与 Python 一致)。
#include "semantic_map_core/obsmap.hpp"

#include <cmath>
#include <unordered_set>

namespace smc {

ObsMap::ObsMap(int nx, int ny, int nz, std::array<long, 3> vmin, float voxel_size)
    : nx_(nx), ny_(ny), nz_(nz), vmin_(vmin), voxel_size_(voxel_size),
      logodds_(static_cast<size_t>(size()), 0.0f),
      sem_label_(static_cast<size_t>(size()), 0),
      sem_count_(static_cast<size_t>(size()), 0) {}

void ObsMap::vote_label(long id, uint8_t L) {
  uint8_t cur = sem_label_[id];
  uint16_t cnt = sem_count_[id];
  if (cur == L) {
    if (cnt < 65535) sem_count_[id] = cnt + 1;
  } else if (cnt == 0) {
    sem_label_[id] = L;
    sem_count_[id] = 1;
  } else {
    sem_count_[id] = cnt - 1;  // Boyer-Moore: 对立票
  }
}

void ObsMap::integrate_frame(const float origin[3], const float* pts, int n,
                             const uint8_t* labels, float free_margin) {
  const double vs = voxel_size_;
  std::unordered_set<long> miss_set, hit_set;
  miss_set.reserve(n * 8);
  hit_set.reserve(n);

  for (int i = 0; i < n; ++i) {
    const double px1 = pts[3 * i], py1 = pts[3 * i + 1], pz1 = pts[3 * i + 2];
    const double dx = px1 - origin[0], dy = py1 - origin[1], dz = pz1 - origin[2];
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len <= free_margin) continue;  // keep = len > free_margin
    const double ux = dx / len, uy = dy / len, uz = dz / len;
    // misses: 定步采样, step < len - free_margin
    for (int s = 0;; ++s) {
      const double step = s * vs;
      if (step >= len - free_margin) break;
      const long vi = static_cast<long>(std::floor((origin[0] + step * ux) / vs)) - vmin_[0];
      const long vj = static_cast<long>(std::floor((origin[1] + step * uy) / vs)) - vmin_[1];
      const long vk = static_cast<long>(std::floor((origin[2] + step * uz) / vs)) - vmin_[2];
      if (in_bounds(vi, vj, vk)) miss_set.insert(idx(vi, vj, vk));
    }
    // hit endpoint
    const long hi = static_cast<long>(std::floor(px1 / vs)) - vmin_[0];
    const long hj = static_cast<long>(std::floor(py1 / vs)) - vmin_[1];
    const long hk = static_cast<long>(std::floor(pz1 / vs)) - vmin_[2];
    if (in_bounds(hi, hj, hk)) hit_set.insert(idx(hi, hj, hk));
  }

  last_hit_cells = (long)hit_set.size();
  last_miss_cells = (long)miss_set.size();
  for (long id : miss_set) logodds_[id] += L_MISS;
  for (long id : hit_set) logodds_[id] += L_HIT;
  for (long id : miss_set)
    logodds_[id] = logodds_[id] < L_MIN ? L_MIN : (logodds_[id] > L_MAX ? L_MAX : logodds_[id]);
  for (long id : hit_set)
    logodds_[id] = logodds_[id] < L_MIN ? L_MIN : (logodds_[id] > L_MAX ? L_MAX : logodds_[id]);

  if (labels) {
    for (int i = 0; i < n; ++i) {
      const uint8_t L = labels[i];
      if (L == 0) continue;
      const double px1 = pts[3 * i], py1 = pts[3 * i + 1], pz1 = pts[3 * i + 2];
      const double dx = px1 - origin[0], dy = py1 - origin[1], dz = pz1 - origin[2];
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <= free_margin) continue;
      const long hi = static_cast<long>(std::floor(px1 / vs)) - vmin_[0];
      const long hj = static_cast<long>(std::floor(py1 / vs)) - vmin_[1];
      const long hk = static_cast<long>(std::floor(pz1 / vs)) - vmin_[2];
      if (in_bounds(hi, hj, hk)) vote_label(idx(hi, hj, hk), L);
    }
  }
}

std::vector<uint8_t> ObsMap::occupancy_mask() const {
  std::vector<uint8_t> m(static_cast<size_t>(size()));
  for (size_t i = 0; i < m.size(); ++i) m[i] = logodds_[i] >= OCC_THR ? 1 : 0;
  return m;
}

std::vector<uint8_t> ObsMap::observed_free_mask() const {
  std::vector<uint8_t> m(static_cast<size_t>(size()));
  for (size_t i = 0; i < m.size(); ++i) m[i] = logodds_[i] <= FREE_THR ? 1 : 0;
  return m;
}

}  // namespace smc
