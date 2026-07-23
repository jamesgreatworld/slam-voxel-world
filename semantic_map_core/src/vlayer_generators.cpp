// vlayer_generators.cpp — 见 vlayer_generators.hpp。逐行移植 generators/*.py。
#include "semantic_map_core/vlayer_generators.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace smc {

// 2D(nx×nz)4-连通(十字)膨胀/腐蚀, 越界=0(scipy 默认 border_value=0)。
static void dilate2d(const std::vector<uint8_t>& a, int NX, int NZ, std::vector<uint8_t>& out) {
  out.assign((size_t)NX * NZ, 0);
  for (int x = 0; x < NX; ++x)
    for (int z = 0; z < NZ; ++z) {
      long id = (long)x * NZ + z;
      bool v = a[id] || (x > 0 && a[id - NZ]) || (x + 1 < NX && a[id + NZ]) ||
               (z > 0 && a[id - 1]) || (z + 1 < NZ && a[id + 1]);
      out[id] = v ? 1 : 0;
    }
}
static void erode2d(const std::vector<uint8_t>& a, int NX, int NZ, std::vector<uint8_t>& out) {
  out.assign((size_t)NX * NZ, 0);
  for (int x = 0; x < NX; ++x)
    for (int z = 0; z < NZ; ++z) {
      long id = (long)x * NZ + z;
      bool v = a[id] && (x > 0 ? (bool)a[id - NZ] : false) && (x + 1 < NX ? (bool)a[id + NZ] : false) &&
               (z > 0 ? (bool)a[id - 1] : false) && (z + 1 < NZ ? (bool)a[id + 1] : false);
      out[id] = v ? 1 : 0;
    }
}
static void closing2d(std::vector<uint8_t>& a, int NX, int NZ, int iters) {
  std::vector<uint8_t> t;
  for (int i = 0; i < iters; ++i) { dilate2d(a, NX, NZ, t); a.swap(t); }
  for (int i = 0; i < iters; ++i) { erode2d(a, NX, NZ, t); a.swap(t); }
}

// _cluster_levels: 按 gap 把出现过的 y 分簇, 每簇取计数峰值 y(平局取最小 y)。
static std::vector<int> cluster_levels(const std::vector<int>& ys, int gap, int ny) {
  if (ys.empty()) return {};
  std::vector<long> bc(ny, 0);
  for (int y : ys) bc[y]++;
  std::vector<int> u;
  for (int y = 0; y < ny; ++y) if (bc[y]) u.push_back(y);
  std::vector<int> peaks;
  std::vector<int> cur{u[0]};
  auto flush = [&]() {
    int best = cur[0]; long bcnt = bc[cur[0]];
    for (int y : cur) if (bc[y] > bcnt) { bcnt = bc[y]; best = y; }  // 第一个最大(y 升序)
    peaks.push_back(best);
  };
  for (size_t i = 1; i < u.size(); ++i) {
    if (u[i] - cur.back() > gap) { flush(); cur.clear(); }
    cur.push_back(u[i]);
  }
  flush();
  return peaks;
}

SlabFill::SlabFill(int label, const std::string& side, double thickness_m,
                   double level_gap_m, int close_radius)
    : label_(label), floor_(side == "floor"), thickness_m_(thickness_m),
      level_gap_m_(level_gap_m), close_radius_(close_radius) {
  id = std::string("slab_fill_") + side;
  stage = 1;
  default_binding = "persistent";
}

std::vector<VoxelDelta> SlabFill::run(const MapView& m) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  const double vs = m.voxel_size;
  // surf = occ & (sem == label)
  std::vector<uint8_t> surf((size_t)nx * ny * nz, 0);
  std::vector<int> ys;
  bool any = false;
  for (long i = 0; i < (long)nx * ny * nz; ++i)
    if (m.occ[i] && m.sem[i] == label_) { surf[i] = 1; any = true; ys.push_back((int)((i / nz) % ny)); }
  std::vector<VoxelDelta> out;
  if (!any) return out;

  int T = std::max(1, (int)std::ceil(thickness_m_ / vs - 1e-9));
  int gap = std::max(1, (int)std::lround(level_gap_m_ / vs));

  for (int Y : cluster_levels(ys, gap, ny)) {
    int w = std::max(gap, 3);
    int ylo = std::max(0, Y - w), yhi = std::min(ny, Y + w + 1);
    // foot(x,z) = free[:,ylo:yhi,:].any(y) | surf[:,Y-1:Y+2,:].any(y)
    int slo = std::max(0, Y - 1), shi = std::min(ny, Y + 2);
    std::vector<uint8_t> foot((size_t)nx * nz, 0);
    for (int x = 0; x < nx; ++x)
      for (int z = 0; z < nz; ++z) {
        bool f = false;
        for (int y = ylo; y < yhi && !f; ++y) if (m.free[m.id(x, y, z)]) f = true;
        for (int y = slo; y < shi && !f; ++y) if (surf[m.id(x, y, z)]) f = true;
        foot[(long)x * nz + z] = f ? 1 : 0;
      }
    if (close_radius_ > 0) closing2d(foot, nx, nz, close_radius_);

    for (int x = 0; x < nx; ++x)
      for (int z = 0; z < nz; ++z) {
        if (!foot[(long)x * nz + z]) continue;
        for (int k = 0; k < T; ++k) {
          int yy = floor_ ? Y - k : Y + k;
          if (yy < 0 || yy >= ny) break;
          long id = m.id(x, yy, z);
          if (m.free[id]) break;                 // 不穿观测开口
          if (m.occ[id]) { if (k == 0) continue; else break; }
          VoxelDelta d;
          d.idx[0] = x; d.idx[1] = yy; d.idx[2] = z;
          d.op = DeltaOp::ADD; d.sem = label_;
          d.generator = "slab_fill"; d.binding = default_binding;
          out.push_back(d);
        }
      }
  }
  return out;
}

}  // namespace smc
