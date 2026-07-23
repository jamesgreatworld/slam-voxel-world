// vlayer_generators.cpp — 见 vlayer_generators.hpp。逐行移植 generators/*.py。
#include "semantic_map_core/vlayer_generators.hpp"

#include <algorithm>
#include <cmath>
#include <set>
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

// 2D binary_fill_holes(默认 4-conn): 洞 = 背景中从边界不可达的连通域。
static void fill_holes2d(const std::vector<uint8_t>& fg, int NX, int NZ, std::vector<uint8_t>& out) {
  std::vector<uint8_t> reach((size_t)NX * NZ, 0);
  std::vector<long> q;
  auto push = [&](int x, int z) {
    long id = (long)x * NZ + z;
    if (!fg[id] && !reach[id]) { reach[id] = 1; q.push_back(id); }
  };
  for (int x = 0; x < NX; ++x) { push(x, 0); push(x, NZ - 1); }
  for (int z = 0; z < NZ; ++z) { push(0, z); push(NX - 1, z); }
  for (size_t h = 0; h < q.size(); ++h) {
    long id = q[h]; int x = id / NZ, z = id % NZ;
    if (x > 0) push(x - 1, z);
    if (x + 1 < NX) push(x + 1, z);
    if (z > 0) push(x, z - 1);
    if (z + 1 < NZ) push(x, z + 1);
  }
  out.assign((size_t)NX * NZ, 0);
  for (long i = 0; i < (long)NX * NZ; ++i) out[i] = (fg[i] || !reach[i]) ? 1 : 0;
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

std::vector<VoxelDelta> SlabFill::run(const MapView& m, const Overlay& acc) const {
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

// ===================== OcclusionFill =====================
OcclusionFill::OcclusionFill(int min_neighbors, int fill_label)
    : min_neighbors_(min_neighbors), fill_label_(fill_label) {
  id = "occlusion_fill"; stage = 1; default_binding = "persistent";
}
std::vector<VoxelDelta> OcclusionFill::run(const MapView& m, const Overlay& acc) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  std::vector<VoxelDelta> out;
  bool anyocc = false;
  for (long i = 0; i < (long)nx * ny * nz && !anyocc; ++i) if (m.occ[i]) anyocc = true;
  if (!anyocc) return out;
  // argwhere(cand) 行主序: x 外 y 中 z 内
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z) {
        long id = m.id(x, y, z);
        if (m.occ[id] || m.free[id]) continue;  // unknown = !occ & !free
        int nb = 0;
        if (x > 0) nb += m.occ[id - (long)ny * nz];
        if (x + 1 < nx) nb += m.occ[id + (long)ny * nz];
        if (y > 0) nb += m.occ[id - nz];
        if (y + 1 < ny) nb += m.occ[id + nz];
        if (z > 0) nb += m.occ[id - 1];
        if (z + 1 < nz) nb += m.occ[id + 1];
        if (nb < min_neighbors_) continue;
        VoxelDelta d;
        d.idx[0] = x; d.idx[1] = y; d.idx[2] = z;
        d.op = DeltaOp::ADD; d.sem = fill_label_;
        d.generator = "occlusion_fill"; d.binding = default_binding;
        out.push_back(d);
      }
  return out;
}

// ===================== WallFill =====================
std::vector<int> projection_peaks(const std::vector<long>& count, int min_cells) {
  const int n = (int)count.size();
  std::vector<int> peaks;
  for (int i = 0; i < n; ++i) {
    if (count[i] < min_cells) continue;
    int lo = std::max(0, i - 1), hi = std::min(n, i + 2);
    long mx = 0;
    for (int j = lo; j < hi; ++j) mx = std::max(mx, count[j]);
    if (count[i] < mx) continue;                    // 非局部峰
    if (!peaks.empty() && i - peaks.back() <= 1) {   // 紧邻上一峰 -> 取最大
      if (count[i] > count[peaks.back()]) peaks.back() = i;
      continue;
    }
    peaks.push_back(i);
  }
  return peaks;
}

WallFill::WallFill(double thickness_m, int min_wall_cells, int min_height, int min_run,
                   int close_radius)
    : thickness_m_(thickness_m), min_wall_cells_(min_wall_cells), min_height_(min_height),
      min_run_(min_run), close_radius_(close_radius) {
  id = "wall_fill"; stage = 1; default_binding = "persistent";
}

std::vector<VoxelDelta> WallFill::run(const MapView& m, const Overlay& acc) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  const double vs = m.voxel_size;
  const int WALL = 19;
  std::vector<uint8_t> wall((size_t)nx * ny * nz, 0);
  bool any = false;
  for (long i = 0; i < (long)nx * ny * nz; ++i)
    if (m.occ[i] && m.sem[i] == WALL) { wall[i] = 1; any = true; }
  std::vector<VoxelDelta> out;
  if (!any) return out;
  int T = std::max(1, (int)std::ceil(thickness_m_ / vs - 1e-9));

  std::vector<long> xcount(nx, 0), zcount(nz, 0);
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z)
        if (wall[m.id(x, y, z)]) { xcount[x]++; zcount[z]++; }

  // plane(a,b)、free_lo/hi(a,b): const_axis="x" -> a=y,b=z; "z" -> a=x,b=y
  auto fill_plane = [&](bool axis_x, int c0) {
    int NA = axis_x ? ny : nx, NB = axis_x ? nz : ny;
    auto W = [&](int a, int b) {           // wall at plane(c0)
      return axis_x ? wall[m.id(c0, a, b)] : wall[m.id(a, b, c0)];
    };
    auto Focc = [&](int c, int a, int b) { return axis_x ? m.occ[m.id(c, a, b)] : m.occ[m.id(a, b, c)]; };
    auto Ffree = [&](int c, int a, int b) { return axis_x ? m.free[m.id(c, a, b)] : m.free[m.id(a, b, c)]; };
    // plane.sum
    long psum = 0; int a0 = NA, a1 = -1, b0 = NB, b1 = -1;
    for (int a = 0; a < NA; ++a)
      for (int b = 0; b < NB; ++b)
        if (W(a, b)) { psum++; a0 = std::min(a0, a); a1 = std::max(a1, a); b0 = std::min(b0, b); b1 = std::max(b1, b); }
    if (psum < min_wall_cells_) return;
    int height = axis_x ? (a1 - a0 + 1) : (b1 - b0 + 1);
    int run = axis_x ? (b1 - b0 + 1) : (a1 - a0 + 1);
    if (height < min_height_ || run < min_run_) return;
    int SA = a1 - a0 + 1, SB = b1 - b0 + 1;
    std::vector<uint8_t> sub((size_t)SA * SB, 0), filled;
    for (int a = a0; a <= a1; ++a)
      for (int b = b0; b <= b1; ++b) sub[(long)(a - a0) * SB + (b - b0)] = W(a, b) ? 1 : 0;
    filled = sub;
    closing2d(filled, SA, SB, close_radius_);   // 2D 4-conn closing
    // 朝墙背增厚: 远离 observed_free 多的一侧(房间)
    long flo = 0, fhi = 0;
    for (int a = 0; a < NA; ++a)
      for (int b = 0; b < NB; ++b)
        if (W(a, b)) {
          if (axis_x) { if (c0 - 1 >= 0) flo += m.free[m.id(c0 - 1, a, b)]; if (c0 + 1 < nx) fhi += m.free[m.id(c0 + 1, a, b)]; }
          else { if (c0 - 1 >= 0) flo += m.free[m.id(a, b, c0 - 1)]; if (c0 + 1 < nz) fhi += m.free[m.id(a, b, c0 + 1)]; }
        }
    int back = (fhi > flo) ? -1 : 1;
    // nonzero(sub | filled): a 外 b 内
    for (int ia = 0; ia < SA; ++ia)
      for (int ib = 0; ib < SB; ++ib) {
        if (!(sub[(long)ia * SB + ib] || filled[(long)ia * SB + ib])) continue;
        int A = a0 + ia, B = b0 + ib;
        for (int k = 0; k < T; ++k) {
          int c = c0 + k * back;
          int x, y, z;
          if (axis_x) { x = c; y = A; z = B; } else { x = A; y = B; z = c; }
          if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) continue;
          long id = m.id(x, y, z);
          if (m.occ[id] || m.free[id]) continue;
          VoxelDelta d;
          d.idx[0] = x; d.idx[1] = y; d.idx[2] = z;
          d.op = DeltaOp::ADD; d.sem = WALL;
          d.generator = "wall_fill"; d.binding = default_binding;
          out.push_back(d);
        }
      }
  };
  for (int x0 : projection_peaks(xcount, min_wall_cells_)) fill_plane(true, x0);
  for (int z0 : projection_peaks(zcount, min_wall_cells_)) fill_plane(false, z0);
  return out;
}

// ===================== RoofCap =====================
RoofCap::RoofCap(double level_gap_m, int band_cells)
    : level_gap_m_(level_gap_m), band_cells_(band_cells) {
  id = "roof_cap"; stage = 2; depends_on = {"slab_fill_ceiling"};
  default_binding = "persistent";
}
std::vector<VoxelDelta> RoofCap::run(const MapView& m, const Overlay&) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  const int CEIL = 4;
  std::vector<uint8_t> ceil_((size_t)nx * ny * nz, 0);
  std::vector<int> ys;
  bool any = false;
  for (long i = 0; i < (long)nx * ny * nz; ++i)
    if (m.occ[i] && m.sem[i] == CEIL) { ceil_[i] = 1; any = true; ys.push_back((int)((i / nz) % ny)); }
  std::vector<VoxelDelta> out;
  if (!any) return out;
  int gap = std::max(1, (int)std::lround(level_gap_m_ / m.voxel_size));
  std::set<long> emitted;
  for (int Y : cluster_levels(ys, gap, ny)) {
    int ylo = std::max(0, Y - band_cells_), yhi = std::min(ny, Y + band_cells_ + 1);
    std::vector<uint8_t> foot((size_t)nx * nz, 0), filled;
    for (int x = 0; x < nx; ++x)
      for (int z = 0; z < nz; ++z) {
        bool f = false;
        for (int y = ylo; y < yhi && !f; ++y) if (ceil_[m.id(x, y, z)]) f = true;
        foot[(long)x * nz + z] = f ? 1 : 0;
      }
    fill_holes2d(foot, nx, nz, filled);
    for (int x = 0; x < nx; ++x)
      for (int z = 0; z < nz; ++z) {
        long f2 = (long)x * nz + z;
        if (!(filled[f2] && !foot[f2])) continue;   // holes = filled & ~foot
        long id = m.id(x, Y, z);
        if (emitted.count(id)) continue;
        if (m.free[id] || m.occ[id]) continue;
        emitted.insert(id);
        VoxelDelta d;
        d.idx[0] = x; d.idx[1] = Y; d.idx[2] = z;
        d.op = DeltaOp::ADD; d.sem = CEIL;
        d.generator = "roof_cap"; d.binding = default_binding;
        out.push_back(d);
      }
  }
  return out;
}

}  // namespace smc
