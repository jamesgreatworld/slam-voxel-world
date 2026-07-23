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

// 2D 8-连通连通分量标号(光栅首遇序, 同 scipy.ndimage.label(struct=2,2))。返回分量数。
static int label8_2d(const std::vector<uint8_t>& mask, int NV, int NH, std::vector<int>& lbl) {
  lbl.assign((size_t)NV * NH, 0);
  int next = 0;
  std::vector<long> q;
  for (int v = 0; v < NV; ++v)
    for (int h = 0; h < NH; ++h) {
      long id = (long)v * NH + h;
      if (!mask[id] || lbl[id]) continue;
      ++next; lbl[id] = next; q.clear(); q.push_back(id);
      for (size_t t = 0; t < q.size(); ++t) {
        int cv = q[t] / NH, ch = q[t] % NH;
        for (int dv = -1; dv <= 1; ++dv)
          for (int dh = -1; dh <= 1; ++dh) {
            if (!dv && !dh) continue;
            int nv = cv + dv, nh = ch + dh;
            if (nv < 0 || nv >= NV || nh < 0 || nh >= NH) continue;
            long nid = (long)nv * NH + nh;
            if (mask[nid] && !lbl[nid]) { lbl[nid] = next; q.push_back(nid); }
          }
      }
    }
  return next;
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

// 2D 精确平方距离场(到 mask=1 的最近格), Felzenszwalb 两趟。返回 d²(int 值域)。
static void edt2d_sq(const std::vector<uint8_t>& mask, int NV, int NH, std::vector<double>& d2) {
  const double BIG = 1e18;
  d2.assign((size_t)NV * NH, BIG);
  // 趟1: 沿 v(列内) 1D 距离
  for (int h = 0; h < NH; ++h) {
    double d = BIG;
    for (int v = 0; v < NV; ++v) {
      if (mask[(long)v * NH + h]) d = 0; else if (d < BIG) d += 1;
      d2[(long)v * NH + h] = d;
    }
    d = BIG;
    for (int v = NV - 1; v >= 0; --v) {
      if (mask[(long)v * NH + h]) d = 0; else if (d < BIG) d += 1;
      double& c = d2[(long)v * NH + h];
      if (d < c) c = d;
    }
  }
  for (long i = 0; i < (long)NV * NH; ++i) if (d2[i] < BIG) d2[i] *= d2[i];
  // 趟2: 沿 h 抛物线包络
  std::vector<double> f(NH), out(NH), zz(NH + 1);
  std::vector<int> vv(NH);
  for (int v = 0; v < NV; ++v) {
    for (int h = 0; h < NH; ++h) f[h] = d2[(long)v * NH + h];
    int k = 0; vv[0] = 0; zz[0] = -BIG; zz[1] = BIG;
    for (int q = 1; q < NH; ++q) {
      double s = ((f[q] + (double)q * q) - (f[vv[k]] + (double)vv[k] * vv[k])) / (2.0 * q - 2.0 * vv[k]);
      while (s <= zz[k]) {
        --k;
        s = ((f[q] + (double)q * q) - (f[vv[k]] + (double)vv[k] * vv[k])) / (2.0 * q - 2.0 * vv[k]);
      }
      ++k; vv[k] = q; zz[k] = s; zz[k + 1] = BIG;
    }
    k = 0;
    for (int q = 0; q < NH; ++q) {
      while (zz[k + 1] < (double)q) ++k;
      out[q] = (double)(q - vv[k]) * (q - vv[k]) + f[vv[k]];
    }
    for (int h = 0; h < NH; ++h) d2[(long)v * NH + h] = out[h];
  }
}

// ===================== PlaneRegularize =====================
static void robust_rect(const std::vector<uint8_t>& comp, int NH, double trim,
                        int& v0, int& v1, int& h0, int& h1);  // 定义在 OpeningCarve 节

PlaneRegularize::PlaneRegularize(double k_per_cell, double prior_cap, double trim_fill,
                                 int min_wall_cells, int min_height, int min_run,
                                 double support_m, double keep_comp_m2)
    : k_(k_per_cell), cap_(prior_cap), trim_fill_(trim_fill), support_m_(support_m),
      keep_comp_m2_(keep_comp_m2), min_wall_cells_(min_wall_cells), min_height_(min_height),
      min_run_(min_run) {
  id = "plane_regularize"; stage = 2; depends_on = {"wall_fill"};
  default_binding = "persistent";
}

std::vector<VoxelDelta> PlaneRegularize::run(const MapView& m, const Overlay&) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  const int WALL = 19;
  std::vector<VoxelDelta> out;
  if (!m.logodds) return out;
  std::vector<uint8_t> wall((size_t)nx * ny * nz, 0);
  bool any = false;
  for (long i = 0; i < (long)nx * ny * nz; ++i)
    if (m.occ[i] && m.sem[i] == WALL) { wall[i] = 1; any = true; }
  if (!any) return out;
  const double vs = m.voxel_size;
  int support_cells = std::max(1, (int)std::lround(support_m_ / vs));
  int keep_comp_cells = std::max(4, (int)std::lround(keep_comp_m2_ / (vs * vs)));
  std::set<long> emitted;

  std::vector<long> xcount(nx, 0), zcount(nz, 0);
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z)
        if (wall[m.id(x, y, z)]) { xcount[x]++; zcount[z]++; }

  auto regularize = [&](bool axis_x, int c0) {
    const int NV = ny, NH = axis_x ? nz : nx;
    auto W2 = [&](int v, int h) { return axis_x ? wall[m.id(c0, v, h)] : wall[m.id(h, v, c0)]; };
    auto F2 = [&](int v, int h) { return axis_x ? m.free[m.id(c0, v, h)] : m.free[m.id(h, v, c0)]; };
    auto L2 = [&](int v, int h) {
      return axis_x ? m.logodds[m.id(c0, v, h)] : m.logodds[m.id(h, v, c0)];
    };
    long wsum = 0; int v0 = NV, v1 = -1, h0 = NH, h1 = -1;
    std::vector<uint8_t> wall2d((size_t)NV * NH, 0);
    for (int v = 0; v < NV; ++v)
      for (int h = 0; h < NH; ++h)
        if (W2(v, h)) {
          wall2d[(long)v * NH + h] = 1; wsum++;
          v0 = std::min(v0, v); v1 = std::max(v1, v);
          h0 = std::min(h0, h); h1 = std::max(h1, h);
        }
    if (wsum < min_wall_cells_) return;
    if ((v1 - v0 + 1) < min_height_ || (h1 - h0 + 1) < min_run_) return;
    int rv0 = v0, rv1 = v1, rh0 = h0, rh1 = h1;
    robust_rect(wall2d, NH, trim_fill_, rv0, rv1, rh0, rh1);
    // near_wall = EDT(~wall2d) <= support_cells(平方比较, 与 float sqrt 等价)
    std::vector<double> d2;
    edt2d_sq(wall2d, NV, NH, d2);
    const double s2 = (double)support_cells * support_cells;
    // cut 大分量保护: wall2d & ~inside 的 8 连通分量 >= keep_comp_cells 不删
    std::vector<uint8_t> outside_wall((size_t)NV * NH, 0);
    for (int v = 0; v < NV; ++v)
      for (int h = 0; h < NH; ++h) {
        bool inside = (v - rv0 > -1) && (rv1 - v > -1) && (h - rh0 > -1) && (rh1 - h > -1);
        if (!inside && wall2d[(long)v * NH + h]) outside_wall[(long)v * NH + h] = 1;
      }
    std::vector<int> comp;
    int ncomp = label8_2d(outside_wall, NV, NH, comp);
    std::vector<long> csz(ncomp + 1, 0);
    for (long i = 0; i < (long)NV * NH; ++i) if (comp[i]) csz[comp[i]]++;
    // fill 与 cut(fill 先, argwhere 行主序)
    auto prior_at = [&](int v, int h) {
      long din = std::min(std::min((long)(v - rv0), (long)(rv1 - v)),
                          std::min((long)(h - rh0), (long)(rh1 - h))) + 1;
      if (din > 0) return std::min((double)din * k_, cap_);
      long dout = std::max(std::max((long)(rv0 - v), (long)(v - rv1)),
                           std::max((long)(rh0 - h), (long)(h - rh1)));
      return -std::min((double)dout * k_, cap_);
    };
    auto emit = [&](int v, int h, bool is_fill) {
      int x, y, z;
      if (axis_x) { x = c0; y = v; z = h; } else { x = h; y = v; z = c0; }
      long id3 = m.id(x, y, z);
      if (emitted.count(id3)) return;
      emitted.insert(id3);
      VoxelDelta d;
      d.idx[0] = x; d.idx[1] = y; d.idx[2] = z;
      d.op = is_fill ? DeltaOp::ADD : DeltaOp::REMOVE;
      d.sem = is_fill ? WALL : 0;
      d.generator = "plane_regularize"; d.binding = default_binding;
      out.push_back(d);
    };
    for (int v = 0; v < NV; ++v)
      for (int h = 0; h < NH; ++h) {
        long i2 = (long)v * NH + h;
        bool inside = (v >= rv0 && v <= rv1 && h >= rh0 && h <= rh1);
        if (!(inside && !wall2d[i2] && !F2(v, h))) continue;
        double post = (double)L2(v, h) + prior_at(v, h);
        if (post >= 0.85 && d2[i2] <= s2) emit(v, h, true);
      }
    for (int v = 0; v < NV; ++v)
      for (int h = 0; h < NH; ++h) {
        long i2 = (long)v * NH + h;
        bool inside = (v >= rv0 && v <= rv1 && h >= rh0 && h <= rh1);
        if (!(!inside && wall2d[i2])) continue;
        double post = (double)L2(v, h) + prior_at(v, h);
        if (post >= 0.85) continue;
        if (comp[i2] && csz[comp[i2]] >= keep_comp_cells) continue;  // 大分量=真墙翼
        emit(v, h, false);
      }
  };
  for (int c0 : projection_peaks(xcount, min_wall_cells_)) regularize(true, c0);
  for (int c0 : projection_peaks(zcount, min_wall_cells_)) regularize(false, c0);
  return out;
}

// ===================== RansacPlaneFill =====================
RansacPlaneFill::RansacPlaneFill(std::vector<int> labels, double dist_thresh, int min_inliers,
                                 int max_planes, int iters, uint64_t seed)
    : labels_(std::move(labels)), dist_thresh_(dist_thresh), min_inliers_(min_inliers),
      max_planes_(max_planes), iters_(iters), seed_(seed) {
  id = "ransac_plane"; stage = 1; default_binding = "persistent";
}

std::vector<VoxelDelta> RansacPlaneFill::run(const MapView& m, const Overlay&) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  std::vector<VoxelDelta> out;
  std::set<int> labset(labels_.begin(), labels_.end());
  std::vector<std::array<double, 3>> pts;
  for (int x = 0; x < nx; ++x)          // argwhere 行主序
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z) {
        long id = m.id(x, y, z);
        if (m.occ[id] && labset.count(m.sem[id]))
          pts.push_back({(double)x, (double)y, (double)z});
      }
  if ((int)pts.size() < min_inliers_) return out;
  int fill_label = labels_[0];
  std::vector<int> remaining(pts.size());
  for (size_t i = 0; i < remaining.size(); ++i) remaining[i] = (int)i;
  uint64_t rng = seed_ * 0x9E3779B97F4A7C15ull + 0xBF58476D1CE4E5B9ull;  // splitmix64 态
  auto rnd = [&]() {
    rng += 0x9E3779B97F4A7C15ull;
    uint64_t z = rng;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  };
  size_t trial_i = 0;
  for (int p = 0; p < max_planes_; ++p) {
    if ((int)remaining.size() < min_inliers_) break;
    double bn[3] = {0, 0, 0}, bd = 0;
    int bcnt = 0;
    bool have = false;
    for (int it = 0; it < iters_; ++it) {
      int i0, i1, i2;
      if (trials) {
        if (trial_i >= trials->size()) break;
        auto& t = (*trials)[trial_i++];
        i0 = t[0]; i1 = t[1]; i2 = t[2];
      } else {   // 无放回抽 3(内置 rng, 生产路径)
        int n = (int)remaining.size();
        i0 = (int)(rnd() % n);
        do { i1 = (int)(rnd() % n); } while (i1 == i0);
        do { i2 = (int)(rnd() % n); } while (i2 == i0 || i2 == i1);
      }
      const auto& A = pts[remaining[i0]];
      const auto& B = pts[remaining[i1]];
      const auto& C = pts[remaining[i2]];
      double u[3] = {B[0] - A[0], B[1] - A[1], B[2] - A[2]};
      double v[3] = {C[0] - A[0], C[1] - A[1], C[2] - A[2]};
      double n3[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                      u[0] * v[1] - u[1] * v[0]};
      double nn = std::sqrt(n3[0] * n3[0] + n3[1] * n3[1] + n3[2] * n3[2]);
      if (nn < 1e-6) continue;
      for (double& c : n3) c /= nn;
      double d = n3[0] * A[0] + n3[1] * A[1] + n3[2] * A[2];
      int cnt = 0;
      for (int ri : remaining) {
        double dist = std::fabs(pts[ri][0] * n3[0] + pts[ri][1] * n3[1] + pts[ri][2] * n3[2] - d);
        if (dist < dist_thresh_) ++cnt;
      }
      if (cnt > bcnt) { bcnt = cnt; bd = d; bn[0] = n3[0]; bn[1] = n3[1]; bn[2] = n3[2]; have = true; }
    }
    if (!have || bcnt < min_inliers_) break;
    // inliers + 栅格化
    std::vector<int> inl, rest;
    for (int ri : remaining) {
      double dist = std::fabs(pts[ri][0] * bn[0] + pts[ri][1] * bn[1] + pts[ri][2] * bn[2] - bd);
      (dist < dist_thresh_ ? inl : rest).push_back(ri);
    }
    int ax = 0;
    { double aa = std::fabs(bn[0]);   // argmax(|n|) 首个最大
      if (std::fabs(bn[1]) > aa) { aa = std::fabs(bn[1]); ax = 1; }
      if (std::fabs(bn[2]) > aa) ax = 2; }
    int o0 = -1, o1 = -1;
    for (int a = 0; a < 3; ++a) { if (a == ax) continue; if (o0 < 0) o0 = a; else o1 = a; }
    long lo[3] = {1 << 30, 1 << 30, 1 << 30}, hi[3] = {-(1 << 30), -(1 << 30), -(1 << 30)};
    for (int ri : inl)
      for (int a = 0; a < 3; ++a) {
        long c = (long)pts[ri][a];
        lo[a] = std::min(lo[a], c); hi[a] = std::max(hi[a], c);
      }
    for (long u2 = lo[o0]; u2 <= hi[o0]; ++u2)
      for (long v2 = lo[o1]; v2 <= hi[o1]; ++v2) {
        double w = (bd - bn[o0] * (double)u2 - bn[o1] * (double)v2) / bn[ax];
        long c[3]; c[o0] = u2; c[o1] = v2;
        c[ax] = (long)std::nearbyint(w);   // Python round = 半偶
        int x = (int)c[0], y = (int)c[1], z = (int)c[2];
        if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) continue;
        long id = m.id(x, y, z);
        if (m.occ[id] || m.free[id]) continue;
        VoxelDelta dd;
        dd.idx[0] = x; dd.idx[1] = y; dd.idx[2] = z;
        dd.op = DeltaOp::ADD; dd.sem = fill_label;
        dd.generator = "ransac_plane"; dd.binding = default_binding;
        out.push_back(dd);
      }
    remaining.swap(rest);
  }
  return out;
}

// ===================== StairsFill =====================
StairsFill::StairsFill(double max_depth_m) : max_depth_m_(max_depth_m) {
  id = "stairs_fill"; stage = 1; default_binding = "persistent";
}
std::vector<VoxelDelta> StairsFill::run(const MapView& m, const Overlay&) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  const int STAIRS = 15, FLOOR = 3;
  std::vector<VoxelDelta> out;
  bool any = false, any_floor = false;
  int ground_y = 0;
  for (long i = 0; i < (long)nx * ny * nz; ++i) {
    if (!m.occ[i]) continue;
    if (m.sem[i] == STAIRS) any = true;
    else if (m.sem[i] == FLOOR) {
      int y = (int)((i / nz) % ny);
      if (!any_floor || y < ground_y) ground_y = y;
      any_floor = true;
    }
  }
  if (!any) return out;
  if (!any_floor) ground_y = 0;
  int depth = std::max(1, (int)std::lround(max_depth_m_ / m.voxel_size));
  for (int x = 0; x < nx; ++x)      // argwhere(stairs.any(axis=1)) = (x,z) 升序
    for (int z = 0; z < nz; ++z) {
      int top = -1;
      for (int y = ny - 1; y >= 0; --y) {
        long id = m.id(x, y, z);
        if (m.occ[id] && m.sem[id] == STAIRS) { top = y; break; }
      }
      if (top < 0) continue;
      int lo = std::max(ground_y, top - depth);
      for (int y = top - 1; y >= lo; --y) {
        long id = m.id(x, y, z);
        if (m.occ[id] || m.free[id]) break;
        VoxelDelta d;
        d.idx[0] = x; d.idx[1] = y; d.idx[2] = z;
        d.op = DeltaOp::ADD; d.sem = STAIRS;
        d.generator = "stairs_fill"; d.binding = default_binding;
        out.push_back(d);
      }
    }
  return out;
}

// ===================== OpeningCarve =====================
OpeningCarve::OpeningCarve(double min_area_m2, double max_area_m2, double max_extent_m,
                           double enclosure_min, double thickness_m, int min_wall_cells,
                           int min_height, int min_run, double trim_fill)
    : min_area_m2_(min_area_m2), max_area_m2_(max_area_m2), max_extent_m_(max_extent_m),
      enclosure_min_(enclosure_min), thickness_m_(thickness_m), trim_fill_(trim_fill),
      min_wall_cells_(min_wall_cells), min_height_(min_height), min_run_(min_run) {
  id = "opening_carve"; stage = 2; depends_on = {"wall_fill", "plane_regularize"};
  default_binding = "persistent";
}

// _robust_rect: 迭代剔除填充率 < trim_fill 的边缘行/列。comp 为 NVxNH bool, 传入其 bbox。
static void robust_rect(const std::vector<uint8_t>& comp, int NH, double trim,
                        int& v0, int& v1, int& h0, int& h1) {
  auto C = [&](int v, int h) { return comp[(long)v * NH + h]; };
  bool changed = true;
  while (changed && v1 > v0 && h1 > h0) {
    changed = false;
    int w = h1 - h0 + 1, hgt = v1 - v0 + 1;
    long s;
    s = 0; for (int h = h0; h <= h1; ++h) s += C(v0, h);
    if (s < trim * w) { v0++; changed = true; continue; }
    s = 0; for (int h = h0; h <= h1; ++h) s += C(v1, h);
    if (s < trim * w) { v1--; changed = true; continue; }
    s = 0; for (int v = v0; v <= v1; ++v) s += C(v, h0);
    if (s < trim * hgt) { h0++; changed = true; continue; }
    s = 0; for (int v = v0; v <= v1; ++v) s += C(v, h1);
    if (s < trim * hgt) { h1--; changed = true; }
  }
}

// _fit_template: 矩形 vs 拱形(矩形+半圆顶), IoU 高者胜。local 为 HxW(裁剪后)。
static std::vector<uint8_t> fit_template(const std::vector<uint8_t>& local, int H, int W) {
  long ones = 0; for (uint8_t v : local) ones += v;
  double iou_rect = (double)ones / (double)(H * W);
  double r = W / 2.0;
  std::vector<uint8_t> rect((size_t)H * W, 1);
  if (H <= r) return rect;
  double cv = H - r, ch = (W - 1) / 2.0;
  std::vector<uint8_t> arch((size_t)H * W, 0);
  long inter = 0, uni = 0;
  for (int v = 0; v < H; ++v)
    for (int h = 0; h < W; ++h) {
      bool a = (v < cv) || ((h - ch) * (h - ch) + (v - cv) * (v - cv) <= r * r);
      arch[(long)v * W + h] = a ? 1 : 0;
      bool l = local[(long)v * W + h];
      if (l && a) ++inter;
      if (l || a) ++uni;
    }
  double iou_arch = uni ? (double)inter / (double)uni : 0.0;
  return iou_arch > iou_rect ? arch : rect;
}

std::vector<VoxelDelta> OpeningCarve::run(const MapView& m, const Overlay& acc) const {
  const int nx = m.nx, ny = m.ny, nz = m.nz;
  const double vs = m.voxel_size;
  const int WALL = 19;
  std::vector<uint8_t> wall((size_t)nx * ny * nz, 0);
  bool any = false;
  for (long i = 0; i < (long)nx * ny * nz; ++i)
    if (m.occ[i] && m.sem[i] == WALL) { wall[i] = 1; any = true; }
  std::vector<VoxelDelta> out;
  if (!any) return out;
  int min_cells = std::max(4, (int)std::lround(min_area_m2_ / (vs * vs)));
  int max_cells = (int)std::lround(max_area_m2_ / (vs * vs));
  int max_ext = (int)std::lround(max_extent_m_ / vs);
  int T = std::max(1, (int)std::ceil(thickness_m_ / vs - 1e-9));
  std::set<long> added;                        // 前序补墙(wall 质 add)
  for (const VoxelDelta& d : acc.voxels)
    if (d.op == DeltaOp::ADD && d.sem == WALL) added.insert(m.id(d.idx[0], d.idx[1], d.idx[2]));

  std::vector<long> xcount(nx, 0), zcount(nz, 0);
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z)
        if (wall[m.id(x, y, z)]) { xcount[x]++; zcount[z]++; }
  std::set<long> emitted;

  auto carve = [&](bool axis_x, int c0) {
    const int NV = ny, NH = axis_x ? nz : nx;
    auto W3 = [&](int v, int h) { return axis_x ? wall[m.id(c0, v, h)] : wall[m.id(h, v, c0)]; };
    auto F = [&](int v, int h) { return axis_x ? m.free[m.id(c0, v, h)] : m.free[m.id(h, v, c0)]; };
    auto Occ = [&](int v, int h) { return axis_x ? m.occ[m.id(c0, v, h)] : m.occ[m.id(h, v, c0)]; };
    auto ID3 = [&](int x, int y, int z) { return m.id(x, y, z); };
    auto to_xyz = [&](int V, int H, int dc, int& x, int& y, int& z) {
      if (axis_x) { x = c0 + dc; y = V; z = H; } else { x = H; y = V; z = c0 + dc; }
    };
    // wall2d sum + bbox
    long wsum = 0; int v0 = NV, v1 = -1, h0 = NH, h1 = -1;
    for (int v = 0; v < NV; ++v)
      for (int h = 0; h < NH; ++h)
        if (W3(v, h)) { wsum++; v0 = std::min(v0, v); v1 = std::max(v1, v); h0 = std::min(h0, h); h1 = std::max(h1, h); }
    if (wsum < min_wall_cells_) return;
    if ((v1 - v0 + 1) < min_height_ || (h1 - h0 + 1) < min_run_) return;
    // solid2d = occ | added; inside = bbox
    std::vector<uint8_t> solid2d((size_t)NV * NH, 0), region((size_t)NV * NH, 0);
    for (int v = 0; v < NV; ++v)
      for (int h = 0; h < NH; ++h) {
        int x, y, z; to_xyz(v, h, 0, x, y, z);
        bool sol = Occ(v, h) || added.count(ID3(x, y, z));
        solid2d[(long)v * NH + h] = sol ? 1 : 0;
        bool ins = (v >= v0 && v <= v1 && h >= h0 && h <= h1);
        region[(long)v * NH + h] = (ins && F(v, h)) ? 1 : 0;   // free & inside
      }
    std::vector<int> lbl;
    int n = label8_2d(region, NV, NH, lbl);
    for (int cid = 1; cid <= n; ++cid) {
      std::vector<uint8_t> comp((size_t)NV * NH, 0);
      long csize = 0;
      for (long i = 0; i < (long)NV * NH; ++i) if (lbl[i] == cid) { comp[i] = 1; ++csize; }
      if (csize < min_cells || csize > max_cells) continue;
      // bbox of comp
      int cv0 = NV, cv1 = -1, ch0 = NH, ch1 = -1;
      for (int v = 0; v < NV; ++v)
        for (int h = 0; h < NH; ++h)
          if (comp[(long)v * NH + h]) { cv0 = std::min(cv0, v); cv1 = std::max(cv1, v); ch0 = std::min(ch0, h); ch1 = std::max(ch1, h); }
      int rv0 = cv0, rv1 = cv1, rh0 = ch0, rh1 = ch1;
      robust_rect(comp, NH, trim_fill_, rv0, rv1, rh0, rh1);
      if ((rv1 - rv0 + 1) > max_ext || (rh1 - rh0 + 1) > max_ext) continue;
      // border = dilate8(comp) & ~comp; enclosure
      long nb = 0, nsolid = 0;
      for (int v = 0; v < NV; ++v)
        for (int h = 0; h < NH; ++h) {
          if (comp[(long)v * NH + h]) continue;
          bool border = false;
          for (int dv = -1; dv <= 1 && !border; ++dv)
            for (int dh = -1; dh <= 1 && !border; ++dh) {
              int nvv = v + dv, nhh = h + dh;
              if (nvv < 0 || nvv >= NV || nhh < 0 || nhh >= NH) continue;
              if (comp[(long)nvv * NH + nhh]) border = true;
            }
          if (border) { ++nb; if (solid2d[(long)v * NH + h]) ++nsolid; }
        }
      if (nb == 0 || (double)nsolid / (double)nb < enclosure_min_) continue;
      // local + template
      int LH = rv1 - rv0 + 1, LW = rh1 - rh0 + 1;
      std::vector<uint8_t> local((size_t)LH * LW, 0);
      for (int v = 0; v < LH; ++v)
        for (int h = 0; h < LW; ++h) local[(long)v * LW + h] = comp[(long)(rv0 + v) * NH + (rh0 + h)];
      std::vector<uint8_t> tmpl = fit_template(local, LH, LW);
      for (int lv = 0; lv < LH; ++lv)
        for (int lh = 0; lh < LW; ++lh) {
          if (!tmpl[(long)lv * LW + lh]) continue;
          int V = rv0 + lv, H = rh0 + lh;
          for (int dc = -T; dc <= T; ++dc) {
            int x, y, z; to_xyz(V, H, dc, x, y, z);
            if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) continue;
            long id3 = ID3(x, y, z);
            if (emitted.count(id3)) continue;
            if (wall[id3] || added.count(id3)) {
              emitted.insert(id3);
              VoxelDelta d;
              d.idx[0] = x; d.idx[1] = y; d.idx[2] = z;
              d.op = DeltaOp::REMOVE; d.sem = 0;
              d.generator = "opening_carve"; d.binding = default_binding;
              out.push_back(d);
            }
          }
        }
    }
  };
  for (int c0 : projection_peaks(xcount, min_wall_cells_)) carve(true, c0);
  for (int c0 : projection_peaks(zcount, min_wall_cells_)) carve(false, c0);
  return out;
}

}  // namespace smc
