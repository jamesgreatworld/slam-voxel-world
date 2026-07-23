// coarsen.cpp — 见 coarsen.hpp。逐行移植 vlayer/coarsen.py。
#include "semantic_map_core/coarsen.hpp"

#include "semantic_map_core/field.hpp"   // compute_esdf(最近站点 parent)

#include <algorithm>
#include <queue>

namespace smc {

static inline long ID(int x, int y, int z, int ny, int nz) {
  return ((long)x * ny + y) * nz + z;
}

void downsample_occupancy(const uint8_t* occ, const uint8_t* sem, int nx, int ny, int nz,
                          const long vmin[3], float vs, int factor, int min_fine,
                          std::vector<uint8_t>& occ_c, std::vector<uint8_t>& sem_c,
                          long vmin_c[3], float& vs_c, int dims_c[3]) {
  if (factor <= 1) {
    occ_c.assign(occ, occ + (long)nx * ny * nz);
    sem_c.assign(sem, sem + (long)nx * ny * nz);
    for (int a = 0; a < 3; ++a) vmin_c[a] = vmin[a];
    dims_c[0] = nx; dims_c[1] = ny; dims_c[2] = nz;
    vs_c = vs;
    return;
  }
  const int f = factor;
  const int dims[3] = {nx, ny, nz};
  int lo[3], padded[3];
  for (int a = 0; a < 3; ++a) {
    lo[a] = (int)(((vmin[a] % f) + f) % f);          // Python mod
    int sz = dims[a] + lo[a];
    int hi = (int)(((-(long)sz) % f + f) % f);
    padded[a] = sz + hi;
    vmin_c[a] = (vmin[a] - lo[a]) / f;               // 整除(vmin-lo 必为 f 倍数)
    dims_c[a] = padded[a] / f;
  }
  vs_c = vs * f;
  const int NX = dims_c[0], NY = dims_c[1], NZ = dims_c[2];
  occ_c.assign((size_t)NX * NY * NZ, 0);
  sem_c.assign((size_t)NX * NY * NZ, 0);
  std::vector<int> cnt((size_t)NX * NY * NZ, 0);
  // 第一遍: 占据计数
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z)
        if (occ[ID(x, y, z, ny, nz)])
          cnt[ID((x + lo[0]) / f, (y + lo[1]) / f, (z + lo[2]) / f, NY, NZ)]++;
  for (long i = 0; i < (long)NX * NY * NZ; ++i) occ_c[i] = cnt[i] >= min_fine ? 1 : 0;
  // 第二遍: 语义众数(L 从 1 升序, 严格 > -> 平局取小 label)
  // 按块收集: 块数可能大, 用每块 256 计数太大 -> 逐块扫描(块内细格定位)
  for (int bx = 0; bx < NX; ++bx)
    for (int by = 0; by < NY; ++by)
      for (int bz = 0; bz < NZ; ++bz) {
        long bi = ID(bx, by, bz, NY, NZ);
        if (!cnt[bi]) continue;
        int counts[256] = {0};
        for (int dx = 0; dx < f; ++dx)
          for (int dy = 0; dy < f; ++dy)
            for (int dz = 0; dz < f; ++dz) {
              int x = bx * f + dx - lo[0], y = by * f + dy - lo[1], z = bz * f + dz - lo[2];
              if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) continue;
              long id = ID(x, y, z, ny, nz);
              if (occ[id]) counts[sem[id]]++;
            }
        int best_cnt = 0, best_lab = 0;
        for (int L = 1; L <= 255; ++L)
          if (counts[L] > best_cnt) { best_cnt = counts[L]; best_lab = L; }
        sem_c[bi] = (uint8_t)best_lab;
      }
}

void prune_dangles(std::vector<uint8_t>& occ, std::vector<uint8_t>& sem,
                   int nx, int ny, int nz, int iterations) {
  for (int it = 0; it < iterations; ++it) {
    std::vector<long> dangle;
    for (int x = 0; x < nx; ++x)
      for (int y = 0; y < ny; ++y)
        for (int z = 0; z < nz; ++z) {
          long id = ID(x, y, z, ny, nz);
          if (!occ[id]) continue;
          int nb = 0;
          if (x > 0) nb += occ[id - (long)ny * nz];
          if (x + 1 < nx) nb += occ[id + (long)ny * nz];
          if (y > 0) nb += occ[id - nz];
          if (y + 1 < ny) nb += occ[id + nz];
          if (z > 0) nb += occ[id - 1];
          if (z + 1 < nz) nb += occ[id + 1];
          if (nb <= 1) dangle.push_back(id);
        }
    if (dangle.empty()) break;
    for (long id : dangle) { occ[id] = 0; sem[id] = 0; }
  }
}

// 6-连通 3D 膨胀/腐蚀(border=0, 与 scipy binary_closing 默认一致)
static void dilate3d6(const std::vector<uint8_t>& a, int nx, int ny, int nz,
                      std::vector<uint8_t>& out) {
  out.assign(a.size(), 0);
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z) {
        long id = ID(x, y, z, ny, nz);
        bool v = a[id] || (x > 0 && a[id - (long)ny * nz]) || (x + 1 < nx && a[id + (long)ny * nz]) ||
                 (y > 0 && a[id - nz]) || (y + 1 < ny && a[id + nz]) ||
                 (z > 0 && a[id - 1]) || (z + 1 < nz && a[id + 1]);
        out[id] = v ? 1 : 0;
      }
}
static void erode3d6(const std::vector<uint8_t>& a, int nx, int ny, int nz,
                     std::vector<uint8_t>& out) {
  out.assign(a.size(), 0);
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z) {
        long id = ID(x, y, z, ny, nz);
        bool v = a[id] && (x > 0 ? (bool)a[id - (long)ny * nz] : false) &&
                 (x + 1 < nx ? (bool)a[id + (long)ny * nz] : false) &&
                 (y > 0 ? (bool)a[id - nz] : false) && (y + 1 < ny ? (bool)a[id + nz] : false) &&
                 (z > 0 ? (bool)a[id - 1] : false) && (z + 1 < nz ? (bool)a[id + 1] : false);
        out[id] = v ? 1 : 0;
      }
}

void clean_coarse(std::vector<uint8_t>& occ, std::vector<uint8_t>& sem,
                  int nx, int ny, int nz, int min_component, int close_radius,
                  bool keep_largest) {
  const long N = (long)nx * ny * nz;
  if (close_radius > 0) {
    std::vector<uint8_t> closed = occ, tmp;
    for (int i = 0; i < close_radius; ++i) { dilate3d6(closed, nx, ny, nz, tmp); closed.swap(tmp); }
    for (int i = 0; i < close_radius; ++i) { erode3d6(closed, nx, ny, nz, tmp); closed.swap(tmp); }
    bool any_new = false;
    for (long i = 0; i < N; ++i) if (closed[i] && !occ[i]) { any_new = true; break; }
    if (any_new) {
      // 新格 sem = 最近原占据格的 sem(compute_esdf 的站点传播)
      std::vector<float> dist; std::vector<long> parent;
      compute_esdf(occ.data(), nx, ny, nz, 1.0f, dist, parent);
      for (long i = 0; i < N; ++i)
        if (closed[i] && !occ[i] && parent[i] >= 0) sem[i] = sem[parent[i]];
      occ = closed;
    }
  }
  auto drop_components = [&](bool only_keep_largest) {
    std::vector<int> lbl(N, 0);
    int ncomp = 0;
    std::vector<long> q;
    for (long s = 0; s < N; ++s) {
      if (!occ[s] || lbl[s]) continue;
      ++ncomp; lbl[s] = ncomp; q.clear(); q.push_back(s);
      for (size_t h = 0; h < q.size(); ++h) {
        long id = q[h];
        int x = (int)(id / ((long)ny * nz)); long r = id % ((long)ny * nz);
        int y = (int)(r / nz), z = (int)(r % nz);
        const long nbs[6] = {x > 0 ? id - (long)ny * nz : -1, x + 1 < nx ? id + (long)ny * nz : -1,
                             y > 0 ? id - nz : -1, y + 1 < ny ? id + nz : -1,
                             z > 0 ? id - 1 : -1, z + 1 < nz ? id + 1 : -1};
        for (long nb : nbs)
          if (nb >= 0 && occ[nb] && !lbl[nb]) { lbl[nb] = ncomp; q.push_back(nb); }
      }
    }
    if (!ncomp) return;
    std::vector<long> sizes(ncomp + 1, 0);
    for (long i = 0; i < N; ++i) if (lbl[i]) sizes[lbl[i]]++;
    std::vector<uint8_t> drop(ncomp + 1, 0);
    if (only_keep_largest) {
      if (ncomp <= 1) return;
      int keep = 1;
      for (int c = 2; c <= ncomp; ++c) if (sizes[c] > sizes[keep]) keep = c;  // argmax 平局取小 id
      for (int c = 1; c <= ncomp; ++c) drop[c] = (c != keep);
    } else {
      for (int c = 1; c <= ncomp; ++c) drop[c] = sizes[c] < min_component;
    }
    for (long i = 0; i < N; ++i)
      if (lbl[i] && drop[lbl[i]]) { occ[i] = 0; sem[i] = 0; }
  };
  if (min_component > 1) drop_components(false);
  if (keep_largest) drop_components(true);
}

}  // namespace smc
