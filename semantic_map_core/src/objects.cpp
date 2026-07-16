// objects.cpp — 见 objects.hpp。移植自 m3_adapter/gvd/objects.py + clustering.py。
#include "semantic_map_core/objects.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>

namespace smc {

namespace {
inline long IDX(int i, int j, int k, int ny, int nz) {
  return (static_cast<long>(i) * ny + j) * nz + k;
}
inline long ckey(int i, int j, int k) {
  const long K = 1L << 20;
  return ((long)(i + K) << 42) | ((long)(j + K) << 21) | (long)(k + K);
}
}  // namespace

std::vector<int> dbscan_labels_int3(const std::vector<std::array<int, 3>>& pts,
                                    double eps, int min_samples) {
  const int n = (int)pts.size();
  std::vector<int> labels(n, -1);
  if (n == 0) return labels;
  const double eps2 = eps * eps;
  const int R = (int)std::ceil(eps);
  std::unordered_map<long, int> h;
  h.reserve(n * 2);
  for (int p = 0; p < n; ++p) h[ckey(pts[p][0], pts[p][1], pts[p][2])] = p;

  std::vector<std::vector<int>> nbrs(n);
  std::vector<char> core(n, 0);
  for (int p = 0; p < n; ++p) {
    int i = pts[p][0], j = pts[p][1], k = pts[p][2];
    int cnt = 0;
    for (int di = -R; di <= R; ++di)
      for (int dj = -R; dj <= R; ++dj)
        for (int dk = -R; dk <= R; ++dk) {
          if ((double)(di * di + dj * dj + dk * dk) > eps2) continue;
          auto it = h.find(ckey(i + di, j + dj, k + dk));
          if (it == h.end()) continue;
          ++cnt;  // 含自身
          if (it->second != p) nbrs[p].push_back(it->second);
        }
    core[p] = cnt >= min_samples;
  }
  bool any = false;
  for (int p = 0; p < n; ++p) if (core[p]) { any = true; break; }
  if (!any) return labels;

  std::vector<int> par(n);
  for (int p = 0; p < n; ++p) par[p] = p;
  std::function<int(int)> find = [&](int x) {
    while (par[x] != x) { par[x] = par[par[x]]; x = par[x]; }
    return x;
  };
  for (int p = 0; p < n; ++p)
    if (core[p])
      for (int q : nbrs[p])
        if (core[q]) { int ra = find(p), rb = find(q); if (ra != rb) par[std::max(ra, rb)] = std::min(ra, rb); }

  std::unordered_map<int, int> r2l;
  for (int p = 0; p < n; ++p)
    if (core[p]) {
      int r = find(p);
      auto it = r2l.find(r);
      if (it == r2l.end()) { int lb = (int)r2l.size(); r2l[r] = lb; labels[p] = lb; }
      else labels[p] = it->second;
    }
  // border: 非 core 但 eps 内有 core -> 取最小 core 邻居簇号
  for (int p = 0; p < n; ++p)
    if (!core[p])
      for (int q : nbrs[p])
        if (core[q]) { int lq = labels[q]; if (labels[p] == -1 || lq < labels[p]) labels[p] = lq; }
  return labels;
}

namespace {
int bbox_gap(const int* b1min, const int* b1max, const int* b2min, const int* b2max) {
  int gap = 0;
  for (int a = 0; a < 3; ++a) {
    int sep = std::max(std::max(b2min[a] - b1max[a] - 1, b1min[a] - b2max[a] - 1), 0);
    gap = std::max(gap, sep);
  }
  return gap;
}
}  // namespace

std::vector<ObjectNode> extract_objects(const uint8_t* occ, const uint8_t* sem, int nx,
                                        int ny, int nz,
                                        const std::set<int>& structure_labels,
                                        int min_voxels, double cluster_eps, int min_samples,
                                        int merge_gap) {
  std::vector<ObjectNode> out;
  // 出现的非结构、非 0 类(升序)
  std::set<int> present;
  for (long id = 0; id < (long)nx * ny * nz; ++id)
    if (occ[id] && sem[id] != 0 && !structure_labels.count(sem[id])) present.insert(sem[id]);

  for (int L : present) {
    // cells_all = argwhere(occ & sem==L), C 序
    std::vector<std::array<int, 3>> cells;
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < ny; ++j)
        for (int k = 0; k < nz; ++k) {
          long id = IDX(i, j, k, ny, nz);
          if (occ[id] && sem[id] == L) cells.push_back({i, j, k});
        }
    if ((int)cells.size() < min_voxels) continue;

    auto lab = dbscan_labels_int3(cells, cluster_eps, min_samples);
    int maxlab = -1;
    for (int l : lab) maxlab = std::max(maxlab, l);
    // clusters: g 升序, g>=0, size>=min_voxels
    std::vector<std::vector<int>> clusters;  // 每簇存 cells 索引
    for (int g = 0; g <= maxlab; ++g) {
      std::vector<int> idxs;
      for (int p = 0; p < (int)cells.size(); ++p) if (lab[p] == g) idxs.push_back(p);
      if ((int)idxs.size() >= min_voxels) clusters.push_back(std::move(idxs));
    }
    if (clusters.empty()) continue;

    // union-find bbox 邻近合并
    int nc = (int)clusters.size();
    std::vector<std::array<int, 3>> bmin(nc), bmax(nc);
    for (int c = 0; c < nc; ++c) {
      std::array<int, 3> lo{1 << 30, 1 << 30, 1 << 30}, hi{-(1 << 30), -(1 << 30), -(1 << 30)};
      for (int pi : clusters[c])
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], cells[pi][a]); hi[a] = std::max(hi[a], cells[pi][a]); }
      bmin[c] = lo; bmax[c] = hi;
    }
    std::vector<int> par(nc);
    for (int c = 0; c < nc; ++c) par[c] = c;
    std::function<int(int)> find = [&](int x) { while (par[x] != x) { par[x] = par[par[x]]; x = par[x]; } return x; };
    for (int a = 0; a < nc; ++a)
      for (int b = a + 1; b < nc; ++b)
        if (bbox_gap(bmin[a].data(), bmax[a].data(), bmin[b].data(), bmax[b].data()) <= merge_gap) {
          int ra = find(a), rb = find(b); if (ra != rb) par[rb] = ra;  // union(a,b): parent[find(b)]=find(a)
        }
    // 分组(按 root 首次出现序), 合并 cells
    std::unordered_map<int, std::vector<int>> groups;
    std::vector<int> group_order;
    for (int c = 0; c < nc; ++c) {
      int r = find(c);
      if (!groups.count(r)) group_order.push_back(r);
      for (int pi : clusters[c]) groups[r].push_back(pi);
    }
    for (int r : group_order) {
      auto& merged = groups[r];
      if ((int)merged.size() < min_voxels) continue;
      double cx = 0, cy = 0, cz = 0;
      std::array<int, 3> lo{1 << 30, 1 << 30, 1 << 30}, hi{-(1 << 30), -(1 << 30), -(1 << 30)};
      for (int pi : merged) {
        cx += cells[pi][0]; cy += cells[pi][1]; cz += cells[pi][2];
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], cells[pi][a]); hi[a] = std::max(hi[a], cells[pi][a]); }
      }
      double m = merged.size();
      ObjectNode o;
      o.ci = (int)std::lrint(cx / m); o.cj = (int)std::lrint(cy / m); o.ck = (int)std::lrint(cz / m);
      o.label = L; o.voxel_count = (int)merged.size();
      for (int a = 0; a < 3; ++a) { o.bmin[a] = lo[a]; o.bmax[a] = hi[a]; }
      out.push_back(o);
    }
  }
  return out;
}

}  // namespace smc
