// surface.cpp — 见 surface.hpp。移植自 surface.py(scipy ndimage.label + binary_dilation)。
#include "semantic_map_core/surface.hpp"

#include <cmath>
#include <queue>

namespace smc {

void occupancy_from_counts(const int* surf_hits, const int* obst_hits, int n,
                           int surf_min, int obst_min,
                           std::vector<uint8_t>& walkable, std::vector<int8_t>& cost) {
  walkable.assign(n, 0);
  cost.assign(n, -1);
  for (int i = 0; i < n; ++i) {
    bool w = (surf_hits[i] >= surf_min) && (obst_hits[i] < obst_min);
    bool b = (obst_hits[i] >= obst_min);
    walkable[i] = w ? 1 : 0;
    if (w) cost[i] = 0;
    if (b) cost[i] = 100;  // 与 Python 顺序一致: 先写 walkable=0, 再 blocked=100
  }
}

// 8 连通、光栅顺序首遇编号(复刻 scipy.ndimage.label 的连续编号): 行优先扫描,
// 遇到未标记的 walkable 格就 BFS 整个分量, label 递增。
static int label_cc(const uint8_t* walkable, int ny, int nx, std::vector<int>& lab) {
  lab.assign((size_t)ny * nx, 0);
  int next = 0;
  std::queue<int> q;
  for (int r = 0; r < ny; ++r) {
    for (int c = 0; c < nx; ++c) {
      int idx = r * nx + c;
      if (!walkable[idx] || lab[idx]) continue;
      ++next;
      lab[idx] = next;
      q.push(idx);
      while (!q.empty()) {
        int cur = q.front(); q.pop();
        int cr = cur / nx, cc = cur % nx;
        for (int dr = -1; dr <= 1; ++dr)
          for (int dc = -1; dc <= 1; ++dc) {
            if (!dr && !dc) continue;
            int nr = cr + dr, ncc = cc + dc;
            if (nr < 0 || nr >= ny || ncc < 0 || ncc >= nx) continue;
            int ni = nr * nx + ncc;
            if (walkable[ni] && !lab[ni]) { lab[ni] = next; q.push(ni); }
          }
      }
    }
  }
  return next;
}

// mask 就地 8 连通膨胀一步后 &passable。
static void dilate1_and(std::vector<uint8_t>& mask, const std::vector<uint8_t>& passable,
                        int ny, int nx, std::vector<uint8_t>& out) {
  out.assign((size_t)ny * nx, 0);
  for (int r = 0; r < ny; ++r)
    for (int c = 0; c < nx; ++c) {
      int idx = r * nx + c;
      bool any = false;
      for (int dr = -1; dr <= 1 && !any; ++dr)
        for (int dc = -1; dc <= 1 && !any; ++dc) {
          int nr = r + dr, ncc = c + dc;
          if (nr < 0 || nr >= ny || ncc < 0 || ncc >= nx) continue;
          if (mask[nr * nx + ncc]) any = true;
        }
      out[idx] = (any && passable[idx]) ? 1 : 0;
    }
}

SurfaceResult cluster_surface_places(const uint8_t* walkable, int ny, int nx,
                                     double origin_x, double origin_y, double res,
                                     int min_cells, double door_bridge_m,
                                     const uint8_t* blocked) {
  SurfaceResult out;
  std::vector<int> lab;
  int n = label_cc(walkable, ny, nx, lab);

  // 按 label 1..n 顺序: 小分量丢弃(lab=0), 否则建区域(id=保留计数, label_id=r)。
  for (int r = 1; r <= n; ++r) {
    long cnt = 0; double sr = 0, sc = 0;
    for (long i = 0; i < (long)ny * nx; ++i)
      if (lab[i] == r) { ++cnt; sr += i / nx; sc += i % nx; }
    if (cnt < min_cells) {
      for (long i = 0; i < (long)ny * nx; ++i) if (lab[i] == r) lab[i] = 0;
      continue;
    }
    double cen_r = sr / cnt, cen_c = sc / cnt;
    double wx = origin_x + (cen_c + 0.5) * res;
    double wy = origin_y + (cen_r + 0.5) * res;
    SurfRegion reg;
    reg.id = (int)out.regions.size();
    reg.cx = wx; reg.cy = wy;
    reg.area_m2 = (double)cnt * res * res;
    reg.cell_count = (int)cnt;
    reg.label_id = r;
    out.regions.push_back(reg);
  }

  // 门口相邻: 每区域从 (lab==label_id) 起, 测地膨胀 bridge 步(8连通, 每步 &passable),
  // 若够到另一区域的格 => 连边。
  int bridge = (int)std::rint(door_bridge_m / res);  // Python round=银行家舍入
  if (bridge < 1) bridge = 1;
  std::vector<uint8_t> passable((size_t)ny * nx, 1);
  if (blocked)
    for (long i = 0; i < (long)ny * nx; ++i) passable[i] = blocked[i] ? 0 : 1;

  std::vector<std::vector<uint8_t>> masks(out.regions.size());
  for (size_t ri = 0; ri < out.regions.size(); ++ri) {
    int lid = out.regions[ri].label_id;
    std::vector<uint8_t> m((size_t)ny * nx, 0), tmp;
    for (long i = 0; i < (long)ny * nx; ++i) m[i] = (lab[i] == lid) ? 1 : 0;
    for (int s = 0; s < bridge; ++s) { dilate1_and(m, passable, ny, nx, tmp); m.swap(tmp); }
    masks[ri] = std::move(m);
  }
  for (size_t ai = 0; ai < out.regions.size(); ++ai)
    for (size_t bi = ai + 1; bi < out.regions.size(); ++bi) {
      int lidb = out.regions[bi].label_id;
      bool hit = false;
      for (long i = 0; i < (long)ny * nx && !hit; ++i)
        if (masks[ai][i] && lab[i] == lidb) hit = true;
      if (hit) out.edges.emplace_back(out.regions[ai].id, out.regions[bi].id);
    }

  out.label_img = std::move(lab);
  return out;
}

}  // namespace smc
