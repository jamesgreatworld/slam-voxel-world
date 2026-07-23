// vlayer_objects.cpp — 见 vlayer_objects.hpp。逐行移植 vlayer/objects.py。
#include "semantic_map_core/vlayer_objects.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace smc {

// ===================== md5(RFC 1321)=====================
static uint32_t rotl(uint32_t x, int c) { return (x << c) | (x >> (32 - c)); }
std::string md5_hex(const std::string& msg) {
  static const uint32_t K[64] = {
      0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613,
      0xfd469501, 0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193,
      0xa679438e, 0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d,
      0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
      0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942, 0x8771f681, 0x6d9d6122,
      0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70, 0x289b7ec6, 0xeaa127fa,
      0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244,
      0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
      0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
      0xeb86d391};
  static const int S[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                            5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
                            4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                            6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
  std::vector<uint8_t> data(msg.begin(), msg.end());
  uint64_t bitlen = (uint64_t)data.size() * 8;
  data.push_back(0x80);
  while (data.size() % 64 != 56) data.push_back(0);
  for (int i = 0; i < 8; ++i) data.push_back((uint8_t)(bitlen >> (8 * i)));
  uint32_t a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
  for (size_t off = 0; off < data.size(); off += 64) {
    uint32_t M[16];
    for (int i = 0; i < 16; ++i)
      std::memcpy(&M[i], &data[off + 4 * i], 4);
    uint32_t A = a0, B = b0, C = c0, D = d0;
    for (int i = 0; i < 64; ++i) {
      uint32_t F; int g;
      if (i < 16) { F = (B & C) | (~B & D); g = i; }
      else if (i < 32) { F = (D & B) | (~D & C); g = (5 * i + 1) % 16; }
      else if (i < 48) { F = B ^ C ^ D; g = (3 * i + 5) % 16; }
      else { F = C ^ (B | ~D); g = (7 * i) % 16; }
      F = F + A + K[i] + M[g];
      A = D; D = C; C = B;
      B = B + rotl(F, S[i]);
    }
    a0 += A; b0 += B; c0 += C; d0 += D;
  }
  char out[33];
  uint32_t h[4] = {a0, b0, c0, d0};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      std::snprintf(out + i * 8 + j * 2, 3, "%02x", (h[i] >> (8 * j)) & 0xFF);
  return std::string(out, 32);
}
std::string stable_uuid(const std::string& seed) {
  std::string h = md5_hex(seed);
  return h.substr(0, 8) + "-" + h.substr(8, 4) + "-" + h.substr(12, 4) + "-" +
         h.substr(16, 4) + "-" + h.substr(20, 12);
}

static inline long ID(int x, int y, int z, int ny, int nz) {
  return ((long)x * ny + y) * nz + z;
}

std::vector<uint8_t> object_cells_mask(const uint8_t* occ, const uint8_t* sem,
                                       int nx, int ny, int nz, const std::set<int>& labels) {
  std::vector<uint8_t> mask((size_t)nx * ny * nz, 0);
  for (long i = 0; i < (long)nx * ny * nz; ++i)
    if (occ[i] && labels.count(sem[i])) mask[i] = 1;
  return mask;
}

using Cells = std::vector<std::array<int, 3>>;

// 26-连通 3D 连通分量(光栅首遇序), 返回每分量的 cells(argwhere 行主序)
static std::vector<Cells> label26_components(const std::vector<uint8_t>& mask,
                                             int nx, int ny, int nz) {
  std::vector<int> lbl((size_t)nx * ny * nz, 0);
  int ncomp = 0;
  std::vector<long> q;
  for (int x = 0; x < nx; ++x)
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z) {
        long s = ID(x, y, z, ny, nz);
        if (!mask[s] || lbl[s]) continue;
        ++ncomp; lbl[s] = ncomp; q.clear(); q.push_back(s);
        for (size_t h = 0; h < q.size(); ++h) {
          long id = q[h];
          int cx = (int)(id / ((long)ny * nz));
          long r = id % ((long)ny * nz);
          int cy = (int)(r / nz), cz = (int)(r % nz);
          for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
              for (int dz = -1; dz <= 1; ++dz) {
                if (!dx && !dy && !dz) continue;
                int nx2 = cx + dx, ny2 = cy + dy, nz2 = cz + dz;
                if (nx2 < 0 || nx2 >= nx || ny2 < 0 || ny2 >= ny || nz2 < 0 || nz2 >= nz) continue;
                long nid = ID(nx2, ny2, nz2, ny, nz);
                if (mask[nid] && !lbl[nid]) { lbl[nid] = ncomp; q.push_back(nid); }
              }
        }
      }
  std::vector<Cells> comps(ncomp);
  for (int x = 0; x < nx; ++x)     // argwhere(lbl==cid) 行主序
    for (int y = 0; y < ny; ++y)
      for (int z = 0; z < nz; ++z) {
        int c = lbl[ID(x, y, z, ny, nz)];
        if (c) comps[c - 1].push_back({x, y, z});
      }
  return comps;
}

static void aabb(const Cells& c, long lo[3], long hi[3]) {
  lo[0] = lo[1] = lo[2] = 1 << 30;
  hi[0] = hi[1] = hi[2] = -(1 << 30);
  for (auto& p : c)
    for (int a = 0; a < 3; ++a) {
      lo[a] = std::min(lo[a], (long)p[a]);
      hi[a] = std::max(hi[a], (long)p[a]);
    }
}

// _merge_overlapping_components(gap=2): AABB 空隙<=gap 两两合并至收敛, 输出按 min 角排序
static std::vector<Cells> merge_overlapping(std::vector<Cells> merged, int gap = 2) {
  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<Cells> out;
    while (!merged.empty()) {
      Cells cur = std::move(merged.front());
      merged.erase(merged.begin());
      long lo1[3], hi1[3];
      aabb(cur, lo1, hi1);
      std::vector<Cells> keep;
      for (auto& other : merged) {
        long lo2[3], hi2[3];
        aabb(other, lo2, hi2);
        bool adj = true;
        for (int a = 0; a < 3 && adj; ++a)
          if (!(lo1[a] <= hi2[a] + gap + 1 && lo2[a] <= hi1[a] + gap + 1)) adj = false;
        if (adj) {
          cur.insert(cur.end(), other.begin(), other.end());
          changed = true;
          aabb(cur, lo1, hi1);
        } else keep.push_back(std::move(other));
      }
      merged = std::move(keep);
      out.push_back(std::move(cur));
    }
    merged = std::move(out);
  }
  std::stable_sort(merged.begin(), merged.end(), [](const Cells& a, const Cells& b) {
    long la[3], ha[3], lb2[3], hb2[3];
    aabb(a, la, ha); aabb(b, lb2, hb2);
    for (int i = 0; i < 3; ++i) {
      if (la[i] != lb2[i]) return la[i] < lb2[i];
    }
    return false;
  });
  return merged;
}

// _mount_of(reach=8): ceiling / wall / floor
static std::string mount_of(const Cells& cells, const uint8_t* occ, const uint8_t* sem,
                            int nx, int ny, int nz, int reach = 8) {
  auto is_struct = [&](long id) {
    return occ[id] && (sem[id] == 3 || sem[id] == 4 || sem[id] == 19);
  };
  auto is_wall = [&](long id) { return occ[id] && sem[id] == 19; };
  int top = -1, bot = 1 << 30;
  for (auto& c : cells) { top = std::max(top, c[1]); bot = std::min(bot, c[1]); }
  bool above = false, below = false;
  for (auto& c : cells) {
    if (c[1] == top && !above)
      for (int y = top + 1; y < std::min(ny, top + 1 + reach); ++y)
        if (is_struct(ID(c[0], y, c[2], ny, nz))) { above = true; break; }
    if (c[1] == bot && !below)
      for (int y = std::max(0, bot - reach); y < bot; ++y)
        if (is_struct(ID(c[0], y, c[2], ny, nz))) { below = true; break; }
  }
  if (above && !below) return "ceiling";
  if (!below) {
    int step = std::max(1, (int)cells.size() / 64);
    for (size_t i = 0; i < cells.size(); i += step) {
      int x = cells[i][0], y = cells[i][1], z = cells[i][2];
      bool hit = false;
      for (int xx = std::max(0, x - reach); xx < std::min(nx, x + reach + 1) && !hit; ++xx)
        if (is_wall(ID(xx, y, z, ny, nz))) hit = true;
      for (int zz = std::max(0, z - reach); zz < std::min(nz, z + reach + 1) && !hit; ++zz)
        if (is_wall(ID(x, y, zz, ny, nz))) hit = true;
      if (hit) return "wall";
    }
  }
  return "floor";
}

// ---------------- resolve_placements ----------------
static void resolve_placements(std::vector<EntityModel>& ents, const uint8_t* occ,
                               const uint8_t* sem, int nx, int ny, int nz, const long vmin[3],
                               double vs, double max_push_m,
                               const std::map<std::string, std::array<double, 3>>& preset_extents,
                               double coarse_vs) {
  auto rdims = [&](const EntityModel& e, double d[3]) {
    auto it = e.mc_item.empty() ? preset_extents.end() : preset_extents.find(e.mc_item);
    if (it != preset_extents.end()) { d[0] = it->second[0]; d[1] = it->second[1]; d[2] = it->second[2]; }
    else { d[0] = e.bbox_dims[0]; d[1] = e.bbox_dims[1]; d[2] = e.bbox_dims[2]; }
  };
  auto to_cell = [&](const double p[3], long c[3]) {
    for (int a = 0; a < 3; ++a) c[a] = (long)std::floor(p[a] / vs) - vmin[a];
  };
  auto is_floor = [&](long id) { return occ[id] && sem[id] == 3; };
  auto is_ceil = [&](long id) { return occ[id] && sem[id] == 4; };
  auto is_wall = [&](long id) { return occ[id] && sem[id] == 19; };
  int margin = (int)std::lround(0.4 / vs);
  int depth = (int)std::lround(3.0 / vs);

  for (auto& e : ents) {
    for (int a = 0; a < 3; ++a) {
      e.obs_pos[a] = e.position[a];
      e.obs_half[a] = e.bbox_dims[a] / 2.0;
    }
  }

  auto find_floor_top = [&](const double pos[3], const double half[3]) -> std::pair<bool, double> {
    long c[3]; to_cell(pos, c);
    int x0 = std::max(0L, c[0] - (long)(half[0] / vs) - margin);
    int x1 = std::min((long)nx, c[0] + (long)(half[0] / vs) + 1 + margin);
    int z0 = std::max(0L, c[2] - (long)(half[2] / vs) - margin);
    int z1 = std::min((long)nz, c[2] + (long)(half[2] / vs) + 1 + margin);
    int cy = (int)std::min(std::max(c[1], 0L), (long)ny - 1);
    int lo = std::max(0, cy - depth);
    int found = -1;
    for (int y = lo; y < std::min(ny, cy + 2); ++y) {    // col.any 后取最后一个 y
      bool anyf = false;
      for (int x = x0; x < x1 && !anyf; ++x)
        for (int z = z0; z < z1 && !anyf; ++z)
          if (is_floor(ID(x, y, z, ny, nz))) anyf = true;
      if (anyf) found = y - lo;
    }
    if (found >= 0) return {true, (lo + found + 1 + vmin[1]) * vs};
    // 回退: footprint 内逐列 "cy 以下最高占据" 的中位数
    std::vector<int> tops;
    for (int x = x0; x < x1; ++x)
      for (int z = z0; z < z1; ++z) {
        int t = -1;
        for (int y = 0; y <= cy; ++y)
          if (occ[ID(x, y, z, ny, nz)]) t = y;
        if (t >= 0) tops.push_back(t);
      }
    if (!tops.empty()) {
      std::sort(tops.begin(), tops.end());
      double med;
      size_t n = tops.size();
      if (n % 2) med = tops[n / 2];
      else med = (tops[n / 2 - 1] + tops[n / 2]) / 2.0;   // np.median 偶数取均值
      long mi = (long)med;                                 // Python int() 截断(值非负)
      return {true, (mi + 1 + vmin[1]) * vs};
    }
    return {false, 0.0};
  };

  // 1. 支撑感知吸附(按观测底面升序, stable)
  std::vector<EntityModel*> byb;
  for (auto& e : ents) byb.push_back(&e);
  std::stable_sort(byb.begin(), byb.end(), [](EntityModel* a, EntityModel* b) {
    return (a->obs_pos[1] - a->obs_half[1]) < (b->obs_pos[1] - b->obs_half[1]);
  });
  std::vector<EntityModel*> placed;
  for (EntityModel* e : byb) {
    double pos[3] = {e->obs_pos[0], e->obs_pos[1], e->obs_pos[2]};
    double half[3]; rdims(*e, half);
    for (double& h : half) h /= 2.0;
    if (e->mount == "wall") { placed.push_back(e); continue; }
    if (e->mount == "ceiling") {
      long c[3]; to_cell(pos, c);
      int x0 = std::max(0L, c[0] - (long)(half[0] / vs) - margin);
      int x1 = std::min((long)nx, c[0] + (long)(half[0] / vs) + 1 + margin);
      int z0 = std::max(0L, c[2] - (long)(half[2] / vs) - margin);
      int z1 = std::min((long)nz, c[2] + (long)(half[2] / vs) + 1 + margin);
      int cy = (int)std::min(std::max(c[1], 0L), (long)ny - 1);
      int found = -1;
      for (int y = cy; y < std::min(ny, cy + depth); ++y) {
        bool anyc = false;
        for (int x = x0; x < x1 && !anyc; ++x)
          for (int z = z0; z < z1 && !anyc; ++z)
            if (is_ceil(ID(x, y, z, ny, nz))) anyc = true;
        if (anyc) { found = y - cy; break; }               // ys[0] = 第一个
      }
      if (found >= 0) {
        double ceil_under = (cy + found + vmin[1]) * vs;
        if (coarse_vs > 0) ceil_under = std::floor(ceil_under / coarse_vs) * coarse_vs;
        pos[1] = ceil_under - half[1];
      }
    } else {
      double obs_bottom = e->obs_pos[1] - e->obs_half[1];
      auto ft = find_floor_top(pos, half);
      bool has_t = ft.first;
      double target = ft.second;
      if (has_t && coarse_vs > 0)
        target = std::ceil(target / coarse_vs - 1e-9) * coarse_vs;
      EntityModel* parent = nullptr;
      for (EntityModel* f : placed) {
        if (f->mount == "wall") continue;
        double fh[3]; rdims(*f, fh);
        for (double& h : fh) h /= 2.0;
        if (std::fabs(pos[0] - f->position[0]) > fh[0] + 0.05 ||
            std::fabs(pos[2] - f->position[2]) > fh[2] + 0.05)
          continue;
        double f_obs_top = f->obs_pos[1] + f->obs_half[1];
        if (obs_bottom - f_obs_top >= -0.35 && obs_bottom - f_obs_top <= 0.75) {
          double f_top = f->position[1] + fh[1];
          if (!has_t || f_top > target) { target = f_top; has_t = true; parent = f; }
        }
      }
      if (has_t) {
        pos[1] = target + half[1];
        if (parent) e->support_id = parent->id;
      }
    }
    // 2. 推出墙体
    {
      long c[3]; to_cell(pos, c);
      int y0 = std::max(0L, c[1] - (long)(half[1] / vs) + 1);
      int y1 = std::min((long)ny, c[1] + (long)(half[1] / vs));
      int x0 = std::max(0L, c[0] - (long)(half[0] / vs));
      int x1 = std::min((long)nx, c[0] + (long)(half[0] / vs) + 1);
      int z0 = std::max(0L, c[2] - (long)(half[2] / vs));
      int z1 = std::min((long)nz, c[2] + (long)(half[2] / vs) + 1);
      int oxmin = 1 << 30, oxmax = -(1 << 30), ozmin = 1 << 30, ozmax = -(1 << 30);
      bool anyov = false;
      for (int x = x0; x < x1; ++x)
        for (int y = y0; y < y1; ++y)
          for (int z = z0; z < z1; ++z)
            if (is_wall(ID(x, y, z, ny, nz))) {
              anyov = true;
              oxmin = std::min(oxmin, x - x0); oxmax = std::max(oxmax, x - x0);
              ozmin = std::min(ozmin, z - z0); ozmax = std::max(ozmax, z - z0);
            }
      if (anyov) {
        int spanx = oxmax - oxmin + 1, spanz = ozmax - ozmin + 1;
        int axis = (spanx <= spanz) ? 0 : 2;
        int lo2 = (axis == 0) ? oxmin : ozmin;
        int hi2 = (axis == 0) ? oxmax : ozmax;
        int ext = (axis == 0) ? (x1 - x0) : (z1 - z0);
        long push_cells = (lo2 <= ext - 1 - hi2) ? (hi2 + 1) : -(long)(ext - lo2);
        double push = std::min(std::max((double)push_cells * vs, -max_push_m), max_push_m);
        pos[axis == 0 ? 0 : 2] += push;
      }
    }
    e->position[0] = pos[0]; e->position[1] = pos[1]; e->position[2] = pos[2];
    placed.push_back(e);
  }

  // 3. 两两分离(order 按 -渲染体积, stable)
  std::vector<int> order(ents.size());
  for (size_t i = 0; i < ents.size(); ++i) order[i] = (int)i;
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    double da[3], db[3]; rdims(ents[a], da); rdims(ents[b], db);
    return -(da[0] * da[1] * da[2]) < -(db[0] * db[1] * db[2]);
  });
  auto overlaps_any = [&](int k, const double pk[3]) {
    double hk[3]; rdims(ents[k], hk);
    for (double& h : hk) h /= 2.0;
    for (int mI : order) {
      if (mI == k) continue;
      double hm[3]; rdims(ents[mI], hm);
      bool all = true;
      for (int a = 0; a < 3 && all; ++a) {
        double om = (hk[a] + hm[a] / 2.0) - std::fabs(pk[a] - ents[mI].position[a]);
        if (!(om > 1e-6)) all = false;
      }
      if (all) return true;
    }
    return false;
  };
  for (int round = 0; round < 3; ++round) {
    bool moved = false;
    for (size_t ii = 0; ii < order.size(); ++ii)
      for (size_t jj = ii + 1; jj < order.size(); ++jj) {
        EntityModel& a = ents[order[ii]];
        EntityModel& b = ents[order[jj]];
        if (a.support_id == b.id || b.support_id == a.id) continue;
        double ha[3], hb[3];
        rdims(a, ha); rdims(b, hb);
        double ov[3]; bool all = true;
        for (int ax = 0; ax < 3; ++ax) {
          ov[ax] = (ha[ax] / 2 + hb[ax] / 2) - std::fabs(a.position[ax] - b.position[ax]);
          if (!(ov[ax] > 1e-6)) all = false;
        }
        if (!all) continue;
        int axes[2] = {0, 2};
        if (!(ov[0] <= ov[2])) { axes[0] = 2; axes[1] = 0; }
        bool done = false;
        for (int t = 0; t < 2 && !done; ++t) {
          int ax = axes[t];
          double sign = (b.position[ax] >= a.position[ax]) ? 1.0 : -1.0;
          double cand[3] = {b.position[0], b.position[1], b.position[2]};
          cand[ax] += sign * std::min(ov[ax], max_push_m);
          if (!overlaps_any(order[jj], cand)) {
            b.position[0] = cand[0]; b.position[1] = cand[1]; b.position[2] = cand[2];
            done = true;
          }
        }
        if (!done) {
          int ax = axes[0];
          double sign = (b.position[ax] >= a.position[ax]) ? 1.0 : -1.0;
          b.position[ax] += sign * std::min(ov[ax], max_push_m);
        }
        moved = true;
      }
    if (!moved) break;
  }

  // 子随父
  std::map<std::string, EntityModel*> by_id;
  for (auto& e : ents) by_id[e.id] = &e;
  for (auto& e : ents) {
    if (e.support_id.empty()) continue;
    auto it = by_id.find(e.support_id);
    if (it == by_id.end()) continue;
    double fh[3], eh[3];
    rdims(*it->second, fh); rdims(e, eh);
    e.position[1] = it->second->position[1] + fh[1] / 2.0 + eh[1] / 2.0;
  }
}

std::vector<EntityModel> extract_object_models(
    const uint8_t* occ, const uint8_t* sem, int nx, int ny, int nz, const long vmin[3],
    double voxel_size, const std::map<int, std::string>& label_names,
    const std::map<int, std::string>& mc_item_map, int min_voxels,
    const std::set<int>& labels, double max_extent_m,
    const std::map<std::string, std::array<double, 3>>& preset_extents, double coarse_vs) {
  std::vector<EntityModel> ents;
  for (int lab : labels) {   // std::set 迭代 = sorted(labels)
    std::string preset;
    auto pit = mc_item_map.find(lab);
    if (pit != mc_item_map.end()) preset = pit->second;
    std::vector<uint8_t> mask((size_t)nx * ny * nz, 0);
    for (long i = 0; i < (long)nx * ny * nz; ++i)
      if (occ[i] && sem[i] == lab) mask[i] = 1;
    auto comps = label26_components(mask, nx, ny, nz);
    std::vector<Cells> keep;
    int prefilter = std::max(4, min_voxels / 4);
    for (auto& c : comps)
      if ((int)c.size() >= prefilter) keep.push_back(std::move(c));
    auto merged = merge_overlapping(std::move(keep));
    int cid = 0;
    for (auto& cells : merged) {
      ++cid;
      if ((int)cells.size() < min_voxels) continue;
      long lo[3], hi[3];
      aabb(cells, lo, hi);
      double centre[3], dims[3];
      for (int a = 0; a < 3; ++a) {
        double cmin = (double)(lo[a] + vmin[a]), cmax = (double)(hi[a] + vmin[a]);
        centre[a] = (cmin + cmax + 1.0) * 0.5 * voxel_size;
        dims[a] = (cmax - cmin + 1.0) * voxel_size;
      }
      if (std::max(dims[0], std::max(dims[1], dims[2])) > max_extent_m) continue;
      EntityModel e;
      char seed[128];
      std::snprintf(seed, sizeof(seed), "%d-%d-%.3f-%.3f", lab, cid, centre[0], centre[2]);
      e.id = stable_uuid(seed);
      e.label = lab;
      auto nit = label_names.find(lab);
      e.label_name = nit != label_names.end() ? nit->second : std::to_string(lab);
      for (int a = 0; a < 3; ++a) { e.position[a] = centre[a]; e.bbox_dims[a] = dims[a]; }
      e.voxel_count = (int)cells.size();
      e.mount = mount_of(cells, occ, sem, nx, ny, nz);
      if (e.mount == "floor") e.mount.clear();          // Python: floor 不写 meta
      if (!preset.empty() && e.mount.empty()) e.mc_item = preset;
      ents.push_back(std::move(e));
    }
  }
  resolve_placements(ents, occ, sem, nx, ny, nz, vmin, voxel_size, 0.5, preset_extents, coarse_vs);
  return ents;
}

void snap_small_to_support(std::vector<EntityModel>& small_ents,
                           const std::vector<EntityModel>& furn_ents,
                           const std::map<std::string, std::array<double, 3>>& preset_extents,
                           double max_drop_m) {
  auto rdims = [&](const EntityModel& e, double d[3]) {
    auto it = e.mc_item.empty() ? preset_extents.end() : preset_extents.find(e.mc_item);
    if (it != preset_extents.end()) { d[0] = it->second[0]; d[1] = it->second[1]; d[2] = it->second[2]; }
    else { d[0] = e.bbox_dims[0]; d[1] = e.bbox_dims[1]; d[2] = e.bbox_dims[2]; }
  };
  for (auto& s : small_ents) {
    double pos[3] = {s.position[0], s.position[1], s.position[2]};
    double half_h = s.bbox_dims[1] / 2.0;
    double bottom = pos[1] - half_h;
    bool inside = false, has_best = false;
    double best_top = 0;
    for (auto& f : furn_ents) {
      double fh[3]; rdims(f, fh);
      for (double& h : fh) h /= 2.0;
      if (std::fabs(pos[0] - f.position[0]) > fh[0] || std::fabs(pos[2] - f.position[2]) > fh[2])
        continue;
      double top = f.position[1] + fh[1], bot = f.position[1] - fh[1];
      if (bot - 0.02 < pos[1] && pos[1] < top + 0.02) { inside = true; break; }
      if (top <= bottom + 0.05 && (!has_best || top > best_top)) { best_top = top; has_best = true; }
    }
    if (inside) continue;
    if (has_best && bottom - best_top <= max_drop_m) {
      s.position[1] = best_top + half_h;
      s.support = "furniture";
    }
  }
}

}  // namespace smc
