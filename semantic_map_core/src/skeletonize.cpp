// skeletonize.cpp — 3D 骨架细化(Lee 1994), 逐行移植自 skimage 0.26.0
// _skeletonize_lee_cy.pyx.in(Arganda-Carreras 的 ITK 版)。用于 thin_gvd。
// padded(1 圈零)-> 双趟删简单点 -> crop。与 skimage.skeletonize 逐格一致为目标。
#include "semantic_map_core/field.hpp"

#include <chrono>
#include <cstring>
#include <vector>

namespace smc {

static ThinProfile g_prof;
ThinProfile last_thin_profile() { return g_prof; }

namespace {
using Clock = std::chrono::high_resolution_clock;
inline double ms_since(Clock::time_point a) {
  return std::chrono::duration<double, std::milli>(Clock::now() - a).count();
}

// ---- Euler LUT: arr[128] 是 Lee 表2的 δG26 列; LUT[2k+1]=arr[k], 偶数位=0 ----
int EULER_LUT[256];
bool init_euler() {
  static const int arr[128] = {
      1, -1, -1, 1, -3, -1, -1, 1, -1, 1, 1, -1, 3, 1, 1, -1, -3, -1,
      3, 1, 1, -1, 3, 1, -1, 1, 1, -1, 3, 1, 1, -1, -3, 3, -1, 1, 1,
      3, -1, 1, -1, 1, 1, -1, 3, 1, 1, -1, 1, 3, 3, 1, 5, 3, 3, 1,
      -1, 1, 1, -1, 3, 1, 1, -1, -7, -1, -1, 1, -3, -1, -1, 1, -1,
      1, 1, -1, 3, 1, 1, -1, -3, -1, 3, 1, 1, -1, 3, 1, -1, 1, 1,
      -1, 3, 1, 1, -1, -3, 3, -1, 1, 1, 3, -1, 1, -1, 1, 1, -1, 3,
      1, 1, -1, 1, 3, 3, 1, 5, 3, 3, 1, -1, 1, 1, -1, 3, 1, 1, -1};
  for (int i = 0; i < 256; ++i) EULER_LUT[i] = 0;
  for (int k = 0; k < 128; ++k) EULER_LUT[2 * k + 1] = arr[k];
  return true;
}
const bool kEulerInit = init_euler();

// 8 卦限的 7 个邻居索引(is_Euler_invariant 用)
const int NEIGHB_IDX[8][7] = {
    {2, 1, 11, 10, 5, 4, 14}, {0, 9, 3, 12, 1, 10, 4},
    {8, 7, 17, 16, 5, 4, 14}, {6, 15, 7, 16, 3, 12, 4},
    {20, 23, 19, 22, 11, 14, 10}, {18, 21, 9, 12, 19, 22, 10},
    {26, 23, 17, 14, 25, 22, 16}, {24, 25, 15, 16, 21, 22, 12}};

// octree_labeling 表: 每卦限 7 个 cube 索引 + 各自的邻接卦限(<=3 个)
const int OCTREE_IDX[8][7] = {
    {0, 1, 3, 4, 9, 10, 12}, {1, 4, 10, 2, 5, 11, 13},
    {3, 4, 12, 6, 7, 14, 15}, {4, 5, 13, 7, 15, 8, 16},
    {9, 10, 12, 17, 18, 20, 21}, {10, 11, 13, 18, 21, 19, 22},
    {12, 14, 15, 20, 21, 23, 24}, {13, 15, 16, 21, 22, 24, 25}};
const int OCTREE_NCNT[8][7] = {
    {0, 1, 1, 3, 1, 3, 3}, {1, 3, 3, 0, 1, 1, 3}, {1, 3, 3, 0, 1, 1, 3},
    {3, 1, 3, 1, 3, 0, 1}, {1, 3, 3, 0, 1, 1, 3}, {3, 1, 3, 1, 3, 0, 1},
    {3, 1, 3, 1, 3, 0, 1}, {3, 3, 1, 3, 1, 1, 0}};
const int OCTREE_NEW[8][7][3] = {
    {{0, 0, 0}, {2, 0, 0}, {3, 0, 0}, {2, 3, 4}, {5, 0, 0}, {2, 5, 6}, {3, 5, 7}},
    {{1, 0, 0}, {1, 3, 4}, {1, 5, 6}, {0, 0, 0}, {4, 0, 0}, {6, 0, 0}, {4, 6, 8}},
    {{1, 0, 0}, {1, 2, 4}, {1, 5, 7}, {0, 0, 0}, {4, 0, 0}, {7, 0, 0}, {4, 7, 8}},
    {{1, 2, 3}, {2, 0, 0}, {2, 6, 8}, {3, 0, 0}, {3, 7, 8}, {0, 0, 0}, {8, 0, 0}},
    {{1, 0, 0}, {1, 2, 6}, {1, 3, 7}, {0, 0, 0}, {6, 0, 0}, {7, 0, 0}, {6, 7, 8}},
    {{1, 2, 5}, {2, 0, 0}, {2, 4, 8}, {5, 0, 0}, {5, 7, 8}, {0, 0, 0}, {8, 0, 0}},
    {{1, 3, 5}, {3, 0, 0}, {3, 4, 8}, {5, 0, 0}, {5, 6, 8}, {0, 0, 0}, {8, 0, 0}},
    {{2, 4, 6}, {3, 4, 7}, {4, 0, 0}, {5, 6, 7}, {6, 0, 0}, {7, 0, 0}, {0, 0, 0}}};

void octree_labeling(int octant, int label, uint8_t* cube) {
  const int o = octant - 1;
  for (int t = 0; t < 7; ++t) {
    int idx = OCTREE_IDX[o][t];
    if (cube[idx] == 1) {
      cube[idx] = (uint8_t)label;
      for (int m = 0; m < OCTREE_NCNT[o][t]; ++m)
        octree_labeling(OCTREE_NEW[o][t][m], label, cube);
    }
  }
}

inline void get_neighborhood(const uint8_t* img, long RR, long CC, long p, long r,
                             long c, uint8_t* nb) {
  auto A = [&](long dp, long dr, long dc) -> uint8_t {
    return img[(((p + dp) * RR + (r + dr)) * CC + (c + dc))];
  };
  nb[0] = A(-1, -1, -1); nb[1] = A(-1, 0, -1); nb[2] = A(-1, 1, -1);
  nb[3] = A(-1, -1, 0);  nb[4] = A(-1, 0, 0);  nb[5] = A(-1, 1, 0);
  nb[6] = A(-1, -1, 1);  nb[7] = A(-1, 0, 1);  nb[8] = A(-1, 1, 1);
  nb[9] = A(0, -1, -1);  nb[10] = A(0, 0, -1); nb[11] = A(0, 1, -1);
  nb[12] = A(0, -1, 0);  nb[13] = A(0, 0, 0);  nb[14] = A(0, 1, 0);
  nb[15] = A(0, -1, 1);  nb[16] = A(0, 0, 1);  nb[17] = A(0, 1, 1);
  nb[18] = A(1, -1, -1); nb[19] = A(1, 0, -1); nb[20] = A(1, 1, -1);
  nb[21] = A(1, -1, 0);  nb[22] = A(1, 0, 0);  nb[23] = A(1, 1, 0);
  nb[24] = A(1, -1, 1);  nb[25] = A(1, 0, 1);  nb[26] = A(1, 1, 1);
}

inline bool is_endpoint(const uint8_t* nb) {
  int s = 0;
  for (int j = 0; j < 27; ++j) s += nb[j];
  return s == 2;
}

inline bool is_Euler_invariant(const uint8_t* nb) {
  int euler = 0;
  for (int oct = 0; oct < 8; ++oct) {
    int n = 1;
    for (int j = 0; j < 7; ++j)
      if (nb[NEIGHB_IDX[oct][j]] == 1) n |= (1 << (7 - j));
    euler += EULER_LUT[n];
  }
  return euler == 0;
}

bool is_simple_point(const uint8_t* nb) {
  uint8_t cube[26];
  std::memcpy(cube, nb, 13);
  std::memcpy(cube + 13, nb + 14, 13);
  int label = 2;
  for (int i = 0; i < 26; ++i) {
    if (cube[i] == 1) {
      int oc;
      if (i == 0 || i == 1 || i == 3 || i == 4 || i == 9 || i == 10 || i == 12) oc = 1;
      else if (i == 2 || i == 5 || i == 11 || i == 13) oc = 2;
      else if (i == 6 || i == 7 || i == 14 || i == 15) oc = 3;
      else if (i == 8 || i == 16) oc = 4;
      else if (i == 17 || i == 18 || i == 20 || i == 21) oc = 5;
      else if (i == 19 || i == 22) oc = 6;
      else if (i == 23 || i == 24) oc = 7;
      else oc = 8;  // i == 25
      octree_labeling(oc, label, cube);
      ++label;
      if (label - 2 >= 2) return false;
    }
  }
  return true;
}

struct Coord { long p, r, c; };

void find_candidates(const uint8_t* img, long PP, long RR, long CC, int cb,
                     std::vector<Coord>& out) {
  out.clear();
  uint8_t nb[27];
  for (long p = 1; p < PP - 1; ++p)
    for (long r = 1; r < RR - 1; ++r)
      for (long c = 1; c < CC - 1; ++c) {
        long id = (p * RR + r) * CC + c;
        if (img[id] != 1) continue;
        bool border =
            (cb == 1 && img[(p * RR + r) * CC + (c - 1)] == 0) ||
            (cb == 2 && img[(p * RR + r) * CC + (c + 1)] == 0) ||
            (cb == 3 && img[(p * RR + (r + 1)) * CC + c] == 0) ||
            (cb == 4 && img[(p * RR + (r - 1)) * CC + c] == 0) ||
            (cb == 5 && img[((p + 1) * RR + r) * CC + c] == 0) ||
            (cb == 6 && img[((p - 1) * RR + r) * CC + c] == 0);
        if (!border) continue;
        get_neighborhood(img, RR, CC, p, r, c, nb);
        if (is_endpoint(nb) || !is_Euler_invariant(nb) || !is_simple_point(nb)) continue;
        out.push_back({p, r, c});
      }
}

void compute_thin(uint8_t* img, long PP, long RR, long CC) {
  const int borders[6] = {4, 3, 2, 1, 5, 6};
  const int num_borders = (PP == 3) ? 4 : 6;
  std::vector<Coord> cand;
  uint8_t nb[27];
  int unchanged = 0;
  g_prof = ThinProfile{};
  while (unchanged < num_borders) {
    unchanged = 0;
    for (int j = 0; j < num_borders; ++j) {
      int cb = borders[j];
      auto t0 = Clock::now();
      find_candidates(img, PP, RR, CC, cb, cand);
      g_prof.find_ms += ms_since(t0);
      g_prof.iters += 1;
      g_prof.candidates += (long)cand.size();
      auto t1 = Clock::now();
      bool no_change = true;
      for (auto& pt : cand) {
        get_neighborhood(img, RR, CC, pt.p, pt.r, pt.c, nb);
        if (is_simple_point(nb)) {
          img[(pt.p * RR + pt.r) * CC + pt.c] = 0;
          no_change = false;
        }
      }
      g_prof.recheck_ms += ms_since(t1);
      if (no_change) ++unchanged;
    }
  }
}

}  // namespace

std::vector<uint8_t> thin_gvd(const uint8_t* gvd, int nx, int ny, int nz) {
  const long PP = nx + 2, RR = ny + 2, CC = nz + 2;
  std::vector<uint8_t> img((size_t)PP * RR * CC, 0);
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j)
      for (int k = 0; k < nz; ++k)
        if (gvd[(long)(i * ny + j) * nz + k])
          img[(((long)(i + 1)) * RR + (j + 1)) * CC + (k + 1)] = 1;
  compute_thin(img.data(), PP, RR, CC);
  std::vector<uint8_t> out((size_t)nx * ny * nz, 0);
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j)
      for (int k = 0; k < nz; ++k)
        out[(long)(i * ny + j) * nz + k] =
            img[(((long)(i + 1)) * RR + (j + 1)) * CC + (k + 1)];
  return out;
}

}  // namespace smc
