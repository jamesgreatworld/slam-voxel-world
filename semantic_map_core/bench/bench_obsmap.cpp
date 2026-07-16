// bench_obsmap.cpp — 确定性合成场景(箱体房间), 与 Python 版逐点相同, 对拍+计时。
#include "semantic_map_core/obsmap.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace smc;

int main() {
  std::vector<float> pts;
  std::vector<uint8_t> labs;
  // +0.05 让点落在 voxel 中心, 避开 floor 在 voxel 边界的 float 摇摆(纯测试用)
  auto add = [&](double x, double y, double z, uint8_t L) {
    pts.push_back((float)(x + 0.05)); pts.push_back((float)(y + 0.05));
    pts.push_back((float)(z + 0.05)); labs.push_back(L);
  };
  for (int xi = 0; xi < 80; ++xi)
    for (int zi = 0; zi < 30; ++zi) { add(xi * 0.1, 0, zi * 0.1, 1); add(xi * 0.1, 6, zi * 0.1, 1); }
  for (int yi = 0; yi < 60; ++yi)
    for (int zi = 0; zi < 30; ++zi) { add(0, yi * 0.1, zi * 0.1, 1); add(8, yi * 0.1, zi * 0.1, 1); }
  for (int xi = 0; xi < 80; ++xi)
    for (int yi = 0; yi < 60; ++yi) add(xi * 0.1, yi * 0.1, 0, 2);
  const int npts = (int)labs.size();

  ObsMap m(100, 80, 50, {-10, -10, -10}, 0.1f);
  const int F = 30;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int f = 0; f < F; ++f) {
    double t = (double)f / (F - 1);
    float origin[3] = {(float)(1 + 6 * t), 3.f, 1.f};
    m.integrate_frame(origin, pts.data(), npts, labs.data(), 0.10f);
    if (f < 3) {
      auto o = m.occupancy_mask();
      long c = 0; for (auto v : o) c += v;
      std::printf("  [C++ f=%d] occ=%ld  hit_cells=%ld miss_cells=%ld\n",
                  f, c, m.last_hit_cells, m.last_miss_cells);
    }
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  auto occ = m.occupancy_mask();
  long c = 0;
  for (auto v : occ) c += v;
  std::printf("C++  frames=%d pts/frame=%d  occ=%ld  total=%.1fms  per-frame=%.2fms\n",
              F, npts, c, ms, ms / F);
  return 0;
}
