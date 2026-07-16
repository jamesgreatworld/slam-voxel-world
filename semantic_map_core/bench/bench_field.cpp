// bench_field.cpp — 合成箱体房间, 算 ESDF+GVD, 写二进制供 Python 对拍, 计时。
#include "semantic_map_core/field.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace smc;

int main() {
  const int nx = 60, ny = 60, nz = 30;
  const long N = (long)nx * ny * nz;
  std::vector<uint8_t> occ(N, 0), free(N, 0);
  auto ID = [&](int i, int j, int k) { return (long)(i * ny + j) * nz + k; };
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j)
      for (int k = 0; k < nz; ++k) {
        bool inbox = i >= 8 && i <= 51 && j >= 8 && j <= 51 && k >= 4 && k <= 25;
        bool shell = inbox && (i == 8 || i == 51 || j == 8 || j == 51 || k == 4 || k == 25);
        if (shell) occ[ID(i, j, k)] = 1;
        if (i >= 9 && i <= 50 && j >= 9 && j <= 50 && k >= 5 && k <= 24) free[ID(i, j, k)] = 1;
      }
  const float vs = 0.1f;
  std::vector<float> dist;
  std::vector<long> parent;
  auto t0 = std::chrono::high_resolution_clock::now();
  compute_esdf(occ.data(), nx, ny, nz, vs, dist, parent);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto gvd = extract_gvd(free.data(), dist.data(), parent.data(), nx, ny, nz, vs);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto thin = thin_gvd(gvd.data(), nx, ny, nz);
  auto t3 = std::chrono::high_resolution_clock::now();

  FILE* fd = std::fopen("/mnt/hgfs/Shared/claude_jobs/field_dist.bin", "wb");
  std::fwrite(dist.data(), 4, N, fd); std::fclose(fd);
  FILE* fg = std::fopen("/mnt/hgfs/Shared/claude_jobs/field_gvd.bin", "wb");
  std::fwrite(gvd.data(), 1, N, fg); std::fclose(fg);
  FILE* ft = std::fopen("/mnt/hgfs/Shared/claude_jobs/field_thin.bin", "wb");
  std::fwrite(thin.data(), 1, N, ft); std::fclose(ft);

  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  double tms = std::chrono::duration<double, std::milli>(t3 - t2).count();
  long gc = 0; for (auto v : gvd) gc += v;
  long tc = 0; for (auto v : thin) tc += v;
  float dmax = *std::max_element(dist.begin(), dist.end());
  std::printf("C++  esdf=%.2fms  gvd=%ld  dist_max=%.3fm  thin=%.2fms thin_vox=%ld\n",
              ms, gc, dmax, tms, tc);
  return 0;
}
