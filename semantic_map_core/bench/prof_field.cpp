// prof_field.cpp — 在真实 House GVD 上剖析 esdf/gvd/thin 各阶段 + thin 内部瓶颈。
#include "semantic_map_core/field.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace smc;
static double ms(std::chrono::high_resolution_clock::time_point a,
                 std::chrono::high_resolution_clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

int main() {
  const char* O = "/mnt/hgfs/Shared/claude_jobs/";
  int nx, ny, nz;
  size_t rd;
  {
    FILE* f = std::fopen((std::string(O) + "house_dims.bin").c_str(), "rb");
    rd = std::fread(&nx, 4, 1, f); rd = std::fread(&ny, 4, 1, f); rd = std::fread(&nz, 4, 1, f);
    std::fclose(f);
  }
  const long N = (long)nx * ny * nz;
  std::vector<uint8_t> occ(N), free(N);
  {
    FILE* f = std::fopen((std::string(O) + "house_occ.bin").c_str(), "rb");
    rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((std::string(O) + "house_free.bin").c_str(), "rb");
    rd = std::fread(free.data(), 1, N, f); std::fclose(f);
  }
  (void)rd;
  const float vs = 0.1f;
  std::vector<float> dist; std::vector<long> parent;
  auto a = std::chrono::high_resolution_clock::now();
  compute_esdf(occ.data(), nx, ny, nz, vs, dist, parent);
  auto b = std::chrono::high_resolution_clock::now();
  auto gvd = extract_gvd(free.data(), dist.data(), parent.data(), nx, ny, nz, vs);
  auto c = std::chrono::high_resolution_clock::now();
  auto thin = thin_gvd(gvd.data(), nx, ny, nz);
  auto d = std::chrono::high_resolution_clock::now();

  FILE* fg = std::fopen((std::string(O) + "house_gvd_cpp.bin").c_str(), "wb");
  std::fwrite(gvd.data(), 1, N, fg); std::fclose(fg);
  FILE* ft = std::fopen((std::string(O) + "house_thin_cpp.bin").c_str(), "wb");
  std::fwrite(thin.data(), 1, N, ft); std::fclose(ft);

  long gc = 0; for (auto v : gvd) gc += v;
  long tc = 0; for (auto v : thin) tc += v;
  ThinProfile p = last_thin_profile();
  std::printf("C++  grid=%dx%dx%d=%ld\n", nx, ny, nz, N);
  std::printf("C++  esdf=%.1fms  gvd=%.1fms(%ld)  thin=%.1fms(%ld)\n",
              ms(a, b), ms(b, c), gc, ms(c, d), tc);
  std::printf("C++  [thin 剖析] passes=%ld  候选累计=%ld  find_candidates=%.1fms  复检=%.1fms\n",
              p.iters, p.candidates, p.find_ms, p.recheck_ms);
  return 0;
}
