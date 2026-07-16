// bench_surface.cpp — C++ occupancy_from_counts + cluster_surface_places(House 投影 2D),
// dump regions/edges/label_img/cost 供 Python 逐项对拍。
#include "semantic_map_core/surface.hpp"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace smc;
int main() {
  const std::string O = "/mnt/hgfs/Shared/claude_jobs/";
  int ny, nx; size_t rd;
  { FILE* f = std::fopen((O + "surf_dims.bin").c_str(), "rb");
    rd = std::fread(&ny, 4, 1, f); rd = std::fread(&nx, 4, 1, f); std::fclose(f); }
  const long N = (long)ny * nx;
  std::vector<int> surf(N), obst(N);
  { FILE* f = std::fopen((O + "surf_hits.bin").c_str(), "rb"); rd = std::fread(surf.data(), 4, N, f); std::fclose(f);
    f = std::fopen((O + "obst_hits.bin").c_str(), "rb"); rd = std::fread(obst.data(), 4, N, f); std::fclose(f); }
  (void)rd;

  std::vector<uint8_t> walkable; std::vector<int8_t> cost;
  occupancy_from_counts(surf.data(), obst.data(), (int)N, 1, 2, walkable, cost);
  std::vector<uint8_t> blocked(N);
  for (long i = 0; i < N; ++i) blocked[i] = (obst[i] >= 2) ? 1 : 0;

  auto res = cluster_surface_places(walkable.data(), ny, nx, 0.0, 0.0, 0.1, 15, 0.9, blocked.data());
  std::printf("C++  regions=%zu edges=%zu\n", res.regions.size(), res.edges.size());

  FILE* f = std::fopen((O + "surf_cpp.bin").c_str(), "wb");
  int nr = (int)res.regions.size(); std::fwrite(&nr, 4, 1, f);
  for (auto& r : res.regions) {
    std::fwrite(&r.id, 4, 1, f); std::fwrite(&r.label_id, 4, 1, f);
    std::fwrite(&r.cell_count, 4, 1, f);
    std::fwrite(&r.cx, 8, 1, f); std::fwrite(&r.cy, 8, 1, f); std::fwrite(&r.area_m2, 8, 1, f);
  }
  int ne = (int)res.edges.size(); std::fwrite(&ne, 4, 1, f);
  for (auto& e : res.edges) { int v[2] = {e.first, e.second}; std::fwrite(v, 4, 2, f); }
  std::fwrite(res.label_img.data(), 4, N, f);
  std::fwrite(cost.data(), 1, N, f);
  std::fclose(f);
  return 0;
}
