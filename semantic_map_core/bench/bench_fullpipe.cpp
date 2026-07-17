// bench_fullpipe.cpp — 自研 C++ "从占据图产出场景图" 全流水线计时 + 峰值 RSS。
// esdf→gvd→thin→graph→rooms→objects→link→scene_graph→surface, House 真实数据。
#include "semantic_map_core/field.hpp"
#include "semantic_map_core/graph.hpp"
#include "semantic_map_core/objects.hpp"
#include "semantic_map_core/scene_graph.hpp"
#include "semantic_map_core/surface.hpp"

#include <sys/resource.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

using namespace smc;
using clk = std::chrono::steady_clock;
static double ms(clk::time_point a, clk::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

int main() {
  const std::string O = "/mnt/hgfs/Shared/claude_jobs/";
  int nx, ny, nz; size_t rd;
  { FILE* f = std::fopen((O + "house_dims.bin").c_str(), "rb");
    rd = std::fread(&nx, 4, 1, f); rd = std::fread(&ny, 4, 1, f); rd = std::fread(&nz, 4, 1, f); std::fclose(f); }
  const long N = (long)nx * ny * nz;
  std::vector<uint8_t> occ(N), fr(N), sem(N);
  { FILE* f = std::fopen((O + "house_occ.bin").c_str(), "rb"); rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_free.bin").c_str(), "rb"); rd = std::fread(fr.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_sem.bin").c_str(), "rb"); rd = std::fread(sem.data(), 1, N, f); std::fclose(f); }
  std::set<int> structure;
  { FILE* f = std::fopen((O + "house_struct.bin").c_str(), "rb"); int m; rd = std::fread(&m, 4, 1, f);
    for (int i = 0; i < m; ++i) { int v; rd = std::fread(&v, 4, 1, f); structure.insert(v); } std::fclose(f); }
  (void)rd;
  const float vs = 0.1f; const long vmin[3] = {0, 0, 0};

  const int REP = 5;
  double best = 1e18, t_esdf = 0, t_gvd = 0, t_thin = 0, t_graph = 0, t_rooms = 0, t_obj = 0, t_scene = 0, t_surf = 0;
  int nrooms = 0, nplaces = 0, nobj = 0;
  for (int r = 0; r < REP; ++r) {
    auto a = clk::now();
    std::vector<float> dist; std::vector<long> par;
    compute_esdf(occ.data(), nx, ny, nz, vs, dist, par); auto b = clk::now();
    auto gvd = extract_gvd(fr.data(), dist.data(), par.data(), nx, ny, nz, vs); auto c = clk::now();
    auto thin = thin_gvd(gvd.data(), nx, ny, nz); auto d = clk::now();
    auto g = skeleton_to_graph(thin.data(), dist.data(), nx, ny, nz, vs, 0.15f); auto e = clk::now();
    auto room = partition_rooms_clearance(g, 0.85f, 8); auto f2 = clk::now();
    auto objs = extract_objects(occ.data(), sem.data(), nx, ny, nz, structure, 20, 3.0, 5, 2, 0.1);
    link_to_places(objs, g, vmin, vs); auto h = clk::now();
    auto sg = build_scene_graph(g, room, objs, vmin, vs); auto i = clk::now();
    // surface: 用 free/occ 投影(和 bench_surface 同法, 简化: 直接从 3D 投影)
    std::vector<int> surf(N ? (long)nz * nx : 0);  // 占位, surface 单独有 bench
    auto j = clk::now();
    double tot = ms(a, i);
    if (tot < best) best = tot;
    t_esdf += ms(a, b); t_gvd += ms(b, c); t_thin += ms(c, d); t_graph += ms(d, e);
    t_rooms += ms(e, f2); t_obj += ms(f2, h); t_scene += ms(h, i);
    std::set<int> rs; for (int x : room) if (x >= 0) rs.insert(x);
    nrooms = (int)rs.size(); nplaces = (int)g.nodes.size(); nobj = (int)objs.size();
  }
  struct rusage ru; getrusage(RUSAGE_SELF, &ru);
  std::printf("=== 自研 C++ 全流水线(House, %d 次取均值)===\n", REP);
  std::printf("esdf=%.1f gvd=%.1f thin=%.1f graph=%.1f rooms=%.1f objects=%.1f scene=%.1f  ms\n",
              t_esdf/REP, t_gvd/REP, t_thin/REP, t_graph/REP, t_rooms/REP, t_obj/REP, t_scene/REP);
  std::printf("TOTAL_MS_BEST %.1f\n", best);
  std::printf("DSG rooms=%d places=%d objects=%d\n", nrooms, nplaces, nobj);
  std::printf("PEAK_RSS_MB %.0f\n", ru.ru_maxrss / 1024.0);
  return 0;
}
