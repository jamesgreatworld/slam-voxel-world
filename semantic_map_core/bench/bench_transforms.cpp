// bench_transforms.cpp — 图清理变换对拍: House 真实骨架图上跑
// prune_spurs -> merge_close -> drop_small_components -> rooms -> merge_nested_rooms,
// 每步 dump 供 Python 用真实 graph.py/gvd_live.py 同链路对拍。vmin=0, vs=0.1(double)。
#include "semantic_map_core/field.hpp"
#include "semantic_map_core/graph.hpp"

#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

using namespace smc;

static void dump_graph(const std::string& path, const SkelGraph& g,
                       const std::vector<int>* room = nullptr) {
  FILE* f = std::fopen(path.c_str(), "wb");
  int nn = (int)g.nodes.size(); std::fwrite(&nn, 4, 1, f);
  for (auto& n : g.nodes) {
    int v[5] = {n.i, n.j, n.k, n.degree, n.type};
    std::fwrite(v, 4, 5, f);
    std::fwrite(&n.clearance_m, 4, 1, f);
  }
  int ne = (int)g.edges.size(); std::fwrite(&ne, 4, 1, f);
  for (auto& e : g.edges) { int v[2] = {e.a, e.b}; std::fwrite(v, 4, 2, f); std::fwrite(&e.length_m, 4, 1, f); }
  if (room) std::fwrite(room->data(), 4, nn, f);
  std::fclose(f);
}

int main() {
  const std::string O = "/mnt/hgfs/Shared/claude_jobs/";
  int nx, ny, nz; size_t rd;
  { FILE* f = std::fopen((O + "house_dims.bin").c_str(), "rb");
    rd = std::fread(&nx, 4, 1, f); rd = std::fread(&ny, 4, 1, f); rd = std::fread(&nz, 4, 1, f); std::fclose(f); }
  const long N = (long)nx * ny * nz;
  std::vector<uint8_t> occ(N), fr(N);
  { FILE* f = std::fopen((O + "house_occ.bin").c_str(), "rb"); rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_free.bin").c_str(), "rb"); rd = std::fread(fr.data(), 1, N, f); std::fclose(f); }
  (void)rd;
  const float vs = 0.1f; const long vmin[3] = {0, 0, 0};

  std::vector<float> dist; std::vector<long> parent;
  compute_esdf(occ.data(), nx, ny, nz, vs, dist, parent);
  auto gvd = thin_gvd(extract_gvd(fr.data(), dist.data(), parent.data(), nx, ny, nz, vs, 0.20f, 0.40f).data(),
                      nx, ny, nz);
  auto g0 = skeleton_to_graph(gvd.data(), dist.data(), nx, ny, nz, vs, 0.15f);
  dump_graph(O + "tr_g0.bin", g0);
  auto g1 = prune_spurs(g0, 0.3);
  dump_graph(O + "tr_g1.bin", g1);
  auto g2 = merge_close(g1, 0.2, vmin, 0.1);
  dump_graph(O + "tr_g2.bin", g2);
  auto g3 = drop_small_components(g2, 5);
  auto room = partition_rooms_clearance(g3, 0.85f, 8);
  dump_graph(O + "tr_g3.bin", g3, &room);
  merge_nested_rooms(g3, room, 0.1, 0.3);
  dump_graph(O + "tr_g4.bin", g3, &room);
  int nrooms = 0; { std::set<int> s(room.begin(), room.end()); nrooms = (int)s.size(); }
  std::printf("C++ transforms: g0=%zu g1=%zu g2=%zu g3=%zu nodes, rooms(final)=%d\n",
              g0.nodes.size(), g1.nodes.size(), g2.nodes.size(), g3.nodes.size(), nrooms);
  return 0;
}
