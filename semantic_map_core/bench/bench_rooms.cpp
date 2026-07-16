// bench_rooms.cpp — C++ 建图+分房, dump 图(clearance+edges)+ room_of 供 Python 同图对拍。
#include "semantic_map_core/field.hpp"
#include "semantic_map_core/graph.hpp"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace smc;
int main() {
  const std::string O = "/mnt/hgfs/Shared/claude_jobs/";
  int nx, ny, nz; size_t rd;
  { FILE* f = std::fopen((O + "house_dims.bin").c_str(), "rb");
    rd = std::fread(&nx, 4, 1, f); rd = std::fread(&ny, 4, 1, f); rd = std::fread(&nz, 4, 1, f); std::fclose(f); }
  const long N = (long)nx * ny * nz;
  std::vector<uint8_t> occ(N), free(N);
  { FILE* f = std::fopen((O + "house_occ.bin").c_str(), "rb"); rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_free.bin").c_str(), "rb"); rd = std::fread(free.data(), 1, N, f); std::fclose(f); }
  (void)rd;
  const float vs = 0.1f;
  std::vector<float> dist; std::vector<long> parent;
  compute_esdf(occ.data(), nx, ny, nz, vs, dist, parent);
  auto gvd = thin_gvd(extract_gvd(free.data(), dist.data(), parent.data(), nx, ny, nz, vs).data(), nx, ny, nz);
  auto g = skeleton_to_graph(gvd.data(), dist.data(), nx, ny, nz, vs, 0.15f);
  auto room = partition_rooms_clearance(g, 0.85f, 8);

  int nrooms = 0; { std::vector<char> seen(4096, 0); for (int r : room) { if (r + 1 > nrooms) nrooms = r + 1; } }
  std::printf("C++  nodes=%zu edges=%zu  rooms=%d\n", g.nodes.size(), g.edges.size(), nrooms);

  // dump 图 + room_of(供 Python 同图重建)
  FILE* f = std::fopen((O + "rooms_cpp.bin").c_str(), "wb");
  int nn = (int)g.nodes.size(); std::fwrite(&nn, 4, 1, f);
  for (auto& nd : g.nodes) std::fwrite(&nd.clearance_m, 4, 1, f);
  int ne = (int)g.edges.size(); std::fwrite(&ne, 4, 1, f);
  for (auto& e : g.edges) { int v[2] = {e.a, e.b}; std::fwrite(v, 4, 2, f); }
  std::fwrite(room.data(), 4, nn, f);
  std::fclose(f);
  return 0;
}
