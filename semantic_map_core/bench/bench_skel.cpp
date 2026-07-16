// bench_skel.cpp — 真实 House: C++ skeleton_to_graph, 写节点/边供 Python 逐格对拍。
#include "semantic_map_core/field.hpp"
#include "semantic_map_core/graph.hpp"

#include <chrono>
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
  auto gvd0 = extract_gvd(free.data(), dist.data(), parent.data(), nx, ny, nz, vs);
  auto gvd = thin_gvd(gvd0.data(), nx, ny, nz);

  auto a = std::chrono::high_resolution_clock::now();
  auto g = skeleton_to_graph(gvd.data(), dist.data(), nx, ny, nz, vs, 0.15f);
  auto b = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(b - a).count();
  std::printf("C++  skeleton_to_graph: nodes=%zu edges=%zu  %.1fms\n",
              g.nodes.size(), g.edges.size(), ms);

  // 节点: i,j,k,degree,type,clearance ; 边: a,b,length
  FILE* fn = std::fopen((O + "skel_nodes_cpp.bin").c_str(), "wb");
  int nn = (int)g.nodes.size(); std::fwrite(&nn, 4, 1, fn);
  for (auto& n : g.nodes) {
    int v[5] = {n.i, n.j, n.k, n.degree, n.type}; std::fwrite(v, 4, 5, fn);
    std::fwrite(&n.clearance_m, 4, 1, fn);
  }
  std::fclose(fn);
  FILE* fe = std::fopen((O + "skel_edges_cpp.bin").c_str(), "wb");
  int ne = (int)g.edges.size(); std::fwrite(&ne, 4, 1, fe);
  for (auto& e : g.edges) { int v[2] = {e.a, e.b}; std::fwrite(v, 4, 2, fe); std::fwrite(&e.length_m, 4, 1, fe); }
  std::fclose(fe);
  return 0;
}
