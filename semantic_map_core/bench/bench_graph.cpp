// bench_graph.cpp — 真实 House: 压缩抽图 vs 当前 thin+skeleton 的节点数/连通/耗时。
#include "semantic_map_core/field.hpp"
#include "semantic_map_core/graph.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

using namespace smc;
static double ms(std::chrono::high_resolution_clock::time_point a,
                 std::chrono::high_resolution_clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

// 连通分量数(并查集)
static int components(const PlacesGraph& g) {
  std::vector<int> p(g.nodes.size());
  for (size_t i = 0; i < p.size(); ++i) p[i] = (int)i;
  std::function<int(int)> f = [&](int x) { while (p[x] != x) { p[x] = p[p[x]]; x = p[x]; } return x; };
  for (auto& e : g.edges) p[f(e.a)] = f(e.b);
  int c = 0;
  for (size_t i = 0; i < p.size(); ++i) if (f((int)i) == (int)i) ++c;
  return c;
}

int main() {
  const std::string O = "/mnt/hgfs/Shared/claude_jobs/";
  int nx, ny, nz; size_t rd;
  { FILE* f = std::fopen((O + "house_dims.bin").c_str(), "rb");
    rd = std::fread(&nx, 4, 1, f); rd = std::fread(&ny, 4, 1, f); rd = std::fread(&nz, 4, 1, f);
    std::fclose(f); }
  const long N = (long)nx * ny * nz;
  std::vector<uint8_t> occ(N), free(N);
  { FILE* f = std::fopen((O + "house_occ.bin").c_str(), "rb"); rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_free.bin").c_str(), "rb"); rd = std::fread(free.data(), 1, N, f); std::fclose(f); }
  (void)rd;
  const float vs = 0.1f;
  const long vmin[3] = {0, 0, 0};  // 相对裁剪网格; 绝对 vmin 不影响节点数/连通

  std::vector<float> dist; std::vector<long> parent;
  compute_esdf(occ.data(), nx, ny, nz, vs, dist, parent);
  auto gvd = extract_gvd(free.data(), dist.data(), parent.data(), nx, ny, nz, vs);
  long gc = 0; for (auto v : gvd) gc += v;

  auto a = std::chrono::high_resolution_clock::now();
  auto g = compress_gvd_graph(gvd.data(), dist.data(), nx, ny, nz, vmin, vs, 0.5f, 0.3f, 0.3f);
  auto b = std::chrono::high_resolution_clock::now();

  std::printf("C++  GVD体素=%ld\n", gc);
  std::printf("C++  压缩抽图: nodes=%zu edges=%zu  连通分量=%d  耗时=%.1fms\n",
              g.nodes.size(), g.edges.size(), components(g), ms(a, b));
  int deg1 = 0, deg2 = 0, deg3 = 0;
  for (auto& n : g.nodes) { if (n.degree == 1) deg1++; else if (n.degree == 2) deg2++; else if (n.degree >= 3) deg3++; }
  std::printf("C++  度数: endpoint(deg1)=%d passthrough(deg2)=%d junction(deg>=3)=%d\n", deg1, deg2, deg3);

  // 对照: 先 thin(细化成 1 体素中轴)再压缩 -> 判断"厚GVD" vs "压缩方法"
  auto thin = thin_gvd(gvd.data(), nx, ny, nz);
  long tc = 0; for (auto v : thin) tc += v;
  auto g2 = compress_gvd_graph(thin.data(), dist.data(), nx, ny, nz, vmin, vs, 0.5f, 0.3f, 0.3f);
  std::printf("C++  [thin后压缩] thin体素=%ld  nodes=%zu edges=%zu 连通分量=%d\n",
              tc, g2.nodes.size(), g2.edges.size(), components(g2));
  return 0;
}
