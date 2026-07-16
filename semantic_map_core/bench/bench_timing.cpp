// bench_timing.cpp — 在线层 C++ 计时(features / merge_observation / consolidate_fragments),
// 与 time_online.py 同数据(merge_in.bin)、同迭代次数、同 LCG 合成 cells, 报 per-call 时间。
#include "semantic_map_core/features.hpp"
#include "semantic_map_core/graph.hpp"
#include "semantic_map_core/objects.hpp"
#include "semantic_map_core/scene_graph.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace smc;
using clk = std::chrono::steady_clock;
static double ms_since(clk::time_point t) {
  return std::chrono::duration<double, std::milli>(clk::now() - t).count();
}

int main() {
  const std::string O = "/mnt/hgfs/Shared/claude_jobs/";
  // 读 merge_in.bin(图+物体+feat)
  FILE* f = std::fopen((O + "merge_in.bin").c_str(), "rb"); size_t rd;
  int nn; rd = std::fread(&nn, 4, 1, f);
  SkelGraph g; g.nodes.resize(nn);
  std::vector<int> room(nn);
  for (int i = 0; i < nn; ++i) {
    int v[4]; rd = std::fread(v, 4, 4, f);
    g.nodes[i].i = v[0]; g.nodes[i].j = v[1]; g.nodes[i].k = v[2]; g.nodes[i].degree = v[3];
    rd = std::fread(&g.nodes[i].clearance_m, 4, 1, f);
    rd = std::fread(&room[i], 4, 1, f);
  }
  int no; rd = std::fread(&no, 4, 1, f);
  std::vector<ObjectNode> objs(no);
  for (int j = 0; j < no; ++j) {
    int v[11]; rd = std::fread(v, 4, 11, f);
    objs[j].label = v[0]; objs[j].ci = v[1]; objs[j].cj = v[2]; objs[j].ck = v[3];
    objs[j].voxel_count = v[4];
    for (int a = 0; a < 3; ++a) { objs[j].bmin[a] = v[5 + a]; objs[j].bmax[a] = v[8 + a]; }
    rd = std::fread(objs[j].feat.data(), 8, 3, f);
  }
  std::fclose(f); (void)rd;
  const float vs = 0.1f; const long vmin[3] = {0, 0, 0};

  volatile double sink = 0;

  // ---- features: 每物体合成 M=voxel_count 个点(LCG), 计时 shape 全体 ----
  std::vector<std::vector<std::array<int, 3>>> cells(no);
  for (int j = 0; j < no; ++j) {
    unsigned x = (unsigned)(j + 1);
    int M = objs[j].voxel_count;
    cells[j].resize(M);
    for (int p = 0; p < M; ++p) {
      x = x * 1103515245u + 12345u; int a = (x >> 16) % 100;
      x = x * 1103515245u + 12345u; int b = (x >> 16) % 100;
      x = x * 1103515245u + 12345u; int c = (x >> 16) % 100;
      cells[j][p] = {a, b, c};
    }
  }
  const int K1 = 2000;
  auto t = clk::now();
  for (int k = 0; k < K1; ++k)
    for (int j = 0; j < no; ++j) { auto s = shape_feature(cells[j], vs); sink += s[0]; }
  double feat_ms = ms_since(t) / K1;  // 每次 = 全体物体

  // ---- merge_observation ----
  link_to_places(objs, g, vmin, vs);
  SceneGraph existing = build_scene_graph(g, room, objs, vmin, vs);
  if (existing.get("object:0")) existing.get("object:0")->misses = 3;
  std::vector<ObjectNode> fresh;
  for (int j = 0; j < no; ++j) { if (j == 0 || j == 1) continue; ObjectNode o = objs[j];
    if (j == 2) { o.ci += 8; o.bmin[0] += 8; o.bmax[0] += 8; } fresh.push_back(o); }
  link_to_places(fresh, g, vmin, vs);
  const int K2 = 300;
  t = clk::now();
  for (int k = 0; k < K2; ++k) {
    MergeStats st; int c = 0;
    SceneGraph m = merge_observation(existing, g, room, fresh, vmin, vs, st, c, "apartment", 0.20f, 0.5f, 3);
    sink += (double)m.order.size();
  }
  double merge_ms = ms_since(t) / K2;

  // ---- consolidate_fragments(拷贝在计时外)----
  SceneGraph base = build_scene_graph(g, room, objs, vmin, vs);
  for (auto* nd : base.nodes_by_layer("object")) nd->seen_count = 2;
  const int K3 = 3000;
  double copy_ms, both_ms;
  { t = clk::now(); for (int k = 0; k < K3; ++k) { SceneGraph c = base; sink += (double)c.order.size(); } copy_ms = ms_since(t); }
  { t = clk::now(); for (int k = 0; k < K3; ++k) { SceneGraph c = base; sink += consolidate_fragments(c, 1.0, 2); } both_ms = ms_since(t); }
  double consol_ms = (both_ms - copy_ms) / K3;

  std::printf("CPP_FEATURES_MS %.6f\n", feat_ms);
  std::printf("CPP_MERGE_MS %.6f\n", merge_ms);
  std::printf("CPP_CONSOL_MS %.6f\n", consol_ms);
  std::printf("(sink=%.1f objs=%d)\n", (double)sink, no);
  return 0;
}
