// bench_online.cpp — 在线时序层对拍: merge_observation + consolidate_fragments。
// dump merge_in.bin(图+物体+feat, 供 Python 重建同输入)+ merge_cpp.txt + consol_cpp.txt。
#include "semantic_map_core/field.hpp"
#include "semantic_map_core/graph.hpp"
#include "semantic_map_core/objects.hpp"
#include "semantic_map_core/scene_graph.hpp"

#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

using namespace smc;

static void dump_nodes(const std::string& path, SceneGraph& sg) {
  FILE* f = std::fopen(path.c_str(), "wb");
  for (auto& id : sg.order) {
    SceneNode& n = sg.nodes[id];
    std::fprintf(f, "%s|%s|%s|%.6f|%.6f|%.6f|%d|%d|%d|%d\n", n.id.c_str(), n.layer.c_str(),
                 n.parent.c_str(), n.pos[0], n.pos[1], n.pos[2], n.obj_class, n.voxel_count,
                 n.misses, n.seen_count);
  }
  std::fclose(f);
}

int main() {
  const std::string O = "/mnt/hgfs/Shared/claude_jobs/";
  int nx, ny, nz; size_t rd;
  { FILE* f = std::fopen((O + "house_dims.bin").c_str(), "rb");
    rd = std::fread(&nx, 4, 1, f); rd = std::fread(&ny, 4, 1, f); rd = std::fread(&nz, 4, 1, f); std::fclose(f); }
  const long N = (long)nx * ny * nz;
  std::vector<uint8_t> occ(N), free(N), sem(N);
  { FILE* f = std::fopen((O + "house_occ.bin").c_str(), "rb"); rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_free.bin").c_str(), "rb"); rd = std::fread(free.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_sem.bin").c_str(), "rb"); rd = std::fread(sem.data(), 1, N, f); std::fclose(f); }
  std::set<int> structure;
  { FILE* f = std::fopen((O + "house_struct.bin").c_str(), "rb"); int m; rd = std::fread(&m, 4, 1, f);
    for (int i = 0; i < m; ++i) { int v; rd = std::fread(&v, 4, 1, f); structure.insert(v); } std::fclose(f); }
  (void)rd;
  const float vs = 0.1f; const long vmin[3] = {0, 0, 0};

  std::vector<float> dist; std::vector<long> parent;
  compute_esdf(occ.data(), nx, ny, nz, vs, dist, parent);
  auto gvd = thin_gvd(extract_gvd(free.data(), dist.data(), parent.data(), nx, ny, nz, vs).data(), nx, ny, nz);
  auto g = skeleton_to_graph(gvd.data(), dist.data(), nx, ny, nz, vs, 0.15f);
  auto room = partition_rooms_clearance(g, 0.85f, 8);
  auto objs = extract_objects(occ.data(), sem.data(), nx, ny, nz, structure, 20, 3.0, 5, 2, 0.1);
  link_to_places(objs, g, vmin, vs);  // 镜像 pipeline: build 前先 link

  // dump 输入(图+物体+feat)
  { FILE* fi = std::fopen((O + "merge_in.bin").c_str(), "wb");
    int nn = (int)g.nodes.size(); std::fwrite(&nn, 4, 1, fi);
    for (size_t i = 0; i < g.nodes.size(); ++i) {
      int v[4] = {g.nodes[i].i, g.nodes[i].j, g.nodes[i].k, g.nodes[i].degree};
      std::fwrite(v, 4, 4, fi); std::fwrite(&g.nodes[i].clearance_m, 4, 1, fi);
      int r = room[i]; std::fwrite(&r, 4, 1, fi);
    }
    int no = (int)objs.size(); std::fwrite(&no, 4, 1, fi);
    for (auto& o : objs) {
      int v[11] = {o.label, o.ci, o.cj, o.ck, o.voxel_count,
                   o.bmin[0], o.bmin[1], o.bmin[2], o.bmax[0], o.bmax[1], o.bmax[2]};
      std::fwrite(v, 4, 11, fi); std::fwrite(o.feat.data(), 8, 3, fi);
    }
    std::fclose(fi); }

  // ---- MERGE 场景: existing=House; obj:0 预置 misses=3; fresh=去掉 orig{0,1}, orig2 平移+8x ----
  SceneGraph existing = build_scene_graph(g, room, objs, vmin, vs, "apartment", 0.20f);
  if (existing.get("object:0")) existing.get("object:0")->misses = 3;
  std::vector<ObjectNode> fresh;
  for (int j = 0; j < (int)objs.size(); ++j) {
    if (j == 0 || j == 1) continue;
    ObjectNode o = objs[j];
    if (j == 2) { o.ci += 8; o.bmin[0] += 8; o.bmax[0] += 8; }
    fresh.push_back(o);
  }
  link_to_places(fresh, g, vmin, vs);  // fresh 也先 link(镜像调用方)
  MergeStats stats; int counter = 0;
  SceneGraph merged = merge_observation(existing, g, room, fresh, vmin, vs, stats, counter,
                                        "apartment", 0.20f, 0.5f, 3);
  { FILE* f = std::fopen((O + "merge_stats_cpp.txt").c_str(), "wb");
    std::fprintf(f, "matched=%d added=%d removed=%d carried=%d objects_total=%d\n",
                 stats.matched, stats.added, stats.removed, stats.carried, stats.objects_total);
    std::fclose(f); }
  dump_nodes(O + "merge_cpp.txt", merged);
  std::printf("C++ merge: matched=%d added=%d removed=%d carried=%d total=%d\n",
              stats.matched, stats.added, stats.removed, stats.carried, stats.objects_total);

  // ---- CONSOLIDATE 场景: 全部 seen_count=2, gap=1.0m ----
  SceneGraph sg2 = build_scene_graph(g, room, objs, vmin, vs, "apartment", 0.20f);
  for (auto* n : sg2.nodes_by_layer("object")) n->seen_count = 2;
  int merges = consolidate_fragments(sg2, 1.0, 2);
  { FILE* f = std::fopen((O + "consol_stats_cpp.txt").c_str(), "wb");
    std::fprintf(f, "merges=%d objects_total=%d\n", merges, (int)sg2.nodes_by_layer("object").size());
    std::fclose(f); }
  dump_nodes(O + "consol_cpp.txt", sg2);
  std::printf("C++ consolidate: merges=%d objects_total=%d\n", merges,
              (int)sg2.nodes_by_layer("object").size());
  return 0;
}
