// bench_scene.cpp — C++ 全链路(esdf→gvd→thin→graph→rooms→objects→scene_graph),
// vmin={0,0,0}, dump scene 节点(id|layer|parent|x|y|z|class|voxel)供 Python 对拍。
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
  const float vs = 0.1f;

  std::vector<float> dist; std::vector<long> parent;
  compute_esdf(occ.data(), nx, ny, nz, vs, dist, parent);
  auto gvd = thin_gvd(extract_gvd(free.data(), dist.data(), parent.data(), nx, ny, nz, vs).data(), nx, ny, nz);
  auto g = skeleton_to_graph(gvd.data(), dist.data(), nx, ny, nz, vs, 0.15f);
  auto room = partition_rooms_clearance(g, 0.85f, 8);
  auto objs = extract_objects(occ.data(), sem.data(), nx, ny, nz, structure, 20, 3.0, 5, 2);

  const long vmin[3] = {0, 0, 0};
  link_to_places(objs, g, vmin, vs);
  auto sg = build_scene_graph(g, room, objs, vmin, vs, "apartment", 0.20f);

  // dump 图+物体输入(供 Python 用真实类重建, 隔离对拍 build_scene_graph)
  { FILE* fi = std::fopen((O + "scene_in.bin").c_str(), "wb");
    int nn = (int)g.nodes.size(); std::fwrite(&nn, 4, 1, fi);
    for (size_t i = 0; i < g.nodes.size(); ++i) {
      int v[4] = {g.nodes[i].i, g.nodes[i].j, g.nodes[i].k, g.nodes[i].degree};
      std::fwrite(v, 4, 4, fi);
      std::fwrite(&g.nodes[i].clearance_m, 4, 1, fi);
      int r = room[i]; std::fwrite(&r, 4, 1, fi);
    }
    int no = (int)objs.size(); std::fwrite(&no, 4, 1, fi);
    for (auto& o : objs) {
      int v[11] = {o.label, o.ci, o.cj, o.ck, o.voxel_count,
                   o.bmin[0], o.bmin[1], o.bmin[2], o.bmax[0], o.bmax[1], o.bmax[2]};
      std::fwrite(v, 4, 11, fi);
    }
    std::fclose(fi); }

  int nb = 0, nr = 0, np = 0, no = 0;
  FILE* f = std::fopen((O + "scene_cpp.txt").c_str(), "wb");
  for (auto& id : sg.order) {
    SceneNode& n = sg.nodes[id];
    if (n.layer == "building") ++nb; else if (n.layer == "room") ++nr;
    else if (n.layer == "place") ++np; else if (n.layer == "object") ++no;
    std::fprintf(f, "%s|%s|%s|%.6f|%.6f|%.6f|%d|%d\n", n.id.c_str(), n.layer.c_str(),
                 n.parent.c_str(), n.pos[0], n.pos[1], n.pos[2], n.obj_class, n.voxel_count);
  }
  std::fclose(f);
  std::printf("C++  scene nodes=%zu  building=%d rooms=%d places=%d objects=%d\n",
              sg.order.size(), nb, nr, np, no);
  return 0;
}
