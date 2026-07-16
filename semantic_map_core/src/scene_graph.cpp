// scene_graph.cpp — 见 scene_graph.hpp。移植自 scene_graph.py。
#include "semantic_map_core/scene_graph.hpp"

#include <cmath>
#include <set>

namespace smc {

std::vector<SgNode> build_scene_graph(const SkelGraph& g, const std::vector<int>& room_of,
                                      const std::vector<ObjectNode>& objects,
                                      const long vmin[3], float vs,
                                      const std::string& world_id, float support_gap_m) {
  const int P = (int)g.nodes.size();
  auto ppos = [&](int i, double& x, double& y, double& z) {
    x = (g.nodes[i].i + vmin[0]) * (double)vs;  // 与 Python positions_m 一致: (idx+vmin)*vs, 无 +0.5
    y = (g.nodes[i].j + vmin[1]) * (double)vs;
    z = (g.nodes[i].k + vmin[2]) * (double)vs;
  };
  std::vector<SgNode> out;

  // 1. building
  out.push_back({"building:0", "building", "", 0, 0, 0, -1, 0});

  // 2. rooms: 房间号升序; pos = 成员 place 质心
  std::set<int> room_ids;
  for (int i = 0; i < P; ++i) if (room_of[i] >= 0) room_ids.insert(room_of[i]);
  for (int r : room_ids) {
    double sx = 0, sy = 0, sz = 0; int cnt = 0;
    for (int i = 0; i < P; ++i)
      if (room_of[i] == r) { double x, y, z; ppos(i, x, y, z); sx += x; sy += y; sz += z; ++cnt; }
    double cx = cnt ? sx / cnt : 0, cy = cnt ? sy / cnt : 0, cz = cnt ? sz / cnt : 0;
    out.push_back({"room:" + std::to_string(r), "room", "building:0", cx, cy, cz, -1, 0});
  }

  // 3. places
  for (int i = 0; i < P; ++i) {
    double x, y, z; ppos(i, x, y, z);
    std::string parent = "building:0";
    if (room_of[i] >= 0) parent = "room:" + std::to_string(room_of[i]);
    out.push_back({"place:" + std::to_string(i), "place", parent, x, y, z, -1, 0});
  }

  // link_to_places: 每物体最近 place(质心米距离最小, tie 取首个)
  const int O = (int)objects.size();
  std::vector<int> place_id(O, -1);
  std::vector<std::array<double, 3>> ppos_all(P);
  for (int i = 0; i < P; ++i) { double x, y, z; ppos(i, x, y, z); ppos_all[i] = {x, y, z}; }
  for (int j = 0; j < O; ++j) {
    double ox = (objects[j].ci + vmin[0]) * (double)vs, oy = (objects[j].cj + vmin[1]) * (double)vs,
           oz = (objects[j].ck + vmin[2]) * (double)vs;
    double best = 1e30; int bi = -1;
    for (int i = 0; i < P; ++i) {
      double d2 = (ppos_all[i][0] - ox) * (ppos_all[i][0] - ox) +
                  (ppos_all[i][1] - oy) * (ppos_all[i][1] - oy) +
                  (ppos_all[i][2] - oz) * (ppos_all[i][2] - oz);
      if (d2 < best) { best = d2; bi = i; }
    }
    place_id[j] = bi;
  }

  // 4. objects: 先全加(记 bbox_m), 再定父(支撑 object 或 room-via-place)
  int obj_base = (int)out.size();
  std::vector<std::array<double, 6>> bbm(O);  // bmin xyz, bmax xyz (米)
  for (int j = 0; j < O; ++j) {
    double ox = (objects[j].ci + vmin[0]) * (double)vs, oy = (objects[j].cj + vmin[1]) * (double)vs,
           oz = (objects[j].ck + vmin[2]) * (double)vs;
    bbm[j] = {(objects[j].bmin[0] + vmin[0]) * (double)vs, (objects[j].bmin[1] + vmin[1]) * (double)vs,
              (objects[j].bmin[2] + vmin[2]) * (double)vs, (objects[j].bmax[0] + vmin[0]) * (double)vs,
              (objects[j].bmax[1] + vmin[1]) * (double)vs, (objects[j].bmax[2] + vmin[2]) * (double)vs};
    out.push_back({"object:" + std::to_string(j), "object", "", ox, oy, oz,
                   objects[j].label, objects[j].voxel_count});
  }
  for (int j = 0; j < O; ++j) {
    double o_bottom_y = bbm[j][1];
    int best_support = -1; double best_top = -1e18;
    for (int k = 0; k < O; ++k) {
      if (k == j) continue;
      double k_top = bbm[k][4];
      if (!(k_top <= o_bottom_y && o_bottom_y <= k_top + support_gap_m)) continue;
      // 水平 x/z 重叠
      if (bbm[j][3] < bbm[k][0] || bbm[j][0] > bbm[k][3]) continue;
      if (bbm[j][5] < bbm[k][2] || bbm[j][2] > bbm[k][5]) continue;
      if (k_top > best_top) { best_top = k_top; best_support = k; }
    }
    std::string parent;
    if (best_support >= 0) parent = "object:" + std::to_string(best_support);
    else {
      parent = "building:0";
      int pid = place_id[j];
      if (pid >= 0 && pid < P && room_of[pid] >= 0) parent = "room:" + std::to_string(room_of[pid]);
    }
    out[obj_base + j].parent = parent;
  }
  return out;
}

}  // namespace smc
