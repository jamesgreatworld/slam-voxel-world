// scene_graph.cpp — 见 scene_graph.hpp。移植自 scene_graph.py。
#include "semantic_map_core/scene_graph.hpp"

#include "semantic_map_core/features.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>

namespace smc {

// ===================== SceneGraph 方法(复刻 Python dict 语义)=====================
void SceneGraph::add_node(const SceneNode& n, const std::string& parent_id, bool has_parent) {
  if (nodes.count(n.id)) return;  // Python 抛错; 我们的构建保证不重复, 容错跳过
  nodes[n.id] = n;
  order.push_back(n.id);
  if (n.layer == "building" && root_id.empty()) root_id = n.id;
  if (has_parent) set_parent(n.id, parent_id);
}
SceneNode* SceneGraph::get(const std::string& id) {
  auto it = nodes.find(id); return it == nodes.end() ? nullptr : &it->second;
}
void SceneGraph::set_parent(const std::string& id, const std::string& np) {
  SceneNode& n = nodes[id];
  if (!n.parent.empty() && nodes.count(n.parent)) {
    auto& ch = nodes[n.parent].children;
    ch.erase(std::remove(ch.begin(), ch.end(), id), ch.end());
  }
  n.parent = np;
  if (!np.empty()) {
    auto& pc = nodes[np].children;
    if (std::find(pc.begin(), pc.end(), id) == pc.end()) pc.push_back(id);
  }
}
void SceneGraph::remove_node(const std::string& id, bool reparent_children) {
  auto it = nodes.find(id); if (it == nodes.end()) return;
  SceneNode n = it->second;  // 拷贝
  nodes.erase(it);
  order.erase(std::remove(order.begin(), order.end(), id), order.end());
  if (!n.parent.empty() && nodes.count(n.parent)) {
    auto& pc = nodes[n.parent].children;
    pc.erase(std::remove(pc.begin(), pc.end(), id), pc.end());
  }
  for (const std::string& c : n.children) {
    if (!nodes.count(c)) continue;
    if (reparent_children && !n.parent.empty()) set_parent(c, n.parent);
    else nodes[c].parent.clear();
  }
}
std::vector<SceneNode*> SceneGraph::nodes_by_layer(const std::string& layer) {
  std::vector<SceneNode*> out;
  for (const std::string& id : order) { auto& nd = nodes[id]; if (nd.layer == layer) out.push_back(&nd); }
  return out;
}
std::vector<SceneNode*> SceneGraph::ancestors(const std::string& id) {
  std::vector<SceneNode*> out;
  std::string p = nodes.count(id) ? nodes[id].parent : std::string();
  while (!p.empty() && nodes.count(p)) { out.push_back(&nodes[p]); p = nodes[p].parent; }
  return out;
}

// ===================== build_scene_graph =====================
SceneGraph build_scene_graph(const SkelGraph& g, const std::vector<int>& room_of,
                             const std::vector<ObjectNode>& objects, const long vmin[3],
                             float vs, const std::string& world_id, float support_gap_m,
                             const std::vector<std::string>& obj_ids) {
  const int P = (int)g.nodes.size();
  auto ppos = [&](int i, double& x, double& y, double& z) {
    x = (g.nodes[i].i + vmin[0]) * (double)vs;
    y = (g.nodes[i].j + vmin[1]) * (double)vs;
    z = (g.nodes[i].k + vmin[2]) * (double)vs;
  };
  SceneGraph sg;

  { SceneNode b; b.id = "building:0"; b.layer = "building"; sg.add_node(b); }

  std::set<int> room_ids;
  for (int i = 0; i < P; ++i) if (room_of[i] >= 0) room_ids.insert(room_of[i]);
  for (int r : room_ids) {
    double sx = 0, sy = 0, sz = 0; int cnt = 0;
    for (int i = 0; i < P; ++i)
      if (room_of[i] == r) { double x, y, z; ppos(i, x, y, z); sx += x; sy += y; sz += z; ++cnt; }
    SceneNode rn; rn.id = "room:" + std::to_string(r); rn.layer = "room";
    if (cnt) { rn.pos[0] = sx / cnt; rn.pos[1] = sy / cnt; rn.pos[2] = sz / cnt; }
    sg.add_node(rn, "building:0", true);
  }

  for (int i = 0; i < P; ++i) {
    double x, y, z; ppos(i, x, y, z);
    SceneNode pn; pn.id = "place:" + std::to_string(i); pn.layer = "place";
    pn.pos[0] = x; pn.pos[1] = y; pn.pos[2] = z;
    std::string parent = (room_of[i] >= 0) ? ("room:" + std::to_string(room_of[i])) : "building:0";
    sg.add_node(pn, parent, true);
  }

  // 物体的 place_id 由调用方 link_to_places 外部设置(镜像 Python: build 用 o.place_id)。
  const int O = (int)objects.size();
  std::vector<int> place_id(O, -1);
  for (int j = 0; j < O; ++j) place_id[j] = objects[j].place_id;

  // objects: 先全加, 再定父
  std::vector<std::string> ids(O);
  for (int j = 0; j < O; ++j) ids[j] = obj_ids.empty() ? ("object:" + std::to_string(j)) : obj_ids[j];
  for (int j = 0; j < O; ++j) {
    const ObjectNode& o = objects[j];
    SceneNode on; on.id = ids[j]; on.layer = "object";
    on.pos[0] = (o.ci + vmin[0]) * (double)vs; on.pos[1] = (o.cj + vmin[1]) * (double)vs;
    on.pos[2] = (o.ck + vmin[2]) * (double)vs;
    on.obj_class = o.label; on.voxel_count = o.voxel_count; on.feat = o.feat;
    on.place_id = place_id[j]; on.misses = 0; on.seen_count = 1;
    for (int a = 0; a < 3; ++a) {
      on.bmin[a] = o.bmin[a]; on.bmax[a] = o.bmax[a];
      on.bmin_m[a] = (o.bmin[a] + vmin[a]) * (double)vs;
      on.bmax_m[a] = (o.bmax[a] + vmin[a]) * (double)vs;
    }
    sg.add_node(on);  // 无父
  }
  for (int j = 0; j < O; ++j) {
    const double* bj_min = sg.nodes[ids[j]].bmin_m; const double* bj_max = sg.nodes[ids[j]].bmax_m;
    double o_bottom_y = bj_min[1];
    std::string best_support; double best_top = -1e18;
    for (int k = 0; k < O; ++k) {
      if (k == j) continue;
      const double* bk_min = sg.nodes[ids[k]].bmin_m; const double* bk_max = sg.nodes[ids[k]].bmax_m;
      double k_top = bk_max[1];
      if (!(k_top <= o_bottom_y && o_bottom_y <= k_top + support_gap_m)) continue;
      if (bj_max[0] < bk_min[0] || bj_min[0] > bk_max[0]) continue;
      if (bj_max[2] < bk_min[2] || bj_min[2] > bk_max[2]) continue;
      if (k_top > best_top) { best_top = k_top; best_support = ids[k]; }
    }
    std::string parent;
    if (!best_support.empty()) parent = best_support;
    else {
      parent = "building:0";
      int pid = place_id[j];
      if (pid >= 0 && pid < P && room_of[pid] >= 0) parent = "room:" + std::to_string(room_of[pid]);
    }
    sg.set_parent(ids[j], parent);
  }
  return sg;
}

// ===================== Hungarian(min-cost assignment, n<=m)=====================
// e-maxx 版; 返回 assign[i]=j (i in 0..n-1)。
static std::vector<int> hungarian(const std::vector<std::vector<double>>& a, int n, int m) {
  const double INF = 1e18;
  std::vector<double> u(n + 1, 0), v(m + 1, 0);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);
  for (int i = 1; i <= n; ++i) {
    p[0] = i; int j0 = 0;
    std::vector<double> minv(m + 1, INF); std::vector<char> used(m + 1, 0);
    do {
      used[j0] = 1; int i0 = p[j0], j1 = -1; double delta = INF;
      for (int j = 1; j <= m; ++j) if (!used[j]) {
        double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] != 0);
    do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  std::vector<int> assign(n, -1);
  for (int j = 1; j <= m; ++j) if (p[j] >= 1 && p[j] <= n) assign[p[j] - 1] = j - 1;
  return assign;
}

// ===================== merge_observation =====================
SceneGraph merge_observation(SceneGraph& existing, const SkelGraph& fresh_g,
                             const std::vector<int>& fresh_room_of,
                             const std::vector<ObjectNode>& fresh_objects, const long vmin[3],
                             float vs, MergeStats& stats, int& new_id_counter,
                             const std::string& world_id, float support_gap_m,
                             float match_radius_m, int max_misses) {
  struct ExInfo { std::string id; int cls; double pos[3]; std::array<double, 3> feat; int misses; int seen; };
  std::vector<ExInfo> ex_list;  // 插入序(= nodes_by_layer 序)
  for (SceneNode* n : existing.nodes_by_layer("object")) {
    ExInfo e; e.id = n->id; e.cls = n->obj_class;
    e.pos[0] = n->pos[0]; e.pos[1] = n->pos[1]; e.pos[2] = n->pos[2];
    e.feat = n->feat; e.misses = n->misses; e.seen = n->seen_count;
    ex_list.push_back(e);
  }
  // 按类分组(保留插入序)
  std::map<int, std::vector<int>> ex_by_class;   // cls -> ex_list 索引
  for (int i = 0; i < (int)ex_list.size(); ++i) ex_by_class[ex_list[i].cls].push_back(i);
  std::map<int, std::vector<int>> fresh_by_class; // cls -> fresh 索引
  for (int j = 0; j < (int)fresh_objects.size(); ++j) fresh_by_class[fresh_objects[j].label].push_back(j);

  std::map<int, std::string> matched;  // fresh idx -> existing id
  std::set<int> all_classes;
  for (auto& kv : ex_by_class) all_classes.insert(kv.first);
  for (auto& kv : fresh_by_class) all_classes.insert(kv.first);

  for (int cls : all_classes) {
    auto ei = ex_by_class.find(cls), fi = fresh_by_class.find(cls);
    if (ei == ex_by_class.end() || fi == fresh_by_class.end()) continue;
    const auto& exC = ei->second; const auto& frC = fi->second;
    int ne = (int)exC.size(), nf = (int)frC.size();
    // cost[i][jj] = dist + feature_cost
    std::vector<std::vector<double>> cost(ne, std::vector<double>(nf, 0));
    for (int i = 0; i < ne; ++i)
      for (int jj = 0; jj < nf; ++jj) {
        const ObjectNode& fo = fresh_objects[frC[jj]];
        double fx = (fo.ci + vmin[0]) * (double)vs, fy = (fo.cj + vmin[1]) * (double)vs,
               fz = (fo.ck + vmin[2]) * (double)vs;
        const double* ep = ex_list[exC[i]].pos;
        double d = std::sqrt((ep[0]-fx)*(ep[0]-fx) + (ep[1]-fy)*(ep[1]-fy) + (ep[2]-fz)*(ep[2]-fz));
        cost[i][jj] = d + feature_cost(ex_list[exC[i]].feat, fo.feat);
      }
    // scipy linear_sum_assignment: min(ne,nf) 对
    std::vector<std::pair<int, int>> pairs;  // (ex_local_i, fr_local_jj)
    if (ne <= nf) {
      auto as = hungarian(cost, ne, nf);
      for (int i = 0; i < ne; ++i) if (as[i] >= 0) pairs.push_back({i, as[i]});
    } else {
      std::vector<std::vector<double>> ct(nf, std::vector<double>(ne, 0));
      for (int i = 0; i < ne; ++i) for (int jj = 0; jj < nf; ++jj) ct[jj][i] = cost[i][jj];
      auto as = hungarian(ct, nf, ne);
      for (int jj = 0; jj < nf; ++jj) if (as[jj] >= 0) pairs.push_back({as[jj], jj});
    }
    for (auto& pr : pairs) {
      const ObjectNode& fo = fresh_objects[frC[pr.second]];
      double fx = (fo.ci + vmin[0]) * (double)vs, fy = (fo.cj + vmin[1]) * (double)vs,
             fz = (fo.ck + vmin[2]) * (double)vs;
      const double* ep = ex_list[exC[pr.first]].pos;
      double d = std::sqrt((ep[0]-fx)*(ep[0]-fx) + (ep[1]-fy)*(ep[1]-fy) + (ep[2]-fz)*(ep[2]-fz));
      if (d <= match_radius_m) matched[frC[pr.second]] = ex_list[exC[pr.first]].id;
    }
  }

  // Step3: obj_ids
  std::vector<std::string> obj_ids(fresh_objects.size());
  for (int j = 0; j < (int)fresh_objects.size(); ++j) {
    auto it = matched.find(j);
    if (it != matched.end()) obj_ids[j] = it->second;
    else { char buf[32]; std::snprintf(buf, sizeof(buf), "object:%08x", new_id_counter++); obj_ids[j] = buf; }
  }

  // Step4: 新场景图
  SceneGraph new_sg = build_scene_graph(fresh_g, fresh_room_of, fresh_objects, vmin, vs,
                                        world_id, support_gap_m, obj_ids);
  // ex seen_count 查表
  std::map<std::string, int> ex_seen;
  for (auto& e : ex_list) ex_seen[e.id] = e.seen;
  for (int j = 0; j < (int)obj_ids.size(); ++j) {
    SceneNode* node = new_sg.get(obj_ids[j]);
    if (!node) continue;
    node->misses = 0;
    if (matched.count(j)) node->seen_count = ex_seen[obj_ids[j]] + 1;  // 承接旧 seen+1
  }

  // Step5: carry-overs
  std::set<std::string> seen_existing;
  for (auto& kv : matched) seen_existing.insert(kv.second);
  int removed = 0, carried = 0;
  for (auto& e : ex_list) {  // 插入序
    if (seen_existing.count(e.id)) continue;
    int new_misses = e.misses + 1;
    if (new_misses > max_misses) { ++removed; continue; }
    ++carried;
    std::string carry_parent = "building:0";
    for (SceneNode* anc : existing.ancestors(e.id))
      if (anc->layer == "room" && new_sg.nodes.count(anc->id)) { carry_parent = anc->id; break; }
    SceneNode* ex_node = existing.get(e.id);
    SceneNode carry = *ex_node;      // 承接全部 attrs
    carry.children.clear(); carry.parent.clear();
    carry.misses = new_misses;
    new_sg.add_node(carry, carry_parent, true);
  }

  stats.matched = (int)matched.size();
  stats.added = (int)obj_ids.size() - stats.matched;
  stats.removed = removed;
  stats.carried = carried;
  stats.objects_total = (int)new_sg.nodes_by_layer("object").size();
  return new_sg;
}

// ===================== consolidate_fragments =====================
static double bbox_gap_m(const double* amin, const double* amax, const double* bmin, const double* bmax) {
  double g = -1e300;
  for (int a = 0; a < 3; ++a) g = std::max(g, std::max(amin[a] - bmax[a], bmin[a] - amax[a]));
  return g;
}

int consolidate_fragments(SceneGraph& sg, double gap_m, int min_persist) {
  std::vector<SceneNode*> cand;
  for (SceneNode* n : sg.nodes_by_layer("object"))
    if (n->seen_count >= min_persist) cand.push_back(n);  // 所有 object 都有 bbox_m

  std::map<std::string, std::string> uf;
  for (auto* n : cand) uf[n->id] = n->id;
  std::function<std::string(std::string)> find = [&](std::string x) {
    while (uf[x] != x) { uf[x] = uf[uf[x]]; x = uf[x]; }
    return x;
  };
  int n = (int)cand.size();
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j) {
      if (cand[i]->obj_class != cand[j]->obj_class) continue;
      if (bbox_gap_m(cand[i]->bmin_m, cand[i]->bmax_m, cand[j]->bmin_m, cand[j]->bmax_m) <= gap_m)
        uf[find(cand[i]->id)] = find(cand[j]->id);
    }
  // 按 root 分组(候选序)
  std::vector<std::string> group_order;
  std::map<std::string, std::vector<SceneNode*>> groups;
  for (auto* nd : cand) {
    std::string r = find(nd->id);
    if (!groups.count(r)) group_order.push_back(r);
    groups[r].push_back(nd);
  }
  int merges_done = 0;
  for (auto& r : group_order) {
    auto& group = groups[r];
    if (group.size() < 2) continue;
    SceneNode* survivor = group[0];  // max voxel_count, 平局取首个
    for (auto* nd : group) if (nd->voxel_count > survivor->voxel_count) survivor = nd;
    for (auto* absorbed : group) {
      if (absorbed->id == survivor->id) continue;
      for (int a = 0; a < 3; ++a) {
        survivor->bmin_m[a] = std::min(survivor->bmin_m[a], absorbed->bmin_m[a]);
        survivor->bmax_m[a] = std::max(survivor->bmax_m[a], absorbed->bmax_m[a]);
        survivor->bmin[a] = std::min(survivor->bmin[a], absorbed->bmin[a]);
        survivor->bmax[a] = std::max(survivor->bmax[a], absorbed->bmax[a]);
      }
      int vc_s = survivor->voxel_count, vc_a = absorbed->voxel_count, tot = vc_s + vc_a;
      for (int a = 0; a < 3; ++a)
        survivor->pos[a] = (vc_s * survivor->pos[a] + vc_a * absorbed->pos[a]) / tot;
      survivor->voxel_count = tot;
      survivor->seen_count = std::max(survivor->seen_count, absorbed->seen_count);
      for (const std::string& cid : std::vector<std::string>(absorbed->children))
        sg.set_parent(cid, survivor->id);
      sg.remove_node(absorbed->id, false);
      ++merges_done;
    }
  }
  return merges_done;
}

}  // namespace smc
