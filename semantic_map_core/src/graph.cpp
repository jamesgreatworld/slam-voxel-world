// graph.cpp — Hydra 式压缩抽图 + skeleton_to_graph(移植自 graph.py)。见 graph.hpp。
#include "semantic_map_core/graph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

namespace smc {

namespace {
inline long IDX(int i, int j, int k, int ny, int nz) {
  return (static_cast<long>(i) * ny + j) * nz + k;
}
// 把 (cx,cy,cz) 粗格坐标打包成 64bit key(偏移防负)
inline uint64_t ckey(long cx, long cy, long cz) {
  const uint64_t K = 1u << 20;
  return ((uint64_t)(cx + K) << 42) | ((uint64_t)(cy + K) << 21) | (uint64_t)(cz + K);
}
}  // namespace

PlacesGraph compress_gvd_graph(const uint8_t* gvd, const float* dist, int nx, int ny,
                               int nz, const long vmin[3], float vs,
                               float compression_m, float min_node_dist_m,
                               float node_merge_m) {
  PlacesGraph out;
  const double comp = compression_m;

  // ---- 1. 每个合格 GVD 体素量化到粗格; 每格取 max-clearance 成员当代表 ----
  struct Cell { int node; };
  std::unordered_map<uint64_t, int> cell_to_node;  // 粗格 key -> 节点索引
  cell_to_node.reserve(4096);

  auto world = [&](int i, int j, int k, double& wx, double& wy, double& wz) {
    wx = (i + vmin[0] + 0.5) * vs;
    wy = (j + vmin[1] + 0.5) * vs;
    wz = (k + vmin[2] + 0.5) * vs;
  };

  // 临时: 节点的粗格 key(建边时判邻格用) + 每格 key
  std::vector<uint64_t> node_key;
  // 体素 -> 节点索引(建边用), 用 map 省内存
  std::unordered_map<long, int> vox_node;
  vox_node.reserve(1 << 16);

  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j)
      for (int k = 0; k < nz; ++k) {
        long id = IDX(i, j, k, ny, nz);
        if (!gvd[id]) continue;
        float cl = dist[id];
        if (cl < min_node_dist_m) continue;
        double wx, wy, wz; world(i, j, k, wx, wy, wz);
        long cx = (long)std::floor(wx / comp), cy = (long)std::floor(wy / comp),
             cz = (long)std::floor(wz / comp);
        uint64_t key = ckey(cx, cy, cz);
        auto it = cell_to_node.find(key);
        int ni;
        if (it == cell_to_node.end()) {
          ni = (int)out.nodes.size();
          cell_to_node[key] = ni;
          out.nodes.push_back({wx, wy, wz, cl, 1, 0});
          node_key.push_back(key);
        } else {
          ni = it->second;
          PlaceNode& n = out.nodes[ni];
          n.members += 1;
          if (cl > n.clearance_m) {  // 代表 = 净空最大的成员
            n.clearance_m = cl; n.x = wx; n.y = wy; n.z = wz;
          }
        }
        vox_node[id] = ni;
      }

  // ---- 2. 边: 相邻 GVD 体素若属不同粗格节点 -> 两节点连边(去重) ----
  const int off[13][3] = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,-1,0},{1,0,1},
                          {1,0,-1},{0,1,1},{0,1,-1},{1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1}};
  std::unordered_map<uint64_t, int> edge_seen;  // (min<<32|max) -> edge idx
  for (auto& kv : vox_node) {
    long id = kv.first; int na = kv.second;
    int i = id / ((long)ny * nz); long r = id % ((long)ny * nz);
    int j = r / nz, k = r % nz;
    for (int e = 0; e < 13; ++e) {
      int ni2 = i + off[e][0], nj2 = j + off[e][1], nk2 = k + off[e][2];
      if (ni2 < 0 || ni2 >= nx || nj2 < 0 || nj2 >= ny || nk2 < 0 || nk2 >= nz) continue;
      auto it = vox_node.find(IDX(ni2, nj2, nk2, ny, nz));
      if (it == vox_node.end()) continue;
      int nb = it->second;
      if (nb == na) continue;
      int lo = na < nb ? na : nb, hi = na < nb ? nb : na;
      uint64_t ek = ((uint64_t)lo << 32) | (uint32_t)hi;
      if (edge_seen.count(ek)) continue;
      double dx = out.nodes[lo].x - out.nodes[hi].x, dy = out.nodes[lo].y - out.nodes[hi].y,
             dz = out.nodes[lo].z - out.nodes[hi].z;
      edge_seen[ek] = (int)out.edges.size();
      out.edges.push_back({lo, hi, (float)std::sqrt(dx * dx + dy * dy + dz * dz)});
    }
  }

  // ---- 3. 度数 ----
  for (auto& e : out.edges) { out.nodes[e.a].degree++; out.nodes[e.b].degree++; }
  return out;
}

// ===================== skeleton_to_graph(移植自 graph.py)=====================
namespace {
int uf_find(std::vector<int>& p, int i) {
  int r = i;
  while (p[r] != r) r = p[r];
  while (p[i] != r) { int t = p[i]; p[i] = r; i = t; }
  return r;
}
}  // namespace

SkelGraph skeleton_to_graph(const uint8_t* gvd, const float* dist, int nx, int ny,
                            int nz, float vs, float merge_radius_m) {
  SkelGraph out;
  const long nynz = (long)ny * nz;

  auto is_gvd = [&](int i, int j, int k) {
    return i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz && gvd[IDX(i, j, k, ny, nz)];
  };

  // 1. degree = 26 邻居中 gvd 的数量; key = gvd & degree!=2。key_coords 按 C 序。
  std::vector<std::array<int, 3>> key_coords;
  std::unordered_map<long, int> keyidx;  // voxid -> key index
  bool any_gvd = false;
  std::array<int, 3> first_gvd{0, 0, 0};
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j)
      for (int k = 0; k < nz; ++k) {
        if (!gvd[IDX(i, j, k, ny, nz)]) continue;
        if (!any_gvd) { any_gvd = true; first_gvd = {i, j, k}; }
        int deg = 0;
        for (int di = -1; di <= 1; ++di)
          for (int dj = -1; dj <= 1; ++dj)
            for (int dk = -1; dk <= 1; ++dk) {
              if (!di && !dj && !dk) continue;
              if (is_gvd(i + di, j + dj, k + dk)) ++deg;
            }
        if (deg != 2) {
          keyidx[IDX(i, j, k, ny, nz)] = (int)key_coords.size();
          key_coords.push_back({i, j, k});
        }
      }

  if (!any_gvd) return out;
  if (key_coords.empty()) {  // loop-only
    out.nodes.push_back({first_gvd[0], first_gvd[1], first_gvd[2],
                         dist[IDX(first_gvd[0], first_gvd[1], first_gvd[2], ny, nz)], 0, 4});
    return out;
  }

  const int n = (int)key_coords.size();
  const double eps = std::max((double)merge_radius_m / vs, std::sqrt(3.0));
  const double eps2 = eps * eps;
  const int R = (int)std::ceil(eps);

  // 2. DBSCAN(min_samples=1): eps 连通分量, union 按 min 根
  std::vector<int> parent(n);
  for (int i = 0; i < n; ++i) parent[i] = i;
  for (int ka = 0; ka < n; ++ka) {
    int i = key_coords[ka][0], j = key_coords[ka][1], k = key_coords[ka][2];
    for (int di = -R; di <= R; ++di)
      for (int dj = -R; dj <= R; ++dj)
        for (int dk = -R; dk <= R; ++dk) {
          if (!di && !dj && !dk) continue;
          if ((double)(di * di + dj * dj + dk * dk) > eps2) continue;
          int ni = i + di, nj = j + dj, nk = k + dk;
          if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;
          auto it = keyidx.find(IDX(ni, nj, nk, ny, nz));
          if (it == keyidx.end()) continue;
          int ra = uf_find(parent, ka), rb = uf_find(parent, it->second);
          if (ra != rb) parent[std::max(ra, rb)] = std::min(ra, rb);
        }
  }
  // 聚类号: 按 key index 升序(=core 首次出现序)分配
  std::unordered_map<int, int> root_to_label;
  std::vector<int> labels(n);
  for (int i = 0; i < n; ++i) {
    int r = uf_find(parent, i);
    auto it = root_to_label.find(r);
    if (it == root_to_label.end()) { int lb = (int)root_to_label.size(); root_to_label[r] = lb; labels[i] = lb; }
    else labels[i] = it->second;
  }
  const int nc = (int)root_to_label.size();

  // 3. 每簇: snap(最近质心成员) + clearance(max dist)
  std::vector<std::vector<int>> members(nc);
  for (int i = 0; i < n; ++i) members[labels[i]].push_back(i);
  std::vector<std::array<int, 3>> snap(nc);
  std::vector<float> clear(nc, 0.f);
  // voxid -> cluster(建边用)
  auto key_cluster = [&](int i, int j, int k, int& out_lab) -> bool {
    auto it = keyidx.find(IDX(i, j, k, ny, nz));
    if (it == keyidx.end()) return false;
    out_lab = labels[it->second]; return true;
  };
  for (int c = 0; c < nc; ++c) {
    double cx = 0, cy = 0, cz = 0; float mx = 0;
    for (int ki : members[c]) {
      cx += key_coords[ki][0]; cy += key_coords[ki][1]; cz += key_coords[ki][2];
      float d = dist[IDX(key_coords[ki][0], key_coords[ki][1], key_coords[ki][2], ny, nz)];
      if (d > mx) mx = d;
    }
    double m = members[c].size(); cx /= m; cy /= m; cz /= m;
    double best = 1e30; int bki = members[c][0];
    for (int ki : members[c]) {
      double d2 = (key_coords[ki][0] - cx) * (key_coords[ki][0] - cx) +
                  (key_coords[ki][1] - cy) * (key_coords[ki][1] - cy) +
                  (key_coords[ki][2] - cz) * (key_coords[ki][2] - cz);
      if (d2 < best) { best = d2; bki = ki; }
    }
    snap[c] = key_coords[bki]; clear[c] = mx;
  }

  // 4. 边: edge_len 保最小
  std::map<std::pair<int, int>, float> edge_len;
  auto add_edge = [&](int a, int b, float len) {
    if (a == b) return;
    auto e = std::make_pair(std::min(a, b), std::max(a, b));
    auto it = edge_len.find(e);
    if (it == edge_len.end() || len < it->second) edge_len[e] = len;
  };

  // 4a. chain 边: chain = gvd & ~key; 26 连通分量; 恰触 2 簇 -> 边(长=段体素数*vs)
  std::unordered_map<long, char> visited;
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j)
      for (int k = 0; k < nz; ++k) {
        long id = IDX(i, j, k, ny, nz);
        if (!gvd[id] || keyidx.count(id)) continue;  // 只走 chain
        if (visited.count(id)) continue;
        // BFS 一个 chain 分量
        std::queue<std::array<int, 3>> q; q.push({i, j, k}); visited[id] = 1;
        int size = 0; std::set<int> touched;
        while (!q.empty()) {
          auto cur = q.front(); q.pop(); ++size;
          for (int di = -1; di <= 1; ++di)
            for (int dj = -1; dj <= 1; ++dj)
              for (int dk = -1; dk <= 1; ++dk) {
                if (!di && !dj && !dk) continue;
                int ni = cur[0] + di, nj = cur[1] + dj, nk = cur[2] + dk;
                if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;
                int lab;
                if (key_cluster(ni, nj, nk, lab)) { touched.insert(lab); continue; }
                long nid = IDX(ni, nj, nk, ny, nz);
                if (gvd[nid] && !keyidx.count(nid) && !visited.count(nid)) {
                  visited[nid] = 1; q.push({ni, nj, nk});
                }
              }
        }
        if (touched.size() == 2) {
          auto it = touched.begin(); int a = *it; ++it; int b = *it;
          add_edge(a, b, (float)size * vs);
        }
      }

  // 4b. 直连: 不同簇的 key 体素 26 相邻 -> 边(长 vs)
  for (int ka = 0; ka < n; ++ka) {
    int i = key_coords[ka][0], j = key_coords[ka][1], k = key_coords[ka][2];
    int la = labels[ka];
    for (int di = -1; di <= 1; ++di)
      for (int dj = -1; dj <= 1; ++dj)
        for (int dk = -1; dk <= 1; ++dk) {
          if (!di && !dj && !dk) continue;
          int lb;
          if (key_cluster(i + di, j + dj, k + dk, lb) && lb != la) add_edge(la, lb, vs);
        }
  }

  // 5. 组装(edges 按 (a,b) 排序, std::map 天然有序)+ 度数/类型
  std::vector<int> deg(nc, 0);
  for (auto& kv : edge_len) { deg[kv.first.first]++; deg[kv.first.second]++; }
  out.nodes.resize(nc);
  for (int c = 0; c < nc; ++c) {
    int t = deg[c] >= 3 ? 0 : deg[c] == 2 ? 1 : deg[c] == 1 ? 2 : 3;
    out.nodes[c] = {snap[c][0], snap[c][1], snap[c][2], clear[c], deg[c], t};
  }
  for (auto& kv : edge_len)
    out.edges.push_back({kv.first.first, kv.first.second, kv.second});
  return out;
}

std::vector<int> partition_rooms_clearance(const SkelGraph& g, float door_clearance_m,
                                           int min_room_nodes) {
  const int n = (int)g.nodes.size();
  if (n == 0) return {};
  // 邻接表(按 edge 顺序 append 两向, 与 python adjacency() dict 插入序一致)
  std::vector<std::vector<int>> adj(n);
  for (auto& e : g.edges) { adj[e.a].push_back(e.b); adj[e.b].push_back(e.a); }
  std::vector<char> is_door(n);
  for (int i = 0; i < n; ++i) is_door[i] = g.nodes[i].clearance_m < door_clearance_m;
  std::vector<int> room_of(n, -1);
  int rid = 0;
  // 1. 非门口连通分量 = 房间核(DFS/LIFO, 起点按 index 序)
  for (int s = 0; s < n; ++s) {
    if (is_door[s] || room_of[s] != -1) continue;
    std::vector<int> st{s}; room_of[s] = rid;
    while (!st.empty()) {
      int u = st.back(); st.pop_back();
      for (int v : adj[u]) if (!is_door[v] && room_of[v] == -1) { room_of[v] = rid; st.push_back(v); }
    }
    ++rid;
  }
  // 2. 门口/未标 -> 多源 BFS(frontier 按 index 序初始化, FIFO)
  std::deque<int> fr;
  for (int i = 0; i < n; ++i) if (room_of[i] != -1) fr.push_back(i);
  while (!fr.empty()) {
    int u = fr.front(); fr.pop_front();
    for (int v : adj[u]) if (room_of[v] == -1) { room_of[v] = room_of[u]; fr.push_back(v); }
  }
  // 3. 孤立节点各自成房间
  for (int i = 0; i < n; ++i) if (room_of[i] == -1) { room_of[i] = rid; ++rid; }
  // 4. 过小房间并入最常见相邻房间(迭代到稳定; small 按 id 升序)
  bool changed = true;
  while (changed) {
    changed = false;
    std::unordered_map<int, int> sizes;
    for (int r : room_of) sizes[r]++;
    std::vector<int> small;
    for (auto& kv : sizes) if (kv.second < min_room_nodes) small.push_back(kv.first);
    std::sort(small.begin(), small.end());
    if (small.empty()) break;
    for (int r : small) {
      std::vector<int> members;
      for (int i = 0; i < n; ++i) if (room_of[i] == r) members.push_back(i);
      std::unordered_map<int, int> cnt; std::vector<int> order;
      for (int i : members)
        for (int v : adj[i]) {
          int rv = room_of[v];
          if (rv != r) { if (!cnt.count(rv)) order.push_back(rv); cnt[rv]++; }
        }
      if (!order.empty()) {
        int tgt = order[0], best = cnt[order[0]];
        for (int rr : order) if (cnt[rr] > best) { best = cnt[rr]; tgt = rr; }
        for (int i : members) room_of[i] = tgt;
        changed = true;
      }
    }
  }
  // 5. 紧致编号(sorted unique -> 0..k-1)
  std::set<int> uniq(room_of.begin(), room_of.end());
  std::unordered_map<int, int> remap; int k = 0;
  for (int r : uniq) remap[r] = k++;
  for (auto& r : room_of) r = remap[r];
  return room_of;
}

}  // namespace smc
