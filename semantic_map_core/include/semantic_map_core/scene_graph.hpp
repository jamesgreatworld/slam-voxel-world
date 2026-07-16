// scene_graph.hpp — 层次动态场景图(DSG): Building>Room>{Place,Object} + 支撑父子,
// 以及在线增量 merge_observation / consolidate_fragments。
// 移植自 m3_adapter/gvd/scene_graph.py(+ objects.link_to_places / features.feature_cost)。
#pragma once
#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "semantic_map_core/graph.hpp"
#include "semantic_map_core/objects.hpp"

namespace smc {

struct SceneNode {
  std::string id;
  std::string layer;   // building|room|place|object
  double pos[3] = {0, 0, 0};
  std::string parent;  // "" = 无
  std::vector<std::string> children;
  // ---- object 专用 attrs(merge/consolidate 用)----
  int obj_class = -1;         // super_id
  int voxel_count = 0;
  int bmin[3] = {0, 0, 0}, bmax[3] = {0, 0, 0};          // voxel bbox
  double bmin_m[3] = {0, 0, 0}, bmax_m[3] = {0, 0, 0};   // metre bbox
  std::array<double, 3> feat = {0, 0, 0};                // shape 描述子
  int place_id = -1;
  int misses = 0;
  int seen_count = 0;
};

// 保持插入顺序的节点表(复刻 Python dict 语义)。
class SceneGraph {
 public:
  std::vector<std::string> order;                    // 插入顺序 id
  std::unordered_map<std::string, SceneNode> nodes;  // id -> node
  std::string root_id;

  void add_node(const SceneNode& n, const std::string& parent_id = std::string(),
                bool has_parent = false);
  SceneNode* get(const std::string& id);
  void set_parent(const std::string& id, const std::string& new_parent);
  void remove_node(const std::string& id, bool reparent_children = true);
  std::vector<SceneNode*> nodes_by_layer(const std::string& layer);  // 插入序
  std::vector<SceneNode*> ancestors(const std::string& id);
};

// 构建一帧场景图。obj_ids 为空则用默认 "object:{j}"。
SceneGraph build_scene_graph(const SkelGraph& g, const std::vector<int>& room_of,
                             const std::vector<ObjectNode>& objects,
                             const long vmin[3], float voxel_size,
                             const std::string& world_id = "apartment",
                             float support_gap_m = 0.20f,
                             const std::vector<std::string>& obj_ids = {});

struct MergeStats { int matched, added, removed, carried, objects_total; };

// 增量融合: existing + 新一帧(fresh 图 + 物体) -> 新 SceneGraph。
// new_id_gen: 给未匹配新物体分配 id 的计数器起点(对拍用确定性 id "object:{counter:08x}")。
SceneGraph merge_observation(SceneGraph& existing, const SkelGraph& fresh_g,
                             const std::vector<int>& fresh_room_of,
                             const std::vector<ObjectNode>& fresh_objects,
                             const long vmin[3], float voxel_size, MergeStats& stats,
                             int& new_id_counter, const std::string& world_id = "apartment",
                             float support_gap_m = 0.20f, float match_radius_m = 0.5f,
                             int max_misses = 3);

// 时序碎片合并: 同类、bbox(米)间距<=gap_m、且都 seen_count>=min_persist 的物体节点合并。
int consolidate_fragments(SceneGraph& sg, double gap_m = 0.15, int min_persist = 2);

}  // namespace smc
