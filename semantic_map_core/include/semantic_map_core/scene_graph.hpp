// scene_graph.hpp — DSG 组装: Building>Room>{Place,Object} + 支撑父子。
// 移植自 m3_adapter/gvd/scene_graph.py::build_scene_graph(+ link_to_places)。
#pragma once
#include <string>
#include <vector>

#include "semantic_map_core/graph.hpp"
#include "semantic_map_core/objects.hpp"

namespace smc {

struct SgNode {
  std::string id;      // building:0 / room:{r} / place:{i} / object:{j}
  std::string layer;   // building|room|place|object
  std::string parent;  // 父 id(空=无)
  double x, y, z;      // 世界米坐标
  int obj_class = -1;  // 物体的 class(super_id), 非物体为 -1
  int voxel_count = 0;
};

// g: places 图; room_of: 每 place 的房间号(partition_rooms_clearance);
// objects: 物体(需先算 place_id, 内部会 link_to_places)。
std::vector<SgNode> build_scene_graph(const SkelGraph& g, const std::vector<int>& room_of,
                                      const std::vector<ObjectNode>& objects,
                                      const long vmin[3], float voxel_size,
                                      const std::string& world_id = "apartment",
                                      float support_gap_m = 0.20f);

}  // namespace smc
