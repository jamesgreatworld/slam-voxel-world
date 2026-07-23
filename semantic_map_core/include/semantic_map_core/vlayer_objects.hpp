// vlayer_objects.hpp — 先验层物体解耦(③)与模型实例化+摆放解算(④)。
// 移植自 m3_adapter/vlayer/objects.py。实体 dict 与 entities.json 同构。
#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace smc {

struct EntityModel {
  std::string id;                 // _stable_uuid(md5)
  int label = 0;
  std::string label_name;
  double position[3] = {0, 0, 0};
  double rotation[4] = {0, 0, 0, 1};
  double bbox_dims[3] = {0, 0, 0};
  int voxel_count = 0;
  // custom_meta
  std::string mount;              // ""|ceiling|wall
  std::string mc_item;            // 预制模型 id(可空)
  std::string support_id;         // 摆放解算得出的支撑父件
  std::string support;            // snap_small_to_support 的 "furniture"
  // 内部(解算用, 输出前清除语义上等价 Python 的 _obs_*)
  double obs_pos[3] = {0, 0, 0};
  double obs_half[3] = {0, 0, 0};
};

// md5 hex(RFC1321, 自实现零依赖) — _stable_uuid 用
std::string md5_hex(const std::string& s);
std::string stable_uuid(const std::string& seed);

// 物体格掩码(occ & sem∈labels), 供解耦(结构=掩码格清零)
std::vector<uint8_t> object_cells_mask(const uint8_t* occ, const uint8_t* sem,
                                       int nx, int ny, int nz, const std::set<int>& labels);

// 每标签 26-连通分量 -> 实体(OBB 或 mc_item), 含 mount 判定与摆放解算。
std::vector<EntityModel> extract_object_models(
    const uint8_t* occ, const uint8_t* sem, int nx, int ny, int nz, const long vmin[3],
    double voxel_size, const std::map<int, std::string>& label_names,
    const std::map<int, std::string>& mc_item_map, int min_voxels,
    const std::set<int>& labels, double max_extent_m = 2.6,
    const std::map<std::string, std::array<double, 3>>& preset_extents = {},
    double coarse_vs = 0);

// 小物体吸附到家具顶面(独立后处理)
void snap_small_to_support(std::vector<EntityModel>& small_ents,
                           const std::vector<EntityModel>& furn_ents,
                           const std::map<std::string, std::array<double, 3>>& preset_extents = {},
                           double max_drop_m = 1.5);

}  // namespace smc
