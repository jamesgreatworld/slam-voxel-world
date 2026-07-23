// vlayer.hpp — L1 结构覆盖层(相对只读 L0 观测的稀疏体素差异)。
// 移植自 m3_adapter/vlayer/{overlay,pipeline}.py。用户/generator 的编辑都表示为 VoxelDelta。
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace smc {

// L0 只读视图(generator/compose 只需 occ/sem/free 三网格 + 尺寸, 与 Python 一致)。
struct MapView {
  const uint8_t* occ;   // nx*ny*nz 行主序 (x*ny+y)*nz+z
  const uint8_t* sem;
  const uint8_t* free;  // observed_free_mask
  int nx, ny, nz;
  float voxel_size;
  const float* logodds = nullptr;  // 可选(PlaneRegularize 需要)
  inline long id(int x, int y, int z) const { return ((long)x * ny + y) * nz + z; }
};

enum class DeltaOp { ADD = 0, REMOVE = 1, REPLACE = 2 };

struct VoxelDelta {
  int idx[3];                          // (x,y,z) dense 网格索引
  DeltaOp op = DeltaOp::ADD;
  int sem = 0;                         // super_id(add/replace 用), [0,255]
  std::string generator = "manual";
  std::string binding = "persistent";  // live | persistent | independent
};

class Overlay {
 public:
  std::vector<VoxelDelta> voxels;
  void add_voxel(int x, int y, int z, int sem = 0,
                 const std::string& generator = "manual",
                 const std::string& binding = "persistent", DeltaOp op = DeltaOp::ADD);
  bool save(const std::string& path) const;   // 自研 JSON
  static Overlay load(const std::string& path);
};

// 把 overlay 差异叠到 L0 -> 输出态 (occ, sem)。
// completed(非 manual/independent)的 add 只填未占据格; manual/independent 强制写;
// remove 抹掉; replace 只给已占据格改语义。
void compose_structure(const MapView& m, const Overlay& overlay,
                       std::vector<uint8_t>& occ, std::vector<uint8_t>& sem);

// ---- 先验推理流水线 ----
struct Generator {
  std::string id;
  int stage = 0;
  std::vector<std::string> depends_on;
  std::string default_binding = "persistent";
  virtual ~Generator() = default;
  // acc = 到此为止已累积的 overlay(前序 generator 的产出)。多数 generator 忽略它;
  // OpeningCarve 需要读它拿 wall_fill 补墙格。复刻 Python 的 ctx.overlay。
  virtual std::vector<VoxelDelta> run(const MapView& m, const Overlay& acc) const = 0;
};

std::vector<const Generator*> toposort(const std::vector<const Generator*>& gens);
Overlay run_pipeline(const MapView& m, const std::vector<const Generator*>& gens);

}  // namespace smc
