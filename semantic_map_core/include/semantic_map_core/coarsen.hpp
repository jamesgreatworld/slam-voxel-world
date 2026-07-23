// coarsen.hpp — 细占据/语义网格 -> 粗块(降采样/去噪/去悬突)。移植自 vlayer/coarsen.py。
#pragma once
#include <cstdint>
#include <vector>

namespace smc {

// 细 (occ,sem,vmin,vs) -> 粗。粗块占据 = 块内占据细格数>=min_fine;
// 语义 = 占据细格 super_id 众数(严格>, 平局取小 label; 0 不参与)。
// 每轴按 vmin mod factor 对齐(Python mod, 负数非负)。
void downsample_occupancy(const uint8_t* occ, const uint8_t* sem, int nx, int ny, int nz,
                          const long vmin[3], float vs, int factor, int min_fine,
                          std::vector<uint8_t>& occ_c, std::vector<uint8_t>& sem_c,
                          long vmin_c[3], float& vs_c, int dims_c[3]);

// 迭代移除 6-邻支撑<=1 的占据格(尖刺/细链末端)。就地修改。
void prune_dangles(std::vector<uint8_t>& occ, std::vector<uint8_t>& sem,
                   int nx, int ny, int nz, int iterations = 2);

// 6-连通 closing 补小洞(新格 sem=最近原占据格)+ 删 <min_component 的连通分量
// (+可选只留最大分量)。就地修改。
void clean_coarse(std::vector<uint8_t>& occ, std::vector<uint8_t>& sem,
                  int nx, int ny, int nz, int min_component = 2, int close_radius = 1,
                  bool keep_largest = false);

}  // namespace smc
