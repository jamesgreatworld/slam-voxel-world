// bench_vlayer.cpp — vlayer 对拍: House occ/sem/free 上跑 SlabFill(floor+ceiling) 流水线,
// dump deltas + compose 后的 occ/sem 供 Python 逐格对拍。
#include "semantic_map_core/vlayer.hpp"
#include "semantic_map_core/vlayer_generators.hpp"

// 统计各 generator 单独产出(诊断: 看哪些在 House 上非空)

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
  std::vector<uint8_t> occ(N), sem(N), fre(N);
  { FILE* f = std::fopen((O + "house_occ.bin").c_str(), "rb"); rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_sem.bin").c_str(), "rb"); rd = std::fread(sem.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_free.bin").c_str(), "rb"); rd = std::fread(fre.data(), 1, N, f); std::fclose(f); }
  (void)rd;

  MapView m{occ.data(), sem.data(), fre.data(), nx, ny, nz, 0.1f};

  // floor=31, ceiling=... House labelspace: floor 31。ceiling 取一个存在的结构类做管线测试。
  // 用 floor+ceiling 两个 slab 走完整 toposort。ceiling label 用 31(同类, 只测 side 分支)。
  SlabFill sfloor(31, "floor", 0.6, 0.5, 2);
  SlabFill sceil(31, "ceiling", 0.6, 0.5, 2);
  WallFill wall(0.10, 20, 4, 4, 2);
  OcclusionFill occl(5, 19);
  std::vector<const Generator*> gens{&sfloor, &sceil, &wall, &occl};
  Overlay ov = run_pipeline(m, gens);

  std::vector<uint8_t> cocc, csem;
  compose_structure(m, ov, cocc, csem);

  std::printf("C++ per-gen: slab_floor=%zu slab_ceil=%zu wall=%zu occl=%zu  total=%zu  occ_after=%ld\n",
              sfloor.run(m).size(), sceil.run(m).size(), wall.run(m).size(), occl.run(m).size(),
              ov.voxels.size(), [&] { long c = 0; for (uint8_t v : cocc) c += v; return c; }());

  // dump deltas(x,y,z,op,sem) 排序集合无关 + compose 网格
  FILE* f = std::fopen((O + "vlayer_cpp.bin").c_str(), "wb");
  int nd = (int)ov.voxels.size(); std::fwrite(&nd, 4, 1, f);
  for (auto& d : ov.voxels) {
    int v[5] = {d.idx[0], d.idx[1], d.idx[2], (int)d.op, d.sem};
    std::fwrite(v, 4, 5, f);
  }
  std::fwrite(cocc.data(), 1, N, f);
  std::fwrite(csem.data(), 1, N, f);
  std::fclose(f);

  // ---- 合成墙(带 3x3 洞)覆盖 WallFill 非空路径 ----
  const int SX = 24, SY = 24, SZ = 40;
  std::vector<uint8_t> so((size_t)SX * SY * SZ, 0), ss(so.size(), 0), sf(so.size(), 0);
  auto sid = [&](int x, int y, int z) { return ((long)x * SY + y) * SZ + z; };
  for (int y = 3; y <= 20; ++y)
    for (int z = 3; z <= 35; ++z) { so[sid(12, y, z)] = 1; ss[sid(12, y, z)] = 19; }
  for (int y = 9; y <= 11; ++y)
    for (int z = 9; z <= 11; ++z) { so[sid(12, y, z)] = 0; ss[sid(12, y, z)] = 0; }  // 洞
  for (int x = 13; x <= 20; ++x)
    for (int y = 3; y <= 20; ++y)
      for (int z = 3; z <= 35; ++z) sf[sid(x, y, z)] = 1;  // +x 侧房间
  MapView sm{so.data(), ss.data(), sf.data(), SX, SY, SZ, 0.1f};
  auto wd = wall.run(sm);
  std::printf("C++ synth wall deltas=%zu\n", wd.size());
  FILE* g = std::fopen((O + "vlayer_synth_cpp.bin").c_str(), "wb");
  int wn = (int)wd.size(); std::fwrite(&wn, 4, 1, g);
  for (auto& d : wd) { int v[5] = {d.idx[0], d.idx[1], d.idx[2], (int)d.op, d.sem}; std::fwrite(v, 4, 5, g); }
  std::fclose(g);
  return 0;
}
