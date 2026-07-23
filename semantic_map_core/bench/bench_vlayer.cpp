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
  RoofCap roof(0.5, 2);
  std::vector<const Generator*> gens{&sfloor, &sceil, &wall, &occl, &roof};
  Overlay ov = run_pipeline(m, gens);
  Overlay empty;

  std::vector<uint8_t> cocc, csem;
  compose_structure(m, ov, cocc, csem);

  std::printf("C++ per-gen: slab_floor=%zu slab_ceil=%zu wall=%zu occl=%zu roof=%zu  total=%zu  occ_after=%ld\n",
              sfloor.run(m, empty).size(), sceil.run(m, empty).size(), wall.run(m, empty).size(),
              occl.run(m, empty).size(), roof.run(m, empty).size(),
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
  auto wd = wall.run(sm, empty);

  // ---- 合成天花板(label 4)带观测洞 + free 开口, 覆盖 RoofCap ----
  std::vector<uint8_t> co(so.size(), 0), cs(so.size(), 0), cf(so.size(), 0);
  for (int x = 3; x <= 20; ++x)
    for (int z = 3; z <= 35; ++z) { co[sid(x, 20, z)] = 1; cs[sid(x, 20, z)] = 4; }
  for (int x = 10; x <= 12; ++x)
    for (int z = 10; z <= 12; ++z) { co[sid(x, 20, z)] = 0; cs[sid(x, 20, z)] = 0; }  // 观测洞
  for (int x = 15; x <= 16; ++x)
    for (int z = 15; z <= 16; ++z) { co[sid(x, 20, z)] = 0; cs[sid(x, 20, z)] = 0; cf[sid(x, 20, z)] = 1; }  // free 开口
  MapView cm{co.data(), cs.data(), cf.data(), SX, SY, SZ, 0.1f};
  auto rdel = roof.run(cm, empty);
  std::printf("C++ synth wall=%zu roof=%zu\n", wd.size(), rdel.size());

  auto dumpv = [&](const std::string& path, const std::vector<VoxelDelta>& v) {
    FILE* g = std::fopen(path.c_str(), "wb");
    int n = (int)v.size(); std::fwrite(&n, 4, 1, g);
    for (auto& d : v) { int a[5] = {d.idx[0], d.idx[1], d.idx[2], (int)d.op, d.sem}; std::fwrite(a, 4, 5, g); }
    std::fclose(g);
  };
  dumpv(O + "vlayer_synth_cpp.bin", wd);
  dumpv(O + "vlayer_roof_cpp.bin", rdel);

  // ---- 合成门洞: 墙平面 free 连通域 -> OpeningCarve 拟合并挖 remove ----
  // 在合成墙(x=12)上开 9x6 门(54 格 >= min_cells=35), 带 1 格毛边
  for (int y = 4; y <= 12; ++y)
    for (int z = 19; z <= 24; ++z) {
      so[sid(12, y, z)] = 0; ss[sid(12, y, z)] = 0; sf[sid(12, y, z)] = 1;
    }
  so[sid(12, 13, 21)] = 0; ss[sid(12, 13, 21)] = 0; sf[sid(12, 13, 21)] = 1;  // 毛边
  // 门洞内 2 块噪声残砖(拟合矩形会覆盖它们 -> remove)
  so[sid(12, 8, 21)] = 1; ss[sid(12, 8, 21)] = 19; sf[sid(12, 8, 21)] = 0;
  so[sid(12, 10, 22)] = 1; ss[sid(12, 10, 22)] = 19; sf[sid(12, 10, 22)] = 0;
  Overlay acc2;
  acc2.voxels = wall.run(sm, empty);   // 补墙格(门洞后重算)
  OpeningCarve oc;
  auto od = oc.run(sm, acc2);
  std::printf("C++ synth opening=%zu (acc wall adds=%zu)\n", od.size(), acc2.voxels.size());
  dumpv(O + "vlayer_open_cpp.bin", od);

  // ---- 合成阶梯: 3 级台阶(15)+ 地面(3), 向下实心化 ----
  std::vector<uint8_t> to(so.size(), 0), ts(so.size(), 0), tf(so.size(), 0);
  for (int x = 3; x <= 9; ++x)
    for (int z = 3; z <= 9; ++z) { to[sid(x, 2, z)] = 1; ts[sid(x, 2, z)] = 3; }
  for (int k = 0; k < 3; ++k)
    for (int z = 4; z <= 8; ++z) { to[sid(5 + k, 5 + 2 * k, z)] = 1; ts[sid(5 + k, 5 + 2 * k, z)] = 15; }
  MapView tm{to.data(), ts.data(), tf.data(), SX, SY, SZ, 0.1f};
  StairsFill st(1.5);
  auto sd = st.run(tm, empty);
  std::printf("C++ synth stairs=%zu\n", sd.size());
  dumpv(O + "vlayer_stairs_cpp.bin", sd);
  return 0;
}
