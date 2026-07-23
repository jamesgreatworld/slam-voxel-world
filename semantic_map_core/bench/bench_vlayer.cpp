// bench_vlayer.cpp — vlayer 对拍: House occ/sem/free 上跑 SlabFill(floor+ceiling) 流水线,
// dump deltas + compose 后的 occ/sem 供 Python 逐格对拍。
#include "semantic_map_core/coarsen.hpp"
#include "semantic_map_core/vlayer.hpp"
#include "semantic_map_core/vlayer_generators.hpp"

// 统计各 generator 单独产出(诊断: 看哪些在 House 上非空)

#include <cmath>
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

  // ---- PlaneRegularize: 矩形外弱观测飘砖(删) + 矩形内深处未观测洞(补) ----
  so[sid(12, 22, 30)] = 1; ss[sid(12, 22, 30)] = 19;
  so[sid(12, 22, 31)] = 1; ss[sid(12, 22, 31)] = 19;   // 飘砖(弱 logodds)
  so[sid(12, 12, 30)] = 0; ss[sid(12, 12, 30)] = 0;    // 深处未观测洞
  so[sid(12, 12, 31)] = 0; ss[sid(12, 12, 31)] = 0;
  std::vector<float> lov(so.size(), 0.0f);
  for (size_t i = 0; i < so.size(); ++i) lov[i] = so[i] ? 3.5f : (sf[i] ? -2.0f : 0.0f);
  lov[sid(12, 22, 30)] = 0.9f; lov[sid(12, 22, 31)] = 0.9f;
  MapView rm{so.data(), ss.data(), sf.data(), SX, SY, SZ, 0.1f, lov.data()};
  PlaneRegularize pr;
  auto pd = pr.run(rm, empty);
  std::printf("C++ synth regularize=%zu\n", pd.size());
  dumpv(O + "vlayer_reg_cpp.bin", pd);

  // ---- RansacPlaneFill: 斜面带洞, trials 由 gen_ransac.py 注入 ----
  std::vector<uint8_t> po(so.size(), 0), ps(so.size(), 0), pf(so.size(), 0);
  for (int x = 2; x < 22; ++x)
    for (int z = 2; z < 30; ++z) {
      int y = (int)std::nearbyint(0.3 * x + 0.2 * z) + 2;
      if (x >= 8 && x <= 12 && z >= 10 && z <= 16) continue;
      if (y >= 0 && y < SY) { po[sid(x, y, z)] = 1; ps[sid(x, y, z)] = 4; }
    }
  std::vector<std::array<int, 3>> trials;
  { FILE* tf2 = std::fopen((O + "ransac_trials.bin").c_str(), "rb");
    if (tf2) {
      int tn = 0;
      if (std::fread(&tn, 4, 1, tf2) == 1)
        for (int i = 0; i < tn; ++i) {
          int v[3];
          if (std::fread(v, 4, 3, tf2) == 3) trials.push_back({v[0], v[1], v[2]});
        }
      std::fclose(tf2);
    } }
  MapView pm{po.data(), ps.data(), pf.data(), SX, SY, SZ, 0.1f};
  RansacPlaneFill rp;
  rp.trials = &trials;
  auto rpd = rp.run(pm, empty);
  std::printf("C++ synth ransac=%zu (trials=%zu)\n", rpd.size(), trials.size());
  dumpv(O + "ransac_cpp.bin", rpd);

  // ---- coarsen: House 真数据 downsample(factor2, 非平凡 vmin 测 mod 对齐) -> prune -> clean ----
  {
    const long vmn[3] = {-13, 5, -7};
    std::vector<uint8_t> oc, sc;
    long vmc[3]; float vsc; int dc[3];
    downsample_occupancy(occ.data(), sem.data(), nx, ny, nz, vmn, 0.1f, 2, 1, oc, sc, vmc, vsc, dc);
    FILE* f2 = std::fopen((O + "coarsen_cpp.bin").c_str(), "wb");
    int hdr[3] = {dc[0], dc[1], dc[2]};
    long vm64[3] = {vmc[0], vmc[1], vmc[2]};
    std::fwrite(hdr, 4, 3, f2);
    std::fwrite(vm64, 8, 3, f2);
    std::fwrite(oc.data(), 1, oc.size(), f2);
    std::fwrite(sc.data(), 1, sc.size(), f2);
    std::vector<uint8_t> po2 = oc, ps2 = sc;
    prune_dangles(po2, ps2, dc[0], dc[1], dc[2], 2);
    std::fwrite(po2.data(), 1, po2.size(), f2);
    std::fwrite(ps2.data(), 1, ps2.size(), f2);
    std::vector<uint8_t> co2 = oc, cs2 = sc;
    clean_coarse(co2, cs2, dc[0], dc[1], dc[2], 2, 1, false);
    std::fwrite(co2.data(), 1, co2.size(), f2);
    std::fwrite(cs2.data(), 1, cs2.size(), f2);
    std::fclose(f2);
    long n1 = 0, n2 = 0, n3 = 0;
    for (uint8_t v : oc) n1 += v;
    for (uint8_t v : po2) n2 += v;
    for (uint8_t v : co2) n3 += v;
    std::printf("C++ coarsen: down=%ld prune=%ld clean=%ld dims=%dx%dx%d\n",
                n1, n2, n3, dc[0], dc[1], dc[2]);
  }
  return 0;
}
