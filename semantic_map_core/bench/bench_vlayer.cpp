// bench_vlayer.cpp — vlayer 对拍: House occ/sem/free 上跑 SlabFill(floor+ceiling) 流水线,
// dump deltas + compose 后的 occ/sem 供 Python 逐格对拍。
#include "semantic_map_core/vlayer.hpp"
#include "semantic_map_core/vlayer_generators.hpp"

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
  std::vector<const Generator*> gens{&sfloor, &sceil};
  Overlay ov = run_pipeline(m, gens);

  std::vector<uint8_t> cocc, csem;
  compose_structure(m, ov, cocc, csem);

  long add = 0;
  for (auto& d : ov.voxels) if (d.op == DeltaOp::ADD) ++add;
  std::printf("C++ vlayer: deltas=%zu (add=%ld)  occ_after=%ld\n",
              ov.voxels.size(), add, [&] { long c = 0; for (uint8_t v : cocc) c += v; return c; }());

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
  return 0;
}
