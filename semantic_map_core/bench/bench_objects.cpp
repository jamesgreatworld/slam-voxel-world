// bench_objects.cpp — C++ extract_objects(House), dump 供 Python 按集合对拍。
#include "semantic_map_core/objects.hpp"

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
  std::vector<uint8_t> occ(N), sem(N);
  { FILE* f = std::fopen((O + "house_occ.bin").c_str(), "rb"); rd = std::fread(occ.data(), 1, N, f); std::fclose(f);
    f = std::fopen((O + "house_sem.bin").c_str(), "rb"); rd = std::fread(sem.data(), 1, N, f); std::fclose(f); }
  std::set<int> structure;
  { FILE* f = std::fopen((O + "house_struct.bin").c_str(), "rb"); int m; rd = std::fread(&m, 4, 1, f);
    for (int i = 0; i < m; ++i) { int v; rd = std::fread(&v, 4, 1, f); structure.insert(v); } std::fclose(f); }
  (void)rd;

  auto objs = extract_objects(occ.data(), sem.data(), nx, ny, nz, structure, 20, 3.0, 5, 2);
  std::printf("C++  objects=%zu\n", objs.size());
  FILE* f = std::fopen((O + "house_obj_cpp.bin").c_str(), "wb");
  int no = (int)objs.size(); std::fwrite(&no, 4, 1, f);
  for (auto& o : objs) {
    int v[11] = {o.label, o.ci, o.cj, o.ck, o.voxel_count,
                 o.bmin[0], o.bmin[1], o.bmin[2], o.bmax[0], o.bmax[1], o.bmax[2]};
    std::fwrite(v, 4, 11, f);
  }
  std::fclose(f);
  return 0;
}
