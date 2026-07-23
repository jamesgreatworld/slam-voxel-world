// vlayer.cpp — 见 vlayer.hpp。移植自 overlay.py + pipeline.py。
#include "semantic_map_core/vlayer.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace smc {

static const char* kOpStr[3] = {"add", "remove", "replace"};

void Overlay::add_voxel(int x, int y, int z, int sem, const std::string& generator,
                        const std::string& binding, DeltaOp op) {
  if (sem < 0 || sem > 255) throw std::invalid_argument("sem out of uint8 range");
  VoxelDelta d;
  d.idx[0] = x; d.idx[1] = y; d.idx[2] = z;
  d.op = op; d.sem = sem; d.generator = generator; d.binding = binding;
  voxels.push_back(d);
}

bool Overlay::save(const std::string& path) const {
  std::string tmp = path + ".tmp";
  { std::ofstream f(tmp);
    if (!f) return false;
    f << "{\"voxels\":[\n";
    for (size_t i = 0; i < voxels.size(); ++i) {
      const VoxelDelta& d = voxels[i];
      if (i) f << ",\n";
      f << "  {\"idx\":[" << d.idx[0] << "," << d.idx[1] << "," << d.idx[2]
        << "],\"op\":\"" << kOpStr[(int)d.op] << "\",\"sem\":" << d.sem
        << ",\"generator\":\"" << d.generator << "\",\"binding\":\"" << d.binding << "\"}";
    }
    f << "\n]}\n";
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

Overlay Overlay::load(const std::string& path) {
  Overlay ov;
  std::ifstream f(path);
  if (!f) return ov;
  std::stringstream buf; buf << f.rdbuf();
  const std::string s = buf.str();
  size_t pos = 0;
  while (true) {
    size_t b = s.find("{\"idx\":", pos);
    if (b == std::string::npos) break;
    size_t e = s.find('}', b);
    if (e == std::string::npos) break;
    VoxelDelta d;
    { size_t a = s.find('[', b);
      std::sscanf(s.c_str() + a, "[%d,%d,%d]", &d.idx[0], &d.idx[1], &d.idx[2]); }
    { size_t o = s.find("\"op\":\"", b) + 6, oe = s.find('"', o);
      std::string op = s.substr(o, oe - o);
      d.op = op == "remove" ? DeltaOp::REMOVE : op == "replace" ? DeltaOp::REPLACE : DeltaOp::ADD; }
    { size_t sp = s.find("\"sem\":", b); d.sem = std::atoi(s.c_str() + sp + 6); }
    { size_t g = s.find("\"generator\":\"", b) + 13, ge = s.find('"', g);
      d.generator = s.substr(g, ge - g); }
    { size_t bd = s.find("\"binding\":\"", b) + 11, be = s.find('"', bd);
      d.binding = s.substr(bd, be - bd); }
    ov.voxels.push_back(d);
    pos = e + 1;
  }
  return ov;
}

void compose_structure(const MapView& m, const Overlay& overlay,
                       std::vector<uint8_t>& occ, std::vector<uint8_t>& sem) {
  const long N = (long)m.nx * m.ny * m.nz;
  occ.assign(m.occ, m.occ + N);
  sem.assign(m.sem, m.sem + N);
  for (const VoxelDelta& d : overlay.voxels) {
    int x = d.idx[0], y = d.idx[1], z = d.idx[2];
    if (x < 0 || x >= m.nx || y < 0 || y >= m.ny || z < 0 || z >= m.nz) continue;
    long id = m.id(x, y, z);
    if (d.op == DeltaOp::ADD) {
      bool authored = (d.generator == "manual" || d.binding == "independent");
      if (occ[id] && !authored) continue;  // completed 不覆盖观测
      occ[id] = 1; sem[id] = (uint8_t)d.sem;
    } else if (d.op == DeltaOp::REMOVE) {
      occ[id] = 0;
    } else {  // REPLACE
      if (occ[id]) sem[id] = (uint8_t)d.sem;
    }
  }
}

// ---- pipeline ----
std::vector<const Generator*> toposort(const std::vector<const Generator*>& gens) {
  std::vector<const Generator*> ordered, remaining = gens;
  auto in_remaining = [&](const std::string& gid) {
    for (auto* g : remaining) if (g->id == gid) return true;
    return false;
  };
  bool has_id[1] = {false}; (void)has_id;
  auto id_exists = [&](const std::string& gid) {
    for (auto* g : gens) if (g->id == gid) return true;
    return false;
  };
  while (!remaining.empty()) {
    std::vector<const Generator*> ready;  // 从 remaining 过滤, 保序
    for (auto* g : remaining) {
      bool ok = true;
      for (auto& dep : g->depends_on)
        if (id_exists(dep) && in_remaining(dep)) { ok = false; break; }
      if (ok) ready.push_back(g);
    }
    if (ready.empty()) throw std::runtime_error("generator dependency cycle");
    std::stable_sort(ready.begin(), ready.end(),
                     [](const Generator* a, const Generator* b) { return a->stage < b->stage; });
    const Generator* nxt = ready.front();
    ordered.push_back(nxt);
    remaining.erase(std::remove(remaining.begin(), remaining.end(), nxt), remaining.end());
  }
  return ordered;
}

Overlay run_pipeline(const MapView& m, const std::vector<const Generator*>& gens) {
  Overlay ov;
  for (const Generator* g : toposort(gens)) {
    auto ds = g->run(m, ov);  // 传入已累积 overlay
    ov.voxels.insert(ov.voxels.end(), ds.begin(), ds.end());
  }
  return ov;
}

}  // namespace smc
