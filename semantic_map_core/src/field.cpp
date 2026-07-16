#include "semantic_map_core/field.hpp"

#include <cmath>
#include <queue>
#include <vector>

namespace smc {

namespace {
constexpr double BIG = 1e19;

inline long IDX(int i, int j, int k, int ny, int nz) {
  return (static_cast<long>(i) * ny + j) * nz + k;
}

// Felzenszwalb 1D 平方距离下包络 + 最近站点索引传播。
void edt1d(const double* f, const long* site_in, int n, double* d, long* site_out,
           std::vector<int>& v, std::vector<double>& z) {
  int k = 0;
  v[0] = 0;
  z[0] = -BIG;
  z[1] = BIG;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + (double)q * q) - (f[v[k]] + (double)v[k] * v[k])) /
               (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + (double)q * q) - (f[v[k]] + (double)v[k] * v[k])) /
          (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = BIG;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < (double)q) ++k;
    int p = v[k];
    d[q] = (double)(q - p) * (q - p) + f[p];
    site_out[q] = site_in[p];
  }
}
}  // namespace

void compute_esdf(const uint8_t* occ, int nx, int ny, int nz, float voxel_size,
                  std::vector<float>& dist, std::vector<long>& parent) {
  const long N = static_cast<long>(nx) * ny * nz;
  std::vector<double> f(N);
  std::vector<long> site(N);
  for (long id = 0; id < N; ++id) {
    if (occ[id]) { f[id] = 0.0; site[id] = id; }
    else { f[id] = BIG; site[id] = -1; }
  }

  const int nmax = std::max(nx, std::max(ny, nz));
  std::vector<double> fl(nmax), dl(nmax);
  std::vector<long> sl(nmax), sl2(nmax);
  std::vector<int> vv(nmax);
  std::vector<double> zz(nmax + 1);

  // pass X (vary i)
  for (int j = 0; j < ny; ++j)
    for (int k = 0; k < nz; ++k) {
      for (int i = 0; i < nx; ++i) { long id = IDX(i, j, k, ny, nz); fl[i] = f[id]; sl[i] = site[id]; }
      edt1d(fl.data(), sl.data(), nx, dl.data(), sl2.data(), vv, zz);
      for (int i = 0; i < nx; ++i) { long id = IDX(i, j, k, ny, nz); f[id] = dl[i]; site[id] = sl2[i]; }
    }
  // pass Y (vary j)
  for (int i = 0; i < nx; ++i)
    for (int k = 0; k < nz; ++k) {
      for (int j = 0; j < ny; ++j) { long id = IDX(i, j, k, ny, nz); fl[j] = f[id]; sl[j] = site[id]; }
      edt1d(fl.data(), sl.data(), ny, dl.data(), sl2.data(), vv, zz);
      for (int j = 0; j < ny; ++j) { long id = IDX(i, j, k, ny, nz); f[id] = dl[j]; site[id] = sl2[j]; }
    }
  // pass Z (vary k)
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j) {
      for (int k = 0; k < nz; ++k) { long id = IDX(i, j, k, ny, nz); fl[k] = f[id]; sl[k] = site[id]; }
      edt1d(fl.data(), sl.data(), nz, dl.data(), sl2.data(), vv, zz);
      for (int k = 0; k < nz; ++k) { long id = IDX(i, j, k, ny, nz); f[id] = dl[k]; site[id] = sl2[k]; }
    }

  dist.resize(N);
  parent.resize(N);
  for (long id = 0; id < N; ++id) {
    dist[id] = static_cast<float>(std::sqrt(f[id]) * voxel_size);
    parent[id] = site[id];
  }
}

std::vector<uint8_t> extract_gvd(const uint8_t* free, const float* dist,
                                 const long* parent, int nx, int ny, int nz,
                                 float voxel_size, float d_min, float theta_sep) {
  const long N = static_cast<long>(nx) * ny * nz;
  std::vector<uint8_t> gvd(N, 0);
  const double theta_vox2 = (double)(theta_sep / voxel_size) * (theta_sep / voxel_size);
  const long nynz = static_cast<long>(ny) * nz;
  auto punpack = [&](long p, long& px, long& py, long& pz) {
    px = p / nynz; long r = p % nynz; py = r / nz; pz = r % nz;
  };
  // 沿 3 轴比较相邻 free 格的父体素间距
  const long strides[3] = {nynz, (long)nz, 1};
  const int dims[3] = {nx, ny, nz};
  for (int ax = 0; ax < 3; ++ax) {
    long st = strides[ax];
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < ny; ++j)
        for (int k = 0; k < nz; ++k) {
          int coord = (ax == 0 ? i : ax == 1 ? j : k);
          if (coord + 1 >= dims[ax]) continue;
          long id = IDX(i, j, k, ny, nz);
          long id2 = id + st;
          if (!free[id] || !free[id2]) continue;
          long ax0, ay0, az0, bx0, by0, bz0;
          punpack(parent[id], ax0, ay0, az0);
          punpack(parent[id2], bx0, by0, bz0);
          if (parent[id] < 0 || parent[id2] < 0) continue;
          double dd = (double)(ax0 - bx0) * (ax0 - bx0) + (double)(ay0 - by0) * (ay0 - by0) +
                      (double)(az0 - bz0) * (az0 - bz0);
          if (dd >= theta_vox2) {
            if (dist[id] >= d_min) gvd[id] = 1;
            if (dist[id2] >= d_min) gvd[id2] = 1;
          }
        }
  }
  return gvd;
}

std::vector<uint8_t> denoise_occupancy(const uint8_t* occ, int nx, int ny, int nz,
                                       int min_component) {
  const long N = static_cast<long>(nx) * ny * nz;
  std::vector<uint8_t> out(N, 0);
  if (min_component <= 1) { for (long i = 0; i < N; ++i) out[i] = occ[i]; return out; }
  std::vector<int> label(N, 0);
  int cur = 0;
  std::vector<long> comp;
  for (long s = 0; s < N; ++s) {
    if (!occ[s] || label[s]) continue;
    ++cur;
    comp.clear();
    std::queue<long> q;
    q.push(s); label[s] = cur;
    while (!q.empty()) {
      long id = q.front(); q.pop();
      comp.push_back(id);
      int i = id / (static_cast<long>(ny) * nz);
      long r = id % (static_cast<long>(ny) * nz);
      int j = r / nz, k = r % nz;
      for (int di = -1; di <= 1; ++di)
        for (int dj = -1; dj <= 1; ++dj)
          for (int dk = -1; dk <= 1; ++dk) {
            if (!di && !dj && !dk) continue;
            int ni = i + di, nj = j + dj, nk = k + dk;
            if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;
            long nid = IDX(ni, nj, nk, ny, nz);
            if (occ[nid] && !label[nid]) { label[nid] = cur; q.push(nid); }
          }
    }
    if ((int)comp.size() >= min_component)
      for (long id : comp) out[id] = 1;
  }
  return out;
}

}  // namespace smc
