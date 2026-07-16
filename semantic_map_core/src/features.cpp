// features.cpp — 见 features.hpp。ShapeFeature: cov=(centered^T centered)/M, eigvalsh(升序),
// sqrt(clip>=0)*vs 后 [::-1] 降序。3x3 对称特征值用 Jacobi(匹配 numpy eigvalsh)。
#include "semantic_map_core/features.hpp"

#include <algorithm>
#include <cmath>

namespace smc {

// 经典 Jacobi(每步消最大非对角), 对称 3x3, 返回升序特征值。
static std::array<double, 3> jacobi_eigvals(double m00, double m01, double m02,
                                            double m11, double m12, double m22) {
  double a[3][3] = {{m00, m01, m02}, {m01, m11, m12}, {m02, m12, m22}};
  for (int iter = 0; iter < 100; ++iter) {
    int p = 0, q = 1; double mx = std::fabs(a[0][1]);
    if (std::fabs(a[0][2]) > mx) { mx = std::fabs(a[0][2]); p = 0; q = 2; }
    if (std::fabs(a[1][2]) > mx) { mx = std::fabs(a[1][2]); p = 1; q = 2; }
    if (mx < 1e-300) break;
    double app = a[p][p], aqq = a[q][q], apq = a[p][q];
    double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
    double c = std::cos(phi), s = std::sin(phi);
    a[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
    a[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
    a[p][q] = a[q][p] = 0.0;
    int r = 3 - p - q;
    double arp = a[r][p], arq = a[r][q];
    a[r][p] = a[p][r] = c * arp - s * arq;
    a[r][q] = a[q][r] = s * arp + c * arq;
    double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
    if (off < 1e-300) break;
  }
  std::array<double, 3> ev = {a[0][0], a[1][1], a[2][2]};
  std::sort(ev.begin(), ev.end());  // 升序, 同 numpy eigvalsh
  return ev;
}

std::array<double, 3> shape_feature(const std::vector<std::array<int, 3>>& cells, double vs) {
  const int M = (int)cells.size();
  if (M < 3) return {0.0, 0.0, 0.0};
  double mx = 0, my = 0, mz = 0;
  for (auto& c : cells) { mx += c[0]; my += c[1]; mz += c[2]; }
  mx /= M; my /= M; mz /= M;
  double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
  for (auto& c : cells) {
    double dx = c[0] - mx, dy = c[1] - my, dz = c[2] - mz;
    c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
    c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
  }
  c00 /= M; c01 /= M; c02 /= M; c11 /= M; c12 /= M; c22 /= M;
  auto ev = jacobi_eigvals(c00, c01, c02, c11, c12, c22);  // 升序
  std::array<double, 3> out;
  for (int i = 0; i < 3; ++i) {  // ev[::-1] 降序
    double e = ev[2 - i];
    if (e < 0) e = 0;
    out[i] = std::sqrt(e) * vs;
  }
  return out;
}

double feature_cost(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  double s = 0;
  for (int i = 0; i < 3; ++i) { double d = a[i] - b[i]; s += d * d; }
  return 2.0 * std::sqrt(s);
}

}  // namespace smc
