// smc_live_node.cpp — 自研 Hydra C++ 在线驱动节点。
// 复刻 live_ros_stream.py(订阅 depth/seg/rgb + TF -> ObsMap 逐帧积分)
//   + gvd_live.py(周期线程: crop->denoise->esdf->gvd->thin->graph->prune/merge/drop
//                 ->rooms->nested->objects->link->merge_observation -> /dsg MarkerArray)。
// 参数与 VM 上 Python 实跑一致: stride6, bounds -20,-12,-3,8,10,5, interval 10s。
#include "semantic_map_core/field.hpp"
#include "semantic_map_core/graph.hpp"
#include "semantic_map_core/objects.hpp"
#include "semantic_map_core/obsmap.hpp"
#include "semantic_map_core/scene_graph.hpp"
#include "semantic_map_core/surface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using namespace smc;
using Clk = std::chrono::steady_clock;
static double ms_between(Clk::time_point a, Clk::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

// ---------- 配置(与 Python 实跑一致) ----------
static const char* kDepthTopic = "/tg/depth";
static const char* kSegTopic = "/tg/semantic";
static const char* kRgbTopic = "/tg/rgb";
static const char* kInfoTopic = "/tg/camera_info";
static const char* kMapFrame = "odom";
static const int kStride = 6;
static const double kDepthMin = 0.2, kDepthMax = 15.0, kFreeMargin = 0.10;
static const double kVs = 0.1;  // double, 与 Python voxel_size 同精度
static const double kBounds[6] = {-20, -12, -3, 8, 10, 5};  // ROS z-up
static const double kCycleInterval = 10.0;
static const char* kLabelspace =
    "/home/james/Semantic_map_ws/src/Hydra/config/label_spaces/tartanground_house_label_space.yaml";
// nav 2D 前端(nav_frontend.py 同参): 平面范围/分层
static const double kNavX0 = -16, kNavY0 = -9, kNavX1 = 5, kNavY1 = 8;
static const int kNavNX = 210, kNavNY = 170;  // ceil((x1-x0)/vs), ceil((y1-y0)/vs)
static const double kRobotH = 0.5, kGroundEps = 0.08, kLevelGap = 0.8;
static const int kLevelMinPts = 200, kMaxLevels = 4;
// 地图存档目录(占据图+DSG, 启动时存在则恢复)
static const char* kMapDir = "/home/james/semantic_map_core/maps/house_live";
static const char* kStructKw[] = {"wall", "ceiling", "roof", "building", "floor", "ground",
                                  "carpet", "rug", "pillar", "column", "beam", "stair", "sky"};

static const float ROOM_COLORS[12][3] = {
    {1.0f, .31f, .31f}, {.31f, 1.0f, .31f}, {.31f, .31f, 1.0f}, {1.0f, 1.0f, .31f},
    {1.0f, .31f, 1.0f}, {.31f, 1.0f, 1.0f}, {1.0f, .63f, .16f}, {.63f, .31f, 1.0f},
    {.16f, 1.0f, .63f}, {1.0f, .47f, .63f}, {.63f, 1.0f, .31f}, {.47f, .63f, 1.0f}};

// vxw(y-up) -> ROS(z-up): (x,y,z)_ros = (-Z, -X, Y)_vxw
static inline void yup2ros(double x, double y, double z, double& rx, double& ry, double& rz) {
  rx = -z; ry = -x; rz = y;
}

// labelspace yaml 解析: label 名字表 + 结构类 + surface 类 + invalid 类
struct LabelSpace {
  std::map<int, std::string> names;
  std::set<int> structure;   // 名含 STRUCT_KW -> 不算物体
  std::set<int> surface;     // surface_places_labels(地面类, nav 用)
  std::set<int> invalid;     // invalid_labels
};
static std::vector<int> parse_int_list(const std::string& s) {
  std::vector<int> out;
  int v = 0; bool in = false;
  for (char c : s) {
    if (c >= '0' && c <= '9') { v = v * 10 + (c - '0'); in = true; }
    else { if (in) out.push_back(v); v = 0; in = false; }
  }
  if (in) out.push_back(v);
  return out;
}
static LabelSpace load_labelspace(const std::string& path) {
  LabelSpace ls;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.find("surface_places_labels:") != std::string::npos) {
      for (int v : parse_int_list(line.substr(line.find('[')))) ls.surface.insert(v);
      continue;
    }
    if (line.find("invalid_labels:") != std::string::npos) {
      for (int v : parse_int_list(line.substr(line.find('[')))) ls.invalid.insert(v);
      continue;
    }
    auto lp = line.find("label:"); auto np = line.find("name:");
    if (lp == std::string::npos || np == std::string::npos) continue;
    int id = std::atoi(line.c_str() + lp + 6);
    std::string name = line.substr(np + 5);
    while (!name.empty() && (name.back() == '}' || name.back() == ' ' || name.back() == '\r'))
      name.pop_back();
    while (!name.empty() && name.front() == ' ') name.erase(name.begin());
    ls.names[id] = name;
    std::string low = name;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    for (const char* kw : kStructKw)
      if (low.find(kw) != std::string::npos) { ls.structure.insert(id); break; }
  }
  if (ls.invalid.empty()) ls.invalid.insert(0);
  return ls;
}

// 黄金角 HSV 配色(与 Python _label_color/room_color 同式)
static void golden_color(int key, double sat, double val, float& r, float& g, float& b) {
  double h = std::fmod(key * 137.508, 360.0) / 360.0;
  int hi = (int)(h * 6.0) % 6;
  double f = h * 6.0 - (int)(h * 6.0);
  double p = val * (1 - sat), q = val * (1 - f * sat), t = val * (1 - (1 - f) * sat);
  double rr, gg, bb;
  switch (hi) {
    case 0: rr = val; gg = t; bb = p; break;
    case 1: rr = q; gg = val; bb = p; break;
    case 2: rr = p; gg = val; bb = t; break;
    case 3: rr = p; gg = q; bb = val; break;
    case 4: rr = t; gg = p; bb = val; break;
    default: rr = val; gg = p; bb = q; break;
  }
  r = (float)rr; g = (float)gg; b = (float)bb;
}

// _surface_tiles 多楼层版: 每个楼层地板高度上方 clearance_vox 层全 free 的 (X,Z) 列
// -> (瓦片中心 vxw 坐标, 层号)。floors_vox 为空时退回单层(占据 y 3% 分位)。
static std::vector<std::array<double, 4>> surface_tiles(
    const std::vector<uint8_t>& free, const std::vector<uint8_t>& occ, int nx, int ny, int nz,
    const long vmin[3], double vs, std::vector<int> floors_vox, int clearance_vox = 4) {
  std::vector<std::array<double, 4>> out;
  if (floors_vox.empty()) {  // 单层回退: 3% 分位(np.percentile 线性插值后截断)
    std::vector<int> ys;
    for (long id = 0; id < (long)nx * ny * nz; ++id)
      if (occ[id]) ys.push_back((int)((id / nz) % ny));
    if (ys.empty()) return out;
    std::sort(ys.begin(), ys.end());
    double q = 0.03 * (ys.size() - 1);
    size_t lo = (size_t)q;
    double frac = q - lo;
    double val = ys[lo] + (lo + 1 < ys.size() ? frac * (ys[lo + 1] - ys[lo]) : 0.0);
    floors_vox.push_back((int)val);
  }
  for (size_t lvl = 0; lvl < floors_vox.size(); ++lvl) {
    int y_floor = floors_vox[lvl];
    int y0 = y_floor + 1, y1 = y_floor + 1 + clearance_vox;
    if (y0 < 0) y0 = 0;
    if (y1 > ny) y1 = ny;
    for (int i = 0; i < nx; ++i)
      for (int k = 0; k < nz; ++k) {
        bool all = (y1 > y0);
        for (int j = y0; j < y1 && all; ++j)
          if (!free[((long)i * ny + j) * nz + k]) all = false;
        if (all)
          out.push_back({(i + vmin[0] + 0.5) * vs, (y_floor + vmin[1] + 1.0) * vs,
                         (k + vmin[2] + 0.5) * vs, (double)lvl});
      }
  }
  return out;
}

class SmcLiveNode : public rclcpp::Node {
 public:
  SmcLiveNode()
      : Node("smc_live_node"),
        tfbuf_(get_clock(), tf2::durationFromSec(60.0)),
        tflis_(tfbuf_) {
    // vxw 网格: ROS(x,y,z) -> vxw(x,z,y)
    vmin_ = {(long)std::floor(kBounds[0] / kVs), (long)std::floor(kBounds[2] / kVs),
             (long)std::floor(kBounds[1] / kVs)};
    nx_ = (int)std::ceil((kBounds[3] - kBounds[0]) / kVs);
    ny_ = (int)std::ceil((kBounds[5] - kBounds[2]) / kVs);
    nz_ = (int)std::ceil((kBounds[4] - kBounds[1]) / kVs);
    obs_ = std::make_unique<ObsMap>(nx_, ny_, nz_, vmin_, (float)kVs);
    ls_ = load_labelspace(kLabelspace);
    structure_ids_ = ls_.structure;
    RCLCPP_INFO(get_logger(), "grid %dx%dx%d vmin=(%ld,%ld,%ld) labels=%zu structure=%zu surface=%zu",
                nx_, ny_, nz_, vmin_[0], vmin_[1], vmin_[2], ls_.names.size(),
                structure_ids_.size(), ls_.surface.size());

    info_sub_ = create_subscription<CameraInfo>(
        kInfoTopic, 10, [this](CameraInfo::ConstSharedPtr m) {
          if (!have_k_) {
            fx_ = m->k[0]; fy_ = m->k[4]; cx_ = m->k[2]; cy_ = m->k[5];
            have_k_ = true;
            RCLCPP_INFO(get_logger(), "camera_info: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
          }
        });
    depth_sub_.subscribe(this, kDepthTopic);
    seg_sub_.subscribe(this, kSegTopic);
    rgb_sub_.subscribe(this, kRgbTopic);
    sync_ = std::make_unique<Sync>(Policy(30), depth_sub_, seg_sub_, rgb_sub_);
    sync_->registerCallback(&SmcLiveNode::on_frame, this);
    auto latch = rclcpp::QoS(1).transient_local();
    dsg_pub_ = create_publisher<MarkerArray>("/dsg", latch);
    vox_pub_ = create_publisher<MarkerArray>("/semantic_voxels", latch);
    // SceneGraph 元素分话题(rviz 可单独开关)
    obj_pub_ = create_publisher<MarkerArray>("/dsg/objects", latch);
    room_pub_ = create_publisher<MarkerArray>("/dsg/rooms", latch);
    place_pub_ = create_publisher<MarkerArray>("/dsg/places", latch);
    gvd_pub_ = create_publisher<MarkerArray>("/dsg/gvd", latch);
    surf_pub_ = create_publisher<MarkerArray>("/dsg/surface", latch);
    sp_pub_ = create_publisher<MarkerArray>("/nav/surface_places", latch);
    if (load_map()) {
      RCLCPP_INFO(get_logger(), "已恢复存档: %s (DSG nodes=%zu, id_counter=%d, nav层=%zu)",
                  kMapDir, sg_.order.size(), id_counter_, levels_.size());
      publish_nav();     // 恢复后立即发一次代价地图
      publish_voxels();  // 恢复后立即发一次语义体素
    }
    cycle_thread_ = std::thread([this] { cycle_loop(); });
  }

  ~SmcLiveNode() override {
    running_ = false;
    if (cycle_thread_.joinable()) cycle_thread_.join();
    dump_stats();
  }

  void dump_stats() {
    std::ofstream f("/mnt/hgfs/Shared/claude_jobs/cpp_node_stats.txt");
    f << "integrated=" << n_int_ << " skipped_tf=" << n_skip_ << "\n";
    f << "integrate_ms mean=" << (n_int_ ? int_ms_sum_ / n_int_ : 0) << " max=" << int_ms_max_ << "\n";
    f << "cycles=" << n_cycle_ << " cycle_ms mean=" << (n_cycle_ ? cyc_ms_sum_ / n_cycle_ : 0)
      << " max=" << cyc_ms_max_ << "\n";
    f << "last_dsg rooms=" << last_rooms_ << " places=" << last_places_
      << " objects=" << last_objects_ << "\n";
    f << "merge_totals matched=" << tot_matched_ << " added=" << tot_added_
      << " removed=" << tot_removed_ << " carried=" << tot_carried_ << "\n";
  }

 private:
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image, Image>;
  using Sync = message_filters::Synchronizer<Policy>;

  void on_frame(Image::ConstSharedPtr d, Image::ConstSharedPtr s, Image::ConstSharedPtr r) {
    pending_.push_back({Clk::now(), d, s, r});
    std::deque<Pending> still;
    for (auto& p : pending_) {
      geometry_msgs::msg::TransformStamped tr;
      try {
        std::string src = p.depth->header.frame_id.empty() ? "cam_optical" : p.depth->header.frame_id;
        tr = tfbuf_.lookupTransform(kMapFrame, src, p.depth->header.stamp);
      } catch (const std::exception& e) {
        if (ms_between(p.t_in, Clk::now()) < 5000.0) still.push_back(p);
        else {
          ++n_skip_;
          if (n_skip_ % 25 == 1)
            RCLCPP_WARN(get_logger(), "丢帧 %ld (TF 5s 未就绪): %s", n_skip_, e.what());
        }
        continue;
      }
      integrate(*p.depth, *p.seg, tr);
    }
    pending_ = std::move(still);
    while (pending_.size() > 50) pending_.pop_front();
  }

  void integrate(const Image& dm, const Image& sm, const geometry_msgs::msg::TransformStamped& tr) {
    if (!have_k_) return;
    if (dm.encoding != "32FC1" || sm.encoding != "mono8") {
      RCLCPP_ERROR(get_logger(), "encoding mismatch: %s/%s", dm.encoding.c_str(), sm.encoding.c_str());
      return;
    }
    auto t0 = Clk::now();
    const double qx = tr.transform.rotation.x, qy = tr.transform.rotation.y,
                 qz = tr.transform.rotation.z, qw = tr.transform.rotation.w;
    double R[9] = {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
                   2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
                   2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)};
    const double tx = tr.transform.translation.x, ty = tr.transform.translation.y,
                 tz = tr.transform.translation.z;

    const int H = dm.height, W = dm.width;
    const float* depth = reinterpret_cast<const float*>(dm.data.data());
    const uint8_t* seg = sm.data.data();
    const double fx = fx_ / kStride, fy = fy_ / kStride, cx = cx_ / kStride, cy = cy_ / kStride;

    std::vector<float> pts;
    std::vector<uint8_t> labels;
    pts.reserve(20000 * 3);
    std::vector<std::array<double, 3>> nav_surf, nav_obst;  // (gx,gy,z) nav 平面点
    int su = 0;
    for (int v = 0; v < H; v += kStride) {
      int uu = 0;
      for (int u = 0; u < W; u += kStride, ++uu) {
        float z = depth[(long)v * W + u];
        if (!(z > kDepthMin && z < kDepthMax) || !std::isfinite(z)) continue;
        double X = (uu - cx) * z / fx;   // 光学系(x右 y下 z前), strided 像素坐标
        double Y = (su - cy) * z / fy;
        double wx = R[0] * X + R[1] * Y + R[2] * z + tx;   // ROS odom
        double wy = R[3] * X + R[4] * Y + R[5] * z + ty;
        double wz = R[6] * X + R[7] * Y + R[8] * z + tz;
        // ROS -> vxw: X=-y, Y=z, Z=-x
        pts.push_back((float)(-wy)); pts.push_back((float)wz); pts.push_back((float)(-wx));
        int lab = seg[(long)v * W + u];
        labels.push_back((uint8_t)lab);
        // nav 2D 层(nav_frontend 逻辑): surface 类 / 障碍类分拣
        int gx = (int)std::floor((wx - kNavX0) / kVs), gy = (int)std::floor((wy - kNavY0) / kVs);
        if (gx >= 0 && gx < kNavNX && gy >= 0 && gy < kNavNY) {
          if (ls_.surface.count(lab)) nav_surf.push_back({(double)gx, (double)gy, wz});
          else if (!ls_.invalid.count(lab)) nav_obst.push_back({(double)gx, (double)gy, wz});
        }
      }
      ++su;
    }
    if (pts.empty()) return;
    nav_feed(nav_surf, nav_obst);
    if (n_int_ % 25 == 0) {
      std::lock_guard<std::mutex> lk(nav_mtx_);
      RCLCPP_INFO(get_logger(), "nav: surf_pts=%zu obst_pts=%zu levels=%zu",
                  nav_surf.size(), nav_obst.size(), levels_.size());
    }
    float origin[3] = {(float)(-ty), (float)tz, (float)(-tx)};
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      obs_->integrate_frame(origin, pts.data(), (int)labels.size(), labels.data(), (float)kFreeMargin);
    }
    ++n_int_;
    double ms = ms_between(t0, Clk::now());
    int_ms_sum_ += ms;
    if (ms > int_ms_max_) int_ms_max_ = ms;
    if (n_int_ % 25 == 0)
      RCLCPP_INFO(get_logger(), "integrated=%ld skipped_tf=%ld pts=%zu int_ms(mean=%.1f max=%.1f)",
                  n_int_, n_skip_, labels.size(), int_ms_sum_ / n_int_, int_ms_max_);
    if (n_int_ % 10 == 0) publish_voxels();  // 增量语义体素可视化
  }

  // 语义体素 CUBE_LIST(复刻 live_ros_stream._publish_voxels: 黄金角配色, 结构类半透明)
  void publish_voxels() {
    std::vector<uint8_t> occ, sem;
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      occ = obs_->occupancy_mask();
      sem = obs_->sem_label();
    }
    Marker m;
    m.header.frame_id = kMapFrame;
    m.header.stamp = now();
    m.ns = "semantic_voxels"; m.id = 0;
    m.type = Marker::CUBE_LIST; m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = kVs * 0.95;
    std::map<int, std_msgs::msg::ColorRGBA> lut;
    for (int i = 0; i < nx_; ++i)
      for (int j = 0; j < ny_; ++j)
        for (int k = 0; k < nz_; ++k) {
          long id = ((long)i * ny_ + j) * nz_ + k;
          if (!occ[id]) continue;
          double vx = (i + vmin_[0] + 0.5) * kVs, vy = (j + vmin_[1] + 0.5) * kVs,
                 vz = (k + vmin_[2] + 0.5) * kVs;
          geometry_msgs::msg::Point p;
          double rx, ry, rz; yup2ros(vx, vy, vz, rx, ry, rz);
          p.x = rx; p.y = ry; p.z = rz;
          m.points.push_back(p);
          int l = sem[id];
          auto it = lut.find(l);
          if (it == lut.end()) {
            std_msgs::msg::ColorRGBA c;
            if (l == 0) { c.r = c.g = c.b = 0.55f; c.a = 0.15f; }
            else {
              double h = std::fmod(l * 137.508, 360.0) / 360.0;  // 黄金角色相
              double s = 0.75, v = 0.95;
              int hi = (int)(h * 6.0) % 6;
              double f = h * 6.0 - (int)(h * 6.0);
              double pq = v * (1 - s), qq = v * (1 - f * s), tq = v * (1 - (1 - f) * s);
              double r, g, b;
              switch (hi) {
                case 0: r = v; g = tq; b = pq; break;
                case 1: r = qq; g = v; b = pq; break;
                case 2: r = pq; g = v; b = tq; break;
                case 3: r = pq; g = qq; b = v; break;
                case 4: r = tq; g = pq; b = v; break;
                default: r = v; g = pq; b = qq; break;
              }
              c.r = (float)r; c.g = (float)g; c.b = (float)b;
              c.a = structure_ids_.count(l) ? 0.28f : 1.0f;  // 结构类半透明(透墙看物体)
            }
            it = lut.emplace(l, c).first;
          }
          m.colors.push_back(it->second);
        }
    if (m.points.empty()) return;
    MarkerArray arr;
    arr.markers.push_back(m);
    vox_pub_->publish(arr);
  }

  // ---------- nav 2D 多楼层前端(nav_frontend.py 逻辑) ----------
  struct NavLevel {
    std::vector<int> surf, obst;   // (kNavNY*kNavNX) 行主序 gy*NX+gx
    double z_sum; long z_cnt;
    NavLevel(double rep) : surf((size_t)kNavNY * kNavNX, 0), obst((size_t)kNavNY * kNavNX, 0),
                           z_sum(rep), z_cnt(1) {}
    double rep_z() const { return z_sum / z_cnt; }
  };

  void nav_feed(const std::vector<std::array<double, 3>>& surf_pts,
                const std::vector<std::array<double, 3>>& obst_pts) {
    std::lock_guard<std::mutex> lk(nav_mtx_);
    // surface 点: 就近层(|z-rep|<=gap), 剩余最低簇开新层(需 >=min_pts)
    std::vector<char> assigned(surf_pts.size(), 0);
    for (int iter = 0; iter <= kMaxLevels; ++iter) {
      bool any_left = false;
      for (size_t i = 0; i < surf_pts.size(); ++i) {
        if (assigned[i]) continue;
        double z = surf_pts[i][2];
        int best = -1; double bd = 1e18;
        for (size_t k = 0; k < levels_.size(); ++k) {
          double d = std::fabs(z - levels_[k].rep_z());
          if (d < bd) { bd = d; best = (int)k; }
        }
        if (best >= 0 && bd <= kLevelGap) {
          NavLevel& lv = levels_[best];
          long flat = (long)surf_pts[i][1] * kNavNX + (long)surf_pts[i][0];
          lv.surf[flat]++;
          lv.z_sum += z; lv.z_cnt++;
          assigned[i] = 1;
        } else any_left = true;
      }
      if (!any_left || (int)levels_.size() >= kMaxLevels) break;
      // 开新层: 未分配点的最低簇
      std::vector<double> zr;
      for (size_t i = 0; i < surf_pts.size(); ++i) if (!assigned[i]) zr.push_back(surf_pts[i][2]);
      if ((int)zr.size() < kLevelMinPts) break;
      double zmin = *std::min_element(zr.begin(), zr.end());
      std::vector<double> band;
      for (double z : zr) if (z <= zmin + kLevelGap) band.push_back(z);
      if ((int)band.size() < kLevelMinPts) break;
      std::nth_element(band.begin(), band.begin() + band.size() / 2, band.end());
      levels_.emplace_back(band[band.size() / 2]);   // rep = median
      std::sort(levels_.begin(), levels_.end(),
                [](const NavLevel& a, const NavLevel& b) { return a.rep_z() < b.rep_z(); });
    }
    // 障碍点: 落在某层 [rep+eps, rep+robot_h] 高度带 -> 该层挡路
    for (auto& p : obst_pts) {
      long flat = (long)p[1] * kNavNX + (long)p[0];
      for (auto& lv : levels_) {
        double rz = lv.rep_z();
        if (p[2] > rz + kGroundEps && p[2] < rz + kRobotH) lv.obst[flat]++;
      }
    }
    if (n_int_ % 15 == 0) publish_nav_locked();
  }

  void publish_nav() {
    std::lock_guard<std::mutex> lk(nav_mtx_);
    publish_nav_locked();
  }

  void publish_nav_locked() {
    MarkerArray arr;
    Marker wipe; wipe.action = Marker::DELETEALL; arr.markers.push_back(wipe);
    int mid = 0;
    for (size_t k = 0; k < levels_.size(); ++k) {
      NavLevel& lv = levels_[k];
      std::vector<uint8_t> walkable; std::vector<int8_t> cost;
      occupancy_from_counts(lv.surf.data(), lv.obst.data(), kNavNY * kNavNX, 1, 2, walkable, cost);
      std::vector<uint8_t> blocked((size_t)kNavNY * kNavNX);
      for (size_t i = 0; i < blocked.size(); ++i) blocked[i] = lv.obst[i] >= 2 ? 1 : 0;
      // OccupancyGrid(origin.z = 层高 -> rviz 按真实高度堆叠)
      if (k >= navmap_pubs_.size())
        navmap_pubs_.push_back(create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/nav/surface_map_L" + std::to_string(k), rclcpp::QoS(1).transient_local()));
      nav_msgs::msg::OccupancyGrid gmap;
      gmap.header.frame_id = kMapFrame;
      gmap.header.stamp = now();
      gmap.info.resolution = (float)kVs;
      gmap.info.width = kNavNX; gmap.info.height = kNavNY;
      gmap.info.origin.position.x = kNavX0;
      gmap.info.origin.position.y = kNavY0;
      gmap.info.origin.position.z = lv.rep_z();
      gmap.info.origin.orientation.w = 1.0;
      gmap.data.assign(cost.begin(), cost.end());
      navmap_pubs_[k]->publish(gmap);
      // surface place 区域(球+id/面积文字+经门邻接边)
      auto res = cluster_surface_places(walkable.data(), kNavNY, kNavNX, kNavX0, kNavY0, kVs,
                                        15, 0.9, blocked.data());
      double zc = lv.rep_z() + 0.15;
      std::map<int, std::pair<double, double>> cen;
      for (auto& r : res.regions) {
        cen[r.id] = {r.cx, r.cy};
        Marker sph;
        sph.header.frame_id = kMapFrame; sph.header.stamp = now();
        sph.ns = "L" + std::to_string(k) + "_nodes"; sph.id = mid++;
        sph.type = Marker::SPHERE; sph.action = Marker::ADD;
        sph.pose.orientation.w = 1.0;
        sph.pose.position.x = r.cx; sph.pose.position.y = r.cy; sph.pose.position.z = zc;
        sph.scale.x = sph.scale.y = sph.scale.z = 0.25;
        golden_color((int)(r.id + k * 40), 0.55, 1.0, sph.color.r, sph.color.g, sph.color.b);
        sph.color.a = 0.95f;
        arr.markers.push_back(sph);
        Marker txt = sph;
        txt.ns = "L" + std::to_string(k) + "_labels"; txt.id = mid++;
        txt.type = Marker::TEXT_VIEW_FACING;
        txt.pose.position.z = zc + 0.3;
        txt.scale.z = 0.20;
        char buf[48];
        std::snprintf(buf, sizeof(buf), "L%zu.S%d %.1fm2", k, r.id, r.area_m2);
        txt.text = buf;
        txt.color.r = txt.color.g = txt.color.b = 1.0f; txt.color.a = 1.0f;
        arr.markers.push_back(txt);
      }
      Marker eln;
      eln.header.frame_id = kMapFrame; eln.header.stamp = now();
      eln.ns = "L" + std::to_string(k) + "_edges"; eln.id = mid++;
      eln.type = Marker::LINE_LIST; eln.action = Marker::ADD;
      eln.pose.orientation.w = 1.0; eln.scale.x = 0.03;
      for (auto& e : res.edges) {
        geometry_msgs::msg::Point a, b;
        a.x = cen[e.first].first; a.y = cen[e.first].second; a.z = zc;
        b.x = cen[e.second].first; b.y = cen[e.second].second; b.z = zc;
        eln.points.push_back(a); eln.points.push_back(b);
        std_msgs::msg::ColorRGBA c; c.r = 0.2f; c.g = 1.0f; c.b = 0.4f; c.a = 0.9f;
        eln.colors.push_back(c); eln.colors.push_back(c);
      }
      arr.markers.push_back(eln);
    }
    if (arr.markers.size() > 1) sp_pub_->publish(arr);
  }

  // ---------- 地图存档/恢复(占据图 + DSG + id 计数) ----------
  void save_map() {
    std::string dir(kMapDir);
    std::ofstream mk_dir;  // 确保目录存在
    (void)system(("mkdir -p " + dir).c_str());
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      obs_->save(dir + "/obsmap.bin");
    }
    {  // nav 楼层栅格(代价地图状态)
      std::lock_guard<std::mutex> lk(nav_mtx_);
      FILE* f = std::fopen((dir + "/nav.bin").c_str(), "wb");
      if (f) {
        int nl = (int)levels_.size();
        std::fwrite(&nl, 4, 1, f);
        for (auto& lv : levels_) {
          std::fwrite(&lv.z_sum, 8, 1, f);
          std::fwrite(&lv.z_cnt, 8, 1, f);
          std::fwrite(lv.surf.data(), 4, lv.surf.size(), f);
          std::fwrite(lv.obst.data(), 4, lv.obst.size(), f);
        }
        std::fclose(f);
      }
    }
    if (have_sg_) {
      std::ofstream f(dir + "/dsg.txt");
      f << "id_counter " << id_counter_ << "\n";
      for (auto& id : sg_.order) {
        SceneNode& n = sg_.nodes[id];
        f << n.id << "\t" << n.layer << "\t" << (n.parent.empty() ? "-" : n.parent) << "\t"
          << n.pos[0] << " " << n.pos[1] << " " << n.pos[2] << "\t" << n.obj_class << " "
          << n.voxel_count << " " << n.place_id << " " << n.misses << " " << n.seen_count << "\t"
          << n.bmin[0] << " " << n.bmin[1] << " " << n.bmin[2] << " "
          << n.bmax[0] << " " << n.bmax[1] << " " << n.bmax[2] << "\t"
          << n.bmin_m[0] << " " << n.bmin_m[1] << " " << n.bmin_m[2] << " "
          << n.bmax_m[0] << " " << n.bmax_m[1] << " " << n.bmax_m[2] << "\t"
          << n.feat[0] << " " << n.feat[1] << " " << n.feat[2] << "\n";
      }
    }
  }

  bool load_map() {
    std::string dir(kMapDir);
    auto m = ObsMap::load(dir + "/obsmap.bin");
    if (!m) return false;
    if (m->nx() != nx_ || m->ny() != ny_ || m->nz() != nz_) {
      RCLCPP_WARN(get_logger(), "存档网格尺寸不符, 忽略存档");
      return false;
    }
    obs_ = std::move(m);
    {  // nav 楼层栅格恢复
      FILE* nf = std::fopen((dir + "/nav.bin").c_str(), "rb");
      if (nf) {
        int nl = 0;
        if (std::fread(&nl, 4, 1, nf) == 1 && nl >= 0 && nl <= kMaxLevels) {
          std::lock_guard<std::mutex> lk(nav_mtx_);
          levels_.clear();
          for (int i = 0; i < nl; ++i) {
            NavLevel lv(0);
            size_t rd = std::fread(&lv.z_sum, 8, 1, nf);
            rd += std::fread(&lv.z_cnt, 8, 1, nf);
            rd += std::fread(lv.surf.data(), 4, lv.surf.size(), nf);
            rd += std::fread(lv.obst.data(), 4, lv.obst.size(), nf);
            (void)rd;
            levels_.push_back(std::move(lv));
          }
        }
        std::fclose(nf);
      }
    }
    std::ifstream f(dir + "/dsg.txt");
    if (f) {
      std::string line;
      if (std::getline(f, line)) {
        std::istringstream ss(line); std::string tag; ss >> tag >> id_counter_;
      }
      sg_ = SceneGraph{};
      std::vector<std::pair<std::string, std::string>> parents;
      while (std::getline(f, line)) {
        std::istringstream ss(line);
        SceneNode n;
        std::string parent, seg;
        std::getline(ss, n.id, '\t');
        std::getline(ss, n.layer, '\t');
        std::getline(ss, parent, '\t');
        std::getline(ss, seg, '\t');
        { std::istringstream v(seg); v >> n.pos[0] >> n.pos[1] >> n.pos[2]; }
        std::getline(ss, seg, '\t');
        { std::istringstream v(seg); v >> n.obj_class >> n.voxel_count >> n.place_id >> n.misses >> n.seen_count; }
        std::getline(ss, seg, '\t');
        { std::istringstream v(seg); v >> n.bmin[0] >> n.bmin[1] >> n.bmin[2] >> n.bmax[0] >> n.bmax[1] >> n.bmax[2]; }
        std::getline(ss, seg, '\t');
        { std::istringstream v(seg); v >> n.bmin_m[0] >> n.bmin_m[1] >> n.bmin_m[2] >> n.bmax_m[0] >> n.bmax_m[1] >> n.bmax_m[2]; }
        std::getline(ss, seg, '\t');
        { std::istringstream v(seg); v >> n.feat[0] >> n.feat[1] >> n.feat[2]; }
        sg_.add_node(n);
        if (parent != "-") parents.push_back({n.id, parent});
      }
      for (auto& pr : parents)
        if (sg_.nodes.count(pr.second)) sg_.set_parent(pr.first, pr.second);
      have_sg_ = !sg_.order.empty();
    }
    return true;
  }

  // ---------- 周期场景图线程(gvd_live 链路) ----------
  void cycle_loop() {
    while (running_) {
      auto t0 = Clk::now();
      bool did = run_cycle();
      double el = ms_between(t0, Clk::now()) / 1000.0;
      if (did) {
        ++n_cycle_;
        cyc_ms_sum_ += el * 1000.0;
        if (el * 1000.0 > cyc_ms_max_) cyc_ms_max_ = el * 1000.0;
      }
      double wait = kCycleInterval - el;
      while (running_ && wait > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait -= 0.1;
      }
    }
  }

  bool run_cycle() {
    std::vector<uint8_t> occ_full, free_full, sem_full;
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      occ_full = obs_->occupancy_mask();
      free_full = obs_->observed_free_mask();
      sem_full = obs_->sem_label();
    }
    long total_occ = 0;
    for (uint8_t v : occ_full) total_occ += v;
    if (total_occ < 500) return false;

    // 紧致裁剪(pad=1)
    int lo[3] = {nx_, ny_, nz_}, hi[3] = {0, 0, 0};
    for (int i = 0; i < nx_; ++i)
      for (int j = 0; j < ny_; ++j)
        for (int k = 0; k < nz_; ++k)
          if (occ_full[((long)i * ny_ + j) * nz_ + k]) {
            lo[0] = std::min(lo[0], i); lo[1] = std::min(lo[1], j); lo[2] = std::min(lo[2], k);
            hi[0] = std::max(hi[0], i); hi[1] = std::max(hi[1], j); hi[2] = std::max(hi[2], k);
          }
    int cl[3], ch[3];
    for (int a = 0; a < 3; ++a) {
      cl[a] = std::max(lo[a] - 1, 0);
      ch[a] = std::min(hi[a] + 2, a == 0 ? nx_ : a == 1 ? ny_ : nz_);
    }
    int cnx = ch[0] - cl[0], cny = ch[1] - cl[1], cnz = ch[2] - cl[2];
    long CN = (long)cnx * cny * cnz;
    std::vector<uint8_t> occ(CN), sem(CN), fre(CN);
    for (int i = 0; i < cnx; ++i)
      for (int j = 0; j < cny; ++j)
        for (int k = 0; k < cnz; ++k) {
          long src = ((long)(i + cl[0]) * ny_ + (j + cl[1])) * nz_ + (k + cl[2]);
          long dst = ((long)i * cny + j) * cnz + k;
          occ[dst] = occ_full[src];
          sem[dst] = sem_full[src];
          fre[dst] = (free_full[src] && !occ_full[src]) ? 1 : 0;  // free & ~occ(裁剪前语义)
        }
    long vmin_c[3] = {vmin_[0] + cl[0], vmin_[1] + cl[1], vmin_[2] + cl[2]};

    occ = denoise_occupancy(occ.data(), cnx, cny, cnz, 30);
    std::vector<float> dist; std::vector<long> parent;
    compute_esdf(occ.data(), cnx, cny, cnz, (float)kVs, dist, parent);
    auto gvd = thin_gvd(
        extract_gvd(fre.data(), dist.data(), parent.data(), cnx, cny, cnz, (float)kVs, 0.20f, 0.40f).data(),
        cnx, cny, cnz);
    std::vector<std::array<double, 3>> gvd_cells;  // 骨架体素 vxw 世界坐标(可视化)
    for (int i = 0; i < cnx; ++i)
      for (int j = 0; j < cny; ++j)
        for (int k = 0; k < cnz; ++k)
          if (gvd[((long)i * cny + j) * cnz + k])
            gvd_cells.push_back({(i + vmin_c[0] + 0.5) * kVs, (j + vmin_c[1] + 0.5) * kVs,
                                 (k + vmin_c[2] + 0.5) * kVs});
    auto g = skeleton_to_graph(gvd.data(), dist.data(), cnx, cny, cnz, (float)kVs, 0.15f);
    g = prune_spurs(g, 0.3);
    g = merge_close(g, 0.2, vmin_c, kVs);
    g = drop_small_components(g, 5);
    if (g.nodes.empty()) return false;
    auto room = partition_rooms_clearance(g, 0.85f, 8);
    merge_nested_rooms(g, room, kVs, 0.3);
    auto objs = extract_objects(occ.data(), sem.data(), cnx, cny, cnz, structure_ids_,
                                20, 3.0, 5, 2, kVs);
    link_to_places(objs, g, vmin_c, (float)kVs);
    // 楼层地板高度(来自 nav 分层的 rep_z, ROS z = vxw y): 每层独立出 3D surface 瓦片
    std::vector<int> floors_vox;
    {
      std::lock_guard<std::mutex> lk(nav_mtx_);
      for (auto& lv : levels_)
        floors_vox.push_back((int)std::floor(lv.rep_z() / kVs) - (int)vmin_c[1] - 1);
    }
    auto surf = surface_tiles(fre, occ, cnx, cny, cnz, vmin_c, kVs, floors_vox);

    MergeStats st{0, 0, 0, 0, 0};
    if (!have_sg_) {
      sg_ = build_scene_graph(g, room, objs, vmin_c, (float)kVs);
      have_sg_ = true;
    } else {
      sg_ = merge_observation(sg_, g, room, objs, vmin_c, (float)kVs, st, id_counter_);
      tot_matched_ += st.matched; tot_added_ += st.added;
      tot_removed_ += st.removed; tot_carried_ += st.carried;
    }
    int nr = 0, np = 0, no = 0;
    {
      std::set<int> rooms_set;
      for (auto& id : sg_.order) {
        auto& n = sg_.nodes[id];
        if (n.layer == "room") ++nr;
        else if (n.layer == "place") ++np;
        else if (n.layer == "object") ++no;
      }
      (void)rooms_set;
    }
    last_rooms_ = nr; last_places_ = np; last_objects_ = no;
    publish_dsg(g, room, surf, vmin_c, gvd_cells);
    publish_nav();   // 每周期重发代价地图(不依赖新帧, 重启/无播包时 rviz 也能收到)
    dump_dsg();
    save_map();  // 每周期存档(占据图+DSG+nav层), 重启可恢复
    RCLCPP_INFO(get_logger(), "cycle rooms=%d places=%d objects=%d merge={m=%d a=%d r=%d c=%d}",
                nr, np, no, st.matched, st.added, st.removed, st.carried);
    return true;
  }

  // SceneGraph 可视化: 元素分组构建 -> 各自单独话题(/dsg/rooms|places|objects|gvd|surface)
  // + 聚合 /dsg。每组开头 DELETEALL 防残留。复刻 gvd_live.publish_sg 的元素语义。
  void publish_dsg(const SkelGraph& g, const std::vector<int>& room,
                   const std::vector<std::array<double, 4>>& surf, const long vmin_c[3],
                   const std::vector<std::array<double, 3>>& gvd_cells) {
    int mid = 0;
    auto mk = [&](int type, const std::string& ns) {
      Marker m;
      m.header.frame_id = kMapFrame;
      m.header.stamp = now();
      m.ns = ns; m.id = mid++; m.type = type; m.action = Marker::ADD;
      m.pose.orientation.w = 1.0;
      return m;
    };
    auto room_rgba = [&](int rid, float a) {
      std_msgs::msg::ColorRGBA c;
      const float* col = ROOM_COLORS[((rid % 12) + 12) % 12];
      c.r = col[0]; c.g = col[1]; c.b = col[2]; c.a = a;
      return c;
    };
    auto to_ros = [&](const double v[3], double& rx, double& ry, double& rz) {
      yup2ros(v[0], v[1], v[2], rx, ry, rz);
    };
    std::vector<Marker> m_rooms, m_places, m_objects, m_gvd, m_surface;

    // ---- places: 节点球(按房间色)+ roadmap 白线 ----
    auto npos = [&](int i, double& x, double& y, double& z) {
      double v[3] = {(g.nodes[i].i + vmin_c[0]) * kVs, (g.nodes[i].j + vmin_c[1]) * kVs,
                     (g.nodes[i].k + vmin_c[2]) * kVs};
      to_ros(v, x, y, z);
    };
    Marker road = mk(Marker::LINE_LIST, "roadmap");
    road.scale.x = 0.015;
    std_msgs::msg::ColorRGBA rc; rc.r = rc.g = rc.b = 0.9f; rc.a = 0.7f;
    for (auto& e : g.edges) {
      double ax, ay, az, bx, by, bz;
      npos(e.a, ax, ay, az); npos(e.b, bx, by, bz);
      geometry_msgs::msg::Point pa, pb;
      pa.x = ax; pa.y = ay; pa.z = az; pb.x = bx; pb.y = by; pb.z = bz;
      road.points.push_back(pa); road.points.push_back(pb);
      road.colors.push_back(rc); road.colors.push_back(rc);
    }
    m_places.push_back(road);
    Marker pn = mk(Marker::SPHERE_LIST, "places");
    pn.scale.x = pn.scale.y = pn.scale.z = 0.12;
    for (size_t i = 0; i < g.nodes.size(); ++i) {
      double x, y, z; npos((int)i, x, y, z);
      geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = z;
      pn.points.push_back(p);
      pn.colors.push_back(room_rgba(room[i], 0.95f));
    }
    m_places.push_back(pn);

    // ---- 分层平面(Hydra 式): 楼层地面 + 物体层(+2.8m 同平面) + 房间层(+4.8m 同平面) ----
    const double kObjLayer = 2.8, kRoomLayer = 4.8;
    std::vector<double> floor_z;  // 各楼层地面高度(ROS z)
    {
      std::lock_guard<std::mutex> lk(nav_mtx_);
      for (auto& lv : levels_) floor_z.push_back(lv.rep_z());
    }
    if (floor_z.empty()) floor_z.push_back(-3.0);
    auto floor_of = [&](double z) {
      int b = 0; double bd = 1e18;
      for (size_t i = 0; i < floor_z.size(); ++i) {
        double d = std::fabs(z - floor_z[i]);
        if (d < bd) { bd = d; b = (int)i; }
      }
      return b;
    };

    // ---- rooms: 同层同平面的球+标签 + **实际轮廓**(可走瓦片按最近 place 归房, 边界格边) ----
    // 每层 places(ROS x,y) 列表
    std::vector<std::vector<int>> places_by_floor(floor_z.size());
    std::vector<std::array<double, 3>> ppos_ros(g.nodes.size());
    for (size_t i = 0; i < g.nodes.size(); ++i) {
      double x, y, z; npos((int)i, x, y, z);
      ppos_ros[i] = {x, y, z};
      places_by_floor[floor_of(z)].push_back((int)i);
    }
    // 瓦片 -> 房间归属(取该层最近 place 的房间; 该层无 place 则全局最近)
    std::map<std::array<int, 3>, int> tile_room;  // (lvl, i, k) -> rid
    for (auto& t : surf) {
      int lvl = (int)t[3];
      double rx, ry, rz; yup2ros(t[0], t[1], t[2], rx, ry, rz);
      const std::vector<int>* cand = (lvl < (int)places_by_floor.size() &&
                                      !places_by_floor[lvl].empty())
                                         ? &places_by_floor[lvl] : nullptr;
      double bd = 1e18; int best = -1;
      if (cand) {
        for (int i : *cand) {
          double dx = ppos_ros[i][0] - rx, dy = ppos_ros[i][1] - ry;
          double d = dx * dx + dy * dy;
          if (d < bd) { bd = d; best = i; }
        }
      } else {
        for (size_t i = 0; i < ppos_ros.size(); ++i) {
          double dx = ppos_ros[i][0] - rx, dy = ppos_ros[i][1] - ry;
          double d = dx * dx + dy * dy;
          if (d < bd) { bd = d; best = (int)i; }
        }
      }
      if (best < 0) continue;
      int ig = (int)std::lround(t[0] / kVs - vmin_c[0] - 0.5);
      int kg = (int)std::lround(t[2] / kVs - vmin_c[2] - 0.5);
      tile_room[{lvl, ig, kg}] = room[best];
    }
    // 实际轮廓: 与邻格房间不同(或无格)的格边 -> 线段(画在该层房间平面)
    Marker routline = mk(Marker::LINE_LIST, "room_outline");
    routline.scale.x = 0.03;
    for (auto& kv : tile_room) {
      int lvl = kv.first[0], ig = kv.first[1], kg = kv.first[2], rid = kv.second;
      double zl = floor_z[std::min((size_t)lvl, floor_z.size() - 1)] + kRoomLayer;
      double vx = (ig + vmin_c[0] + 0.5) * kVs, vz = (kg + vmin_c[2] + 0.5) * kVs;
      static const int D[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      for (auto& d : D) {
        auto nb = tile_room.find({lvl, ig + d[0], kg + d[1]});
        if (nb != tile_room.end() && nb->second == rid) continue;
        // 边: 垂直于 d 方向, 过格边中点(vxw XZ 平面)
        double ex = vx + d[0] * 0.5 * kVs, ez = vz + d[1] * 0.5 * kVs;
        double ax, az, bx, bz;
        if (d[0] != 0) { ax = ex; az = ez - 0.5 * kVs; bx = ex; bz = ez + 0.5 * kVs; }
        else { ax = ex - 0.5 * kVs; az = ez; bx = ex + 0.5 * kVs; bz = ez; }
        double r1x, r1y, r1z, r2x, r2y, r2z;
        yup2ros(ax, 0, az, r1x, r1y, r1z); yup2ros(bx, 0, bz, r2x, r2y, r2z);
        geometry_msgs::msg::Point pa, pb;
        pa.x = r1x; pa.y = r1y; pa.z = zl; pb.x = r2x; pb.y = r2y; pb.z = zl;
        routline.points.push_back(pa); routline.points.push_back(pb);
        routline.colors.push_back(room_rgba(rid, 0.95f));
        routline.colors.push_back(room_rgba(rid, 0.95f));
      }
    }
    Marker rlines = mk(Marker::LINE_LIST, "room_edges");
    rlines.scale.x = 0.01;
    for (auto& id : sg_.order) {
      auto& n = sg_.nodes[id];
      if (n.layer != "room") continue;
      int rid = std::atoi(n.id.c_str() + 5);
      double x, y, z; to_ros(n.pos, x, y, z);
      double zl = floor_z[floor_of(z)] + kRoomLayer;  // 同层房间同一平面
      Marker s = mk(Marker::SPHERE, "rooms");
      s.pose.position.x = x; s.pose.position.y = y; s.pose.position.z = zl;
      s.scale.x = s.scale.y = s.scale.z = 0.45;
      s.color = room_rgba(rid, 1.0f);
      m_rooms.push_back(s);
      Marker t = mk(Marker::TEXT_VIEW_FACING, "room_labels");
      t.pose.position.x = x; t.pose.position.y = y; t.pose.position.z = zl + 0.4;
      t.scale.z = 0.35; t.text = n.id;
      t.color.r = t.color.g = t.color.b = 1.0f; t.color.a = 1.0f;
      m_rooms.push_back(t);
      for (auto& cid : n.children) {
        auto it = sg_.nodes.find(cid);
        if (it == sg_.nodes.end() || it->second.layer != "place") continue;
        double cx, cy, cz; to_ros(it->second.pos, cx, cy, cz);
        geometry_msgs::msg::Point pa, pb;
        pa.x = x; pa.y = y; pa.z = zl; pb.x = cx; pb.y = cy; pb.z = cz;
        rlines.points.push_back(pa); rlines.points.push_back(pb);
        rlines.colors.push_back(room_rgba(rid, 0.35f));
        rlines.colors.push_back(room_rgba(rid, 0.35f));
      }
    }
    m_rooms.push_back(rlines);
    m_rooms.push_back(routline);

    // ---- objects: 类别色 bbox + 名称文字 + 头顶矩形块 + 连线(物体/所属place) + 支撑线 ----
    static const int E[12][2] = {{0,1},{1,3},{3,2},{2,0},{4,5},{5,7},{7,6},{6,4},{0,4},{1,5},{2,6},{3,7}};
    Marker wire = mk(Marker::LINE_LIST, "object_bbox");
    wire.scale.x = 0.02;
    for (auto& id : sg_.order) {
      auto& n = sg_.nodes[id];
      if (n.layer != "object") continue;
      double a1, a2, a3, b1, b2, b3;
      to_ros(n.bmin_m, a1, a2, a3); to_ros(n.bmax_m, b1, b2, b3);
      double lo[3] = {std::min(a1, b1), std::min(a2, b2), std::min(a3, b3)};
      double hi[3] = {std::max(a1, b1), std::max(a2, b2), std::max(a3, b3)};
      std_msgs::msg::ColorRGBA col;
      golden_color(n.obj_class, 0.75, 0.95, col.r, col.g, col.b);
      col.a = 1.0f;
      double corners[8][3];
      for (int v = 0; v < 8; ++v) {
        corners[v][0] = (v & 1) ? hi[0] : lo[0];
        corners[v][1] = (v & 2) ? hi[1] : lo[1];
        corners[v][2] = (v & 4) ? hi[2] : lo[2];
      }
      for (auto& ed : E)
        for (int t = 0; t < 2; ++t) {
          geometry_msgs::msg::Point p;
          p.x = corners[ed[t]][0]; p.y = corners[ed[t]][1]; p.z = corners[ed[t]][2];
          wire.points.push_back(p);
          wire.colors.push_back(col);
        }
      double cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2;
      // 名称文字
      Marker t = mk(Marker::TEXT_VIEW_FACING, "object_labels");
      t.pose.position.x = cx; t.pose.position.y = cy; t.pose.position.z = hi[2] + 0.12;
      t.scale.z = 0.16;
      auto nit = ls_.names.find(n.obj_class);
      t.text = nit != ls_.names.end() ? nit->second : std::to_string(n.obj_class);
      t.color.r = 1.0f; t.color.g = 0.8f; t.color.b = 0.4f; t.color.a = 1.0f;
      m_objects.push_back(t);
      // 头顶矩形块(Hydra 式物体节点): 同层物体统一放"物体层平面"(空旷带)
      Marker nc = mk(Marker::CUBE, "object_nodes");
      double nz = floor_z[floor_of(hi[2])] + kObjLayer;
      nc.pose.position.x = cx; nc.pose.position.y = cy; nc.pose.position.z = nz;
      nc.scale.x = nc.scale.y = nc.scale.z = 0.14;
      nc.color = col;
      m_objects.push_back(nc);
      // 连线: 头顶块->bbox 顶, 头顶块->所属 place
      Marker el = mk(Marker::LINE_LIST, "object_edges");
      el.scale.x = 0.012;
      geometry_msgs::msg::Point p1, p2;
      p1.x = cx; p1.y = cy; p1.z = nz; p2.x = cx; p2.y = cy; p2.z = hi[2];
      el.points.push_back(p1); el.points.push_back(p2);
      el.colors.push_back(col); el.colors.push_back(col);
      if (n.place_id >= 0) {
        auto pit = sg_.nodes.find("place:" + std::to_string(n.place_id));
        if (pit != sg_.nodes.end()) {
          double px, py, pz; to_ros(pit->second.pos, px, py, pz);
          geometry_msgs::msg::Point p3; p3.x = px; p3.y = py; p3.z = pz;
          el.points.push_back(p1); el.points.push_back(p3);
          std_msgs::msg::ColorRGBA c2 = col; c2.a = 0.45f;
          el.colors.push_back(c2); el.colors.push_back(c2);
        }
      }
      m_objects.push_back(el);
      // 支撑父子(杯在桌上): 品红线连两物体中心
      if (!n.parent.empty()) {
        auto par = sg_.nodes.find(n.parent);
        if (par != sg_.nodes.end() && par->second.layer == "object") {
          Marker se = mk(Marker::LINE_LIST, "support_edges");
          se.scale.x = 0.03;
          std_msgs::msg::ColorRGBA sc; sc.r = 1.0f; sc.g = 0.1f; sc.b = 1.0f; sc.a = 0.95f;
          double ox, oy, oz, px, py, pz;
          to_ros(n.pos, ox, oy, oz); to_ros(par->second.pos, px, py, pz);
          geometry_msgs::msg::Point pa, pb;
          pa.x = ox; pa.y = oy; pa.z = oz; pb.x = px; pb.y = py; pb.z = pz;
          se.points.push_back(pa); se.points.push_back(pb);
          se.colors.push_back(sc); se.colors.push_back(sc);
          m_objects.push_back(se);
        }
      }
    }
    m_objects.push_back(wire);

    // ---- gvd: 骨架体素(青色) ----
    if (!gvd_cells.empty()) {
      Marker sk = mk(Marker::CUBE_LIST, "gvd_skeleton");
      sk.scale.x = sk.scale.y = sk.scale.z = kVs * 0.6;
      std_msgs::msg::ColorRGBA skc; skc.r = 0.0f; skc.g = 0.9f; skc.b = 0.9f; skc.a = 0.9f;
      for (auto& c : gvd_cells) {
        geometry_msgs::msg::Point p;
        double rx, ry, rz; yup2ros(c[0], c[1], c[2], rx, ry, rz);
        p.x = rx; p.y = ry; p.z = rz;
        sk.points.push_back(p);
        sk.colors.push_back(skc);
      }
      m_gvd.push_back(sk);
    }

    // ---- surface: 3D 地面可通行瓦片(多楼层, 每层一色)----
    if (!surf.empty()) {
      Marker sm = mk(Marker::CUBE_LIST, "surface_places");
      sm.scale.x = sm.scale.y = 0.095; sm.scale.z = 0.02;
      std::map<int, std_msgs::msg::ColorRGBA> lut;
      for (auto& p : surf) {
        geometry_msgs::msg::Point q;
        double rx, ry, rz; yup2ros(p[0], p[1], p[2], rx, ry, rz);
        q.x = rx; q.y = ry; q.z = rz;
        sm.points.push_back(q);
        int lvl = (int)p[3];
        auto it = lut.find(lvl);
        if (it == lut.end()) {
          std_msgs::msg::ColorRGBA c;
          if (lvl == 0) { c.r = 0.15f; c.g = 0.85f; c.b = 0.75f; }   // L0 青
          else golden_color(60 + lvl * 7, 0.6, 1.0, c.r, c.g, c.b);  // 其余层黄金角
          c.a = 0.55f;
          it = lut.emplace(lvl, c).first;
        }
        sm.colors.push_back(it->second);
      }
      m_surface.push_back(sm);
    }

    // 分话题发布(各带 DELETEALL)+ 聚合 /dsg
    auto pub_group = [&](rclcpp::Publisher<MarkerArray>::SharedPtr& pub,
                         const std::vector<Marker>& ms) {
      MarkerArray a;
      Marker w; w.action = Marker::DELETEALL;
      a.markers.push_back(w);
      for (auto& m : ms) a.markers.push_back(m);
      pub->publish(a);
    };
    pub_group(room_pub_, m_rooms);
    pub_group(place_pub_, m_places);
    pub_group(obj_pub_, m_objects);
    pub_group(gvd_pub_, m_gvd);
    pub_group(surf_pub_, m_surface);
    MarkerArray all;
    Marker w; w.action = Marker::DELETEALL;
    all.markers.push_back(w);
    for (auto* grp : {&m_rooms, &m_places, &m_objects, &m_gvd, &m_surface})
      for (auto& m : *grp) all.markers.push_back(m);
    dsg_pub_->publish(all);
  }

  void dump_dsg() {
    std::ofstream f("/mnt/hgfs/Shared/claude_jobs/cpp_dsg.txt");
    for (auto& id : sg_.order) {
      auto& n = sg_.nodes[id];
      f << n.id << "|" << n.layer << "|" << n.parent << "|" << n.pos[0] << "," << n.pos[1]
        << "," << n.pos[2] << "|" << n.obj_class << "|" << n.voxel_count << "|misses=" << n.misses
        << "|seen=" << n.seen_count << "\n";
    }
  }

  struct Pending {
    Clk::time_point t_in;
    Image::ConstSharedPtr depth, seg, rgb;
  };

  std::array<long, 3> vmin_;
  int nx_, ny_, nz_;
  std::unique_ptr<ObsMap> obs_;
  std::mutex map_mtx_;
  std::set<int> structure_ids_;
  bool have_k_ = false;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  std::deque<Pending> pending_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  message_filters::Subscriber<Image> depth_sub_, seg_sub_, rgb_sub_;
  std::unique_ptr<Sync> sync_;
  rclcpp::Publisher<MarkerArray>::SharedPtr dsg_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr vox_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr obj_pub_, room_pub_, place_pub_, gvd_pub_, surf_pub_, sp_pub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> navmap_pubs_;
  LabelSpace ls_;
  std::vector<NavLevel> levels_;
  std::mutex nav_mtx_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tflis_;
  std::thread cycle_thread_;
  std::atomic<bool> running_{true};
  // DSG 状态(cycle 线程独占)
  SceneGraph sg_;
  bool have_sg_ = false;
  int id_counter_ = 0;
  // 统计
  long n_int_ = 0, n_skip_ = 0, n_cycle_ = 0;
  double int_ms_sum_ = 0, int_ms_max_ = 0, cyc_ms_sum_ = 0, cyc_ms_max_ = 0;
  int last_rooms_ = 0, last_places_ = 0, last_objects_ = 0;
  long tot_matched_ = 0, tot_added_ = 0, tot_removed_ = 0, tot_carried_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmcLiveNode>();
  rclcpp::spin(node);
  node->dump_stats();
  rclcpp::shutdown();
  return 0;
}
