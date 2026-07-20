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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
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
#include <deque>
#include <fstream>
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

// labelspace yaml: "  - {label: 31, name: floor}" -> 结构类 id 集合
static std::set<int> load_structure_ids(const std::string& path) {
  std::set<int> out;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    auto lp = line.find("label:"); auto np = line.find("name:");
    if (lp == std::string::npos || np == std::string::npos) continue;
    int id = std::atoi(line.c_str() + lp + 6);
    std::string name = line.substr(np + 5);
    while (!name.empty() && (name.back() == '}' || name.back() == ' ' || name.back() == '\r'))
      name.pop_back();
    while (!name.empty() && name.front() == ' ') name.erase(name.begin());
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    for (const char* kw : kStructKw)
      if (name.find(kw) != std::string::npos) { out.insert(id); break; }
  }
  return out;
}

// _surface_tiles 移植: 地板层(占据 y 的 3% 分位, np.percentile 线性插值后截断)上方
// clearance_vox 层全 free 的 (X,Z) 列 -> 瓦片中心 vxw 世界坐标
static std::vector<std::array<double, 3>> surface_tiles(
    const std::vector<uint8_t>& free, const std::vector<uint8_t>& occ, int nx, int ny, int nz,
    const long vmin[3], double vs, int clearance_vox = 4) {
  std::vector<std::array<double, 3>> out;
  std::vector<int> ys;
  for (long id = 0; id < (long)nx * ny * nz; ++id)
    if (occ[id]) ys.push_back((int)((id / nz) % ny));
  if (ys.empty()) return out;
  std::sort(ys.begin(), ys.end());
  double q = 0.03 * (ys.size() - 1);
  size_t lo = (size_t)q;
  double frac = q - lo;
  double val = ys[lo] + (lo + 1 < ys.size() ? frac * (ys[lo + 1] - ys[lo]) : 0.0);
  int y_floor = (int)val;
  int y0 = y_floor + 1, y1 = y_floor + 1 + clearance_vox;
  if (y1 > ny) y1 = ny;
  for (int i = 0; i < nx; ++i)
    for (int k = 0; k < nz; ++k) {
      bool all = (y1 > y0);
      for (int j = y0; j < y1 && all; ++j)
        if (!free[((long)i * ny + j) * nz + k]) all = false;
      if (all)
        out.push_back({(i + vmin[0] + 0.5) * vs, (y_floor + vmin[1] + 1.0) * vs,
                       (k + vmin[2] + 0.5) * vs});
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
    structure_ids_ = load_structure_ids(kLabelspace);
    RCLCPP_INFO(get_logger(), "grid %dx%dx%d vmin=(%ld,%ld,%ld) structure_ids=%zu",
                nx_, ny_, nz_, vmin_[0], vmin_[1], vmin_[2], structure_ids_.size());

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
    dsg_pub_ = create_publisher<MarkerArray>("/dsg", rclcpp::QoS(1).transient_local());
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
        labels.push_back(seg[(long)v * W + u]);
      }
      ++su;
    }
    if (pts.empty()) return;
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
    auto surf = surface_tiles(fre, occ, cnx, cny, cnz, vmin_c, kVs);

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
    publish_dsg(g, room, surf, vmin_c);
    dump_dsg();
    RCLCPP_INFO(get_logger(), "cycle rooms=%d places=%d objects=%d merge={m=%d a=%d r=%d c=%d}",
                nr, np, no, st.matched, st.added, st.removed, st.carried);
    return true;
  }

  void publish_dsg(const SkelGraph& g, const std::vector<int>& room,
                   const std::vector<std::array<double, 3>>& surf, const long vmin_c[3]) {
    MarkerArray arr;
    auto mk = [&](int type, const std::string& ns) {
      Marker m;
      m.header.frame_id = kMapFrame;
      m.header.stamp = now();
      m.ns = ns; m.id = 0; m.type = type; m.action = Marker::ADD;
      m.pose.orientation.w = 1.0;
      return m;
    };
    // places 节点 + 边(按房间着色)
    Marker pn = mk(Marker::SPHERE_LIST, "places");
    pn.scale.x = pn.scale.y = pn.scale.z = 0.12;
    Marker pe = mk(Marker::LINE_LIST, "place_edges");
    pe.scale.x = 0.02;
    auto add_pt = [&](Marker& m, double vx, double vy, double vz, int rid) {
      geometry_msgs::msg::Point p;
      double rx, ry, rz; yup2ros(vx, vy, vz, rx, ry, rz);
      p.x = rx; p.y = ry; p.z = rz;
      m.points.push_back(p);
      std_msgs::msg::ColorRGBA c;
      const float* col = ROOM_COLORS[((rid % 12) + 12) % 12];
      c.r = col[0]; c.g = col[1]; c.b = col[2]; c.a = 1.0f;
      m.colors.push_back(c);
    };
    auto npos = [&](int i, double& x, double& y, double& z) {
      x = (g.nodes[i].i + vmin_c[0]) * kVs;
      y = (g.nodes[i].j + vmin_c[1]) * kVs;
      z = (g.nodes[i].k + vmin_c[2]) * kVs;
    };
    for (size_t i = 0; i < g.nodes.size(); ++i) {
      double x, y, z; npos((int)i, x, y, z);
      add_pt(pn, x, y, z, room[i]);
    }
    for (auto& e : g.edges) {
      double x, y, z; npos(e.a, x, y, z); add_pt(pe, x, y, z, room[e.a]);
      npos(e.b, x, y, z); add_pt(pe, x, y, z, room[e.b]);
    }
    arr.markers.push_back(pn);
    arr.markers.push_back(pe);
    // 房间球 + 物体 bbox(从场景图取)
    Marker rs = mk(Marker::SPHERE_LIST, "rooms");
    rs.scale.x = rs.scale.y = rs.scale.z = 0.35;
    Marker ob = mk(Marker::LINE_LIST, "object_bbox");
    ob.scale.x = 0.03;
    for (auto& id : sg_.order) {
      auto& n = sg_.nodes[id];
      if (n.layer == "room") {
        int rid = std::atoi(id.c_str() + 5);
        add_pt(rs, n.pos[0], n.pos[1], n.pos[2], rid);
      } else if (n.layer == "object") {
        static const int E[12][2] = {{0,1},{1,3},{3,2},{2,0},{4,5},{5,7},{7,6},{6,4},{0,4},{1,5},{2,6},{3,7}};
        double c[8][3];
        for (int v = 0; v < 8; ++v) {
          c[v][0] = (v & 1) ? n.bmax_m[0] : n.bmin_m[0];
          c[v][1] = (v & 2) ? n.bmax_m[1] : n.bmin_m[1];
          c[v][2] = (v & 4) ? n.bmax_m[2] : n.bmin_m[2];
        }
        for (auto& ed : E) {
          for (int t = 0; t < 2; ++t) {
            geometry_msgs::msg::Point p;
            double rx, ry, rz;
            yup2ros(c[ed[t]][0], c[ed[t]][1], c[ed[t]][2], rx, ry, rz);
            p.x = rx; p.y = ry; p.z = rz;
            ob.points.push_back(p);
            std_msgs::msg::ColorRGBA cc; cc.r = 1.0f; cc.g = 0.85f; cc.b = 0.1f; cc.a = 1.0f;
            ob.colors.push_back(cc);
          }
        }
      }
    }
    arr.markers.push_back(rs);
    arr.markers.push_back(ob);
    if (!surf.empty()) {
      Marker sm = mk(Marker::CUBE_LIST, "surface_places");
      sm.scale.x = sm.scale.y = 0.095; sm.scale.z = 0.02;
      for (auto& p : surf) {
        geometry_msgs::msg::Point q;
        double rx, ry, rz; yup2ros(p[0], p[1], p[2], rx, ry, rz);
        q.x = rx; q.y = ry; q.z = rz;
        sm.points.push_back(q);
        std_msgs::msg::ColorRGBA c; c.r = 0.15f; c.g = 0.85f; c.b = 0.75f; c.a = 0.55f;
        sm.colors.push_back(c);
      }
      arr.markers.push_back(sm);
    }
    dsg_pub_->publish(arr);
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
