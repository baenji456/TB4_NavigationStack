#include "nav2_oaa_costmap_plugin/simple_oaa_layer.hpp"

#include <algorithm>
#include <utility>
#include <limits>
#include <cmath>
#include <array>       // std::array


#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace nav2_oaa_costmap_plugin
{

// ---------------- CostmapLayer interface ----------------

void SimpleOaaLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) throw std::runtime_error("SimpleOaaLayer: failed to lock node");

  // Basic layer/bookkeeping
  rolling_window_ = layered_costmap_->isRolling();
  global_frame_   = layered_costmap_->getGlobalFrameID();
  default_value_  = FREE_SPACE;
  current_        = true;

  // --- Parameters (minimal set) ---
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("odom_topic", rclcpp::ParameterValue(odom_topic_));
  declareParameter("unknown_is_free", rclcpp::ParameterValue(unknown_is_free_));
  declareParameter("v_min", rclcpp::ParameterValue(v_min_));
  declareParameter("speed_smoothing_alpha", rclcpp::ParameterValue(speed_smoothing_alpha_));
  declareParameter("scaling_method", rclcpp::ParameterValue(scaling_method_));
  declareParameter("data_timeout", rclcpp::ParameterValue(data_timeout_));

  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".odom_topic", odom_topic_);
  node->get_parameter(name_ + ".unknown_is_free", unknown_is_free_);
  node->get_parameter(name_ + ".v_min", v_min_);
  node->get_parameter(name_ + ".speed_smoothing_alpha", speed_smoothing_alpha_);
  node->get_parameter(name_ + ".scaling_method", scaling_method_);
  node->get_parameter(name_ + ".data_timeout", data_timeout_);

  // QoS: RELIABLE + TRANSIENT_LOCAL so we catch latched last array
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();

  sub_array_ = node->create_subscription<GridArray>(
    topic_, qos,
    std::bind(&SimpleOaaLayer::arrayCallback, this, std::placeholders::_1));

  // Odometry best-effort is fine for speed estimation
  sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&SimpleOaaLayer::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "SimpleOaaLayer in frame '%s', subscribed to '%s' (array) and '%s' (odom)",
              global_frame_.c_str(), topic_.c_str(), odom_topic_.c_str());

    if (!tf_) {
        RCLCPP_ERROR(logger_, "SimpleOaaLayer: tf_ buffer is null!");
    }

  matchSize();
}

void SimpleOaaLayer::matchSize()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  // Nothing special to do here; we only need resolution at runtime
}

void SimpleOaaLayer::reset()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  {
    std::scoped_lock lk(mtx_);
    last_array_.reset();
    rel_times_sec_.clear();
    affines_.clear();
    horizon_sec_ = 0.0;
    have_bounds_ = false;
  }
  resetMaps();
  current_ = true;
}

// Called every costmap cycle to inform the master which region we may update
void SimpleOaaLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  rb_x_ = robot_x; rb_y_ = robot_y;

  // rolling window handling (standard)
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0,
                 robot_y - getSizeInMetersY() / 2.0);
  }

  std::shared_ptr<GridArray> local_arr;
  rclcpp::Time local_stamp;
  bool local_have_bounds = false;
  double lx0=0, ly0=0, lx1=0, ly1=0;

  {
    std::scoped_lock lk(mtx_);
    local_arr = last_array_;
    local_stamp = last_recv_time_;
    local_have_bounds = have_bounds_;
    lx0=bmin_x_; ly0=bmin_y_; lx1=bmax_x_; ly1=bmax_y_;
  }

  if (!local_arr) { current_ = false; return; }

  // Mark stale if data too old
  const double age = (clock_->now() - local_stamp).seconds();
  if (age > data_timeout_) { current_ = false; return; }

  current_ = true;

  if (local_have_bounds) {
    touch(lx0, ly0, min_x, min_y, max_x, max_y);
    touch(lx1, ly1, min_x, min_y, max_x, max_y);
  }
}

// Paint our costs into the master grid
void SimpleOaaLayer::updateCosts(nav2_costmap_2d::Costmap2D & master,
                                 int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());

  std::shared_ptr<GridArray> arr;
  std::vector<double> times;
  std::vector<Affine2D> A;

  {
    std::scoped_lock lk(mtx_);
    arr = last_array_;
    times = rel_times_sec_;
    A = affines_;
  }

  if (!arr || arr->grids.empty() || times.empty() || A.size() != arr->grids.size()) {
    return;
  }

  // Use smoothed speed with a safety floor
  const double v = std::max(v_ema_, v_min_);
  if (v <= 1e-6) return; // nothing meaningful to do

  const unsigned gw = arr->grids.front().info.width;
  const unsigned gh = arr->grids.front().info.height;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master.mapToWorld(i, j, wx, wy);

      // Expected time-to-arrive for this cell
      const double dx = wx - rb_x_;
      const double dy = wy - rb_y_;
      const double t_eta = std::hypot(dx, dy) / v;

      // Outside horizon? Skip.
      if (t_eta > horizon_sec_) continue;

      // Pick nearest time slice (no interpolation)
      // clamp the query time to the available horizon to always get a slice
      const double t_use = std::min(t_eta, horizon_sec_);
      const size_t k = nearestTimeSlice(times, t_use);
      const auto & g = arr->grids[k];

      int gi, gj;
      if (!A[k].worldToCell(wx, wy, gi, gj, gw, gh)) continue;

      const int8_t occ = g.data[static_cast<size_t>(gj) * gw + gi];

      // Treat low-quality/unknown as requested
      if (occ < 0 && unknown_is_free_) continue;

      const unsigned char new_cost = occToCost(occ);

      // Merge with MAX (per requirement)
      const unsigned char old_cost = master.getCost(i, j);
      if (old_cost != NO_INFORMATION) {
        master.setCost(i, j, std::max(old_cost, new_cost));
      } else {
        master.setCost(i, j, new_cost);
      }
    }
  }

  current_ = true;
}

// ---------------- Subscriptions ----------------

void SimpleOaaLayer::arrayCallback(const GridArray::SharedPtr msg)
{
  // We will:
  //  1) Store the array
  //  2) Build per-slice world->cell affines using TF (global_frame_ -> grid.frame)
  //  3) Build relative time axis t_k = (stamp_k - t0).sec with t0 from the first grid
  //  4) Precompute global AABB to limit update region

  if (!msg || msg->grids.empty()) return;

  auto node = node_.lock();
  if (!node) return;

  std::vector<Affine2D> aff(msg->grids.size());
  std::vector<double> times(msg->grids.size(), 0.0);

  // Reference time t0: use first grid stamp; array is guaranteed sorted
  const rclcpp::Time t0(msg->grids.front().header.stamp);

  // Build affines + time axis
  for (size_t k = 0; k < msg->grids.size(); ++k) {
    const auto & g = msg->grids[k];
    buildAffineFromGrid(g, aff[k]);
    times[k] = (rclcpp::Time(g.header.stamp) - t0).seconds();
    if (times[k] < 0.0) times[k] = 0.0; // safety
  }

  // Compute horizon from last timestamp (auto)
  const double horizon = times.back();

  // Compute (conservative) AABB from the first grid in global frame
  double minx=0, miny=0, maxx=0, maxy=0;
  computeGridAABBGlobal(msg->grids.front(), minx, miny, maxx, maxy);

  {
    std::scoped_lock lk(mtx_);
    last_array_ = std::make_shared<GridArray>(*msg);
    affines_ = std::move(aff);
    rel_times_sec_ = std::move(times);
    horizon_sec_ = horizon;
    last_recv_time_ = clock_->now();
    bmin_x_ = minx; bmin_y_ = miny; bmax_x_ = maxx; bmax_y_ = maxy;
    have_bounds_ = true;
  }

  current_ = true;
}

void SimpleOaaLayer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Compute planar speed in base frame and smooth with EMA
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  v_inst_ = std::hypot(vx, vy);

  if (!have_speed_.load(std::memory_order_relaxed)) {
    v_ema_ = v_inst_;
    have_speed_.store(true, std::memory_order_relaxed);
  } else {
    const double a = std::clamp(speed_smoothing_alpha_, 1e-3, 1.0);
    v_ema_ = a * v_inst_ + (1.0 - a) * v_ema_;
  }
}

// ---------------- Geometry helpers ----------------

bool SimpleOaaLayer::buildAffineFromGrid(const Grid &g, Affine2D &A) const
{
  // Convert a point expressed in global_frame_ into the grid's local
  // coordinates (meters), then divide by resolution to get cell index.
  //
  // Derivation:
  //  world(global) --T(global->grid.frame)--> grid frame
  //  then offset by grid.info.origin (pose of cell (0,0) in grid frame)
  //  also rotate by grid.info.origin orientation
  //
  // We pre-compose these into a linear part (M) and translation (C)
  // to map (wx,wy) directly into grid-local meters.

  // Rotation from grid origin
  const double yaw = tf2::getYaw(g.info.origin.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ox = g.info.origin.position.x;
  const double oy = g.info.origin.position.y;

  // R_origin^T (grid axes -> grid frame; later combined with TF)
  const double Rtx00 =  c, Rtx01 =  s;
  const double Rtx10 = -s, Rtx11 =  c;

  // TF: global_frame_ -> g.header.frame_id (source->target)
  double T00=1, T01=0, T10=0, T11=1, Tx=0, Ty=0;
  if (g.header.frame_id != global_frame_) {
    try {
      // lookupTransform(target, source): transform from source to target
      auto tf = tf_->lookupTransform(g.header.frame_id, global_frame_,
                                     rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      const double tyaw = tf2::getYaw(tf.transform.rotation);
      const double ct = std::cos(tyaw), st = std::sin(tyaw);
      T00 =  ct; T01 = -st;
      T10 =  st; T11 =  ct;
      Tx  =  tf.transform.translation.x;
      Ty  =  tf.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "TF lookup %s->%s failed: %s. Assuming same frame.",
        global_frame_.c_str(), g.header.frame_id.c_str(), ex.what());
    }
  }

  // A = R_origin^T * R_tf
  A.M00 = Rtx00 * T00 + Rtx01 * T10;
  A.M01 = Rtx00 * T01 + Rtx01 * T11;
  A.M10 = Rtx10 * T00 + Rtx11 * T10;
  A.M11 = Rtx10 * T01 + Rtx11 * T11;

  // C = R_origin^T * (t_tf - origin)
  const double dx = Tx - ox;
  const double dy = Ty - oy;
  A.C0 = Rtx00 * dx + Rtx01 * dy;
  A.C1 = Rtx10 * dx + Rtx11 * dy;

  A.res = g.info.resolution;
  return true;
}

void SimpleOaaLayer::computeGridAABBGlobal(const Grid &g,
                                           double &minx, double &miny,
                                           double &maxx, double &maxy) const
{
  // Compute axis-aligned bounding box of the grid in the global frame
  const double yaw = tf2::getYaw(g.info.origin.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ox = g.info.origin.position.x;
  const double oy = g.info.origin.position.y;
  const double w = g.info.width  * g.info.resolution;
  const double h = g.info.height * g.info.resolution;

  auto rot = [&](double x, double y) -> std::pair<double,double> {
    return { ox + c*x - s*y, oy + s*x + c*y };
  };

  std::array<std::pair<double,double>,4> corners = {
    rot(0,0), rot(w,0), rot(0,h), rot(w,h)
  };

  double T00=1, T01=0, T10=0, T11=1, Tx=0, Ty=0;
  if (g.header.frame_id != global_frame_) {
    try {
      // transform from grid frame to global frame
      auto tf = tf_->lookupTransform(global_frame_, g.header.frame_id,
                                     rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      const double tyaw = tf2::getYaw(tf.transform.rotation);
      const double ct = std::cos(tyaw), st = std::sin(tyaw);
      T00 =  ct; T01 = -st;
      T10 =  st; T11 =  ct;
      Tx  =  tf.transform.translation.x;
      Ty  =  tf.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "TF %s->%s failed (AABB): %s. Using grid frame.",
        g.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    }
  }

  auto to_global = [&](double x, double y) {
    return std::pair<double,double>{ T00*x + T01*y + Tx, T10*x + T11*y + Ty };
  };

  minx =  std::numeric_limits<double>::max();
  miny =  std::numeric_limits<double>::max();
  maxx = -std::numeric_limits<double>::max();
  maxy = -std::numeric_limits<double>::max();

  for (auto &p : corners) {
    auto pg = to_global(p.first, p.second);
    minx = std::min(minx, pg.first);  maxx = std::max(maxx, pg.first);
    miny = std::min(miny, pg.second); maxy = std::max(maxy, pg.second);
  }
}

// ---------------- Utilities ----------------

inline unsigned char SimpleOaaLayer::occToCost(int8_t occ) const
{
  // Pluggable scaling: currently only "linear" implemented
  if (occ < 0) {
    return unknown_is_free_ ? FREE_SPACE : NO_INFORMATION;
  }

  const int v = std::clamp<int>(occ, 0, 100);

  if (scaling_method_ == "linear") {
    if (v == 100) return LETHAL_OBSTACLE;         // reserve 254 for 100
    // map [0..99] -> [0..252] (approx 2.545 per step)
    return static_cast<unsigned char>(std::lround(v * 2.545));
  }

  // Fallback: treat as linear
  if (v == 100) return LETHAL_OBSTACLE;
  return static_cast<unsigned char>(std::lround(v * 2.545));
}

size_t SimpleOaaLayer::nearestTimeSlice(const std::vector<double>& times, double t_sec) const
{
  // Array is sorted; small linear scan is fine (arrays are typically short).
  // Could be replaced with std::lower_bound for logN time.
  if (times.empty()) return 0;
  if (t_sec <= times.front()) return 0;
  if (t_sec >= times.back())  return times.size() - 1;
  auto it = std::lower_bound(times.begin(), times.end(), t_sec);
  if (it == rel_times_sec_.begin()) return 0;

  const size_t idx = static_cast<size_t>(std::distance(times.begin(), it));
  const double t_hi = times[idx];
  const double t_lo = times[idx - 1];

  return (std::abs(t_hi - t_sec) < std::abs(t_sec - t_lo)) ? idx : (idx - 1);
}

} // namespace nav2_oaa_costmap_plugin

// Register with pluginlib
PLUGINLIB_EXPORT_CLASS(nav2_oaa_costmap_plugin::SimpleOaaLayer, nav2_costmap_2d::Layer)
