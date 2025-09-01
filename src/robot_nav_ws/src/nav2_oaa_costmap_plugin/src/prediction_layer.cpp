#include "nav2_oaa_costmap_plugin/prediction_layer.hpp"

#include <algorithm>
#include <cmath>
#include <array>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "prediction_msgs/msg/occupancy_grid_array.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;
using GridArrayMsg = prediction_msgs::msg::OccupancyGridArray; // anpassen falls nötig

namespace nav2_oaa_costmap_plugin
{

PredictionLayer::PredictionLayer()
{
  // nichts weiter nötig
}

PredictionLayer::~PredictionLayer()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

void PredictionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  rolling_window_ = layered_costmap_->isRolling();
  global_frame_   = layered_costmap_->getGlobalFrameID();
  default_value_  = nav2_costmap_2d::FREE_SPACE;
  current_        = true;

  // Parameter
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("combination_method", rclcpp::ParameterValue(1)); // 0=overwrite,1=max
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("min_apply_cost", rclcpp::ParameterValue(1.0));   // [0..100]
  declareParameter("treat_unknown_as_free", rclcpp::ParameterValue(false));
  declareParameter("require_same_frame", rclcpp::ParameterValue(true));
  declareParameter("data_timeout", rclcpp::ParameterValue(2.0));     // s

  declareParameter("odom_topic", rclcpp::ParameterValue(std::string{"/odom"}));
  declareParameter("forecast_dt", rclcpp::ParameterValue(0.1));      // [s] gleichmäßige Schrittweite
  declareParameter("use_grid_header_times", rclcpp::ParameterValue(false));
  declareParameter("t0_offset", rclcpp::ParameterValue(0.1));        // [s] Startversatz (z.B. 0 oder dt)
  declareParameter("v_min", rclcpp::ParameterValue(0.05));           // [m/s] Schutz gegen div/0
  declareParameter("use_time_interp", rclcpp::ParameterValue(true));  // lineare Zeitinterpolation
  declareParameter("max_time", rclcpp::ParameterValue(5.0));          // [s] Horizont begrenzen

  declareParameter("grid_index", rclcpp::ParameterValue(0)); // NEU
  node->get_parameter(name_ + "." + "grid_index", grid_index_); // NEU


  node->get_parameter(name_ + ".odom_topic", odom_topic_);
  node->get_parameter(name_ + ".forecast_dt", forecast_dt_);
  node->get_parameter(name_ + ".use_grid_header_times", use_grid_header_times_);
  node->get_parameter(name_ + ".t0_offset", t0_offset_);
  node->get_parameter(name_ + ".v_min", v_min_);
  node->get_parameter(name_ + ".use_time_interp", use_time_interp_);
  node->get_parameter(name_ + ".max_time", max_time_);


  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter(name_ + "." + "topic", topic_);
  node->get_parameter(name_ + "." + "min_apply_cost", min_apply_cost_);
  node->get_parameter(name_ + "." + "treat_unknown_as_free", treat_unknown_as_free_);
  node->get_parameter(name_ + "." + "require_same_frame", require_same_frame_);
  node->get_parameter(name_ + "." + "data_timeout", data_timeout_);

  matchSize();

  // QoS: latched + reliable
  rclcpp::QoS qos(1);
  qos.reliable();
  // qos.transient_local();

// Subscribe to a *sequence* of future occupancy grids and to odometry.
// QoS for grids is reliable + transient_local ("latched") so late joiners get the latest array.
// Odometry gives current planar speed for ETA calculation.
  sub_array_ = node->create_subscription<GridArrayMsg>(
    topic_, qos, std::bind(&PredictionLayer::gridArrayCallback, this, std::placeholders::_1));

  sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10).best_effort(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Betrag der Geschwindigkeit in Ebene (vx, vy in base_link)
      const double vx = msg->twist.twist.linear.x;
      const double vy = msg->twist.twist.linear.y;
      v_lin_ = std::hypot(vx, vy);
      have_odom_.store(true, std::memory_order_relaxed);
    });
  

  dyn_params_handler_ =
      node->add_on_set_parameters_callback(
        std::bind(&PredictionLayer::dynamicParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "PredictionLayer subscribed to '%s' (frame=%s, rolling=%d)",
              topic_.c_str(), global_frame_.c_str(), rolling_window_);
}

void PredictionLayer::matchSize()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D* cm = layered_costmap_->getCostmap();
  resolution_ = cm->getResolution();
}

void PredictionLayer::activate()
{
  current_ = true;
}

void PredictionLayer::deactivate()
{
  // latched Topic: keine besondere Aktion
}

void PredictionLayer::reset()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  last_grid_.reset();
  have_bounds_ = false;
  resetMaps();
  current_ = true;
}

unsigned char PredictionLayer::occToCost(int8_t occ) const
{
  if (occ < 0) {
    return treat_unknown_as_free_ ? FREE_SPACE : NO_INFORMATION;
  }
  const int v = std::clamp<int>(occ, 0, 100);
  // Map [0..99] -> [0..252], 100 -> lethal (254)
  if (v == 100) return nav2_costmap_2d::LETHAL_OBSTACLE;  // 254
  return static_cast<unsigned char>(std::lround(v * 2.545)); // 0..252
}

// void PredictionLayer::gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//   auto node = node_.lock();
//   if (!node) { return; }

//   if (require_same_frame_ && msg->header.frame_id != global_frame_) {
//     RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
//       "PredictionLayer: incoming grid frame '%s' != global_frame '%s' (ignored). "
//       "Set '%s.require_same_frame:=false' to enable TF-based fusion.",
//       msg->header.frame_id.c_str(), global_frame_.c_str(), name_.c_str());
//     return;
//   }

//   {
//     std::scoped_lock lk(grid_mutex_);
//     last_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
//     last_stamp_ = clock_->now();
//   }

//   double minx, miny, maxx, maxy;
//   computeGridAABBGlobal(*msg, minx, miny, maxx, maxy);
//   {
//     std::scoped_lock lk(grid_mutex_);
//     gmin_x_ = minx; gmin_y_ = miny; gmax_x_ = maxx; gmax_y_ = maxy;
//     have_bounds_ = true;
//   }

//   current_ = true;
// }

// Called when a new OccupancyGridArray arrives.
// Stores the array, precomputes per-slice affines (world->cell),
// and constructs the time axis either from header stamps or uniform dt.
void PredictionLayer::gridArrayCallback(const GridArrayMsg::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node || msg->grids.empty()) { return; }

  {
    std::scoped_lock lk(grid_mutex_);
    last_grid_array_ = std::make_shared<GridArrayMsg>(*msg);
    last_stamp_ = clock_->now();

    const size_t H = last_grid_array_->grids.size();
    affines_.resize(H);
    x_of_grid_.resize(H);

    // Referenzzeit t0
    const rclcpp::Time t0 = use_grid_header_times_
        ? rclcpp::Time(last_grid_array_->header.stamp)  // <-- konvertieren
        : clock_->now();
    const double t0_sec = t0.seconds() + t0_offset_;

    for (size_t k = 0; k < H; ++k) {
      const auto &g = last_grid_array_->grids[k];
      buildAffineFromGrid(g, affines_[k]);
      x_of_grid_[k] = use_grid_header_times_
        ? (rclcpp::Time(g.header.stamp).seconds() - t0_sec)  // <-- konvertieren
        : (static_cast<double>(k) * forecast_dt_);    
    }

    if (!use_grid_header_times_) {
      // Gleichmäßige dt: letzter gültiger Zeitpunkt = (H-1)*forecast_dt_
      max_time_ = std::min(max_time_, (H > 0) ? (H - 1) * forecast_dt_ : 0.0);
    } else {
      // Ungleichmäßige Zeiten: nimm das Ende der tatsächlichen Zeitachse
      const double horizon = x_of_grid_.empty() ? 0.0 : x_of_grid_.back();
      max_time_ = std::min(max_time_, horizon);
    }

    // Bounds – minimal: aus erster Karte
    double minx, miny, maxx, maxy;
    computeGridAABBGlobal(last_grid_array_->grids.front(), minx, miny, maxx, maxy);
    gmin_x_ = minx; gmin_y_ = miny; gmax_x_ = maxx; gmax_y_ = maxy;
    
    have_bounds_ = true;
  }

  current_ = true;
}




rcl_interfaces::msg::SetParametersResult
PredictionLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> params)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  for (auto & p : params) {
    const auto & n = p.get_name();
    const auto  t = p.get_type();
    if (n == name_ + ".enabled" && t == ParameterType::PARAMETER_BOOL) {
      enabled_ = p.as_bool();
    } else if (n == name_ + ".combination_method" && t == ParameterType::PARAMETER_INTEGER) {
      combination_method_ = p.as_int();
    } else if (n == name_ + ".min_apply_cost" && t == ParameterType::PARAMETER_DOUBLE) {
      min_apply_cost_ = p.as_double();
    } else if (n == name_ + ".treat_unknown_as_free" && t == ParameterType::PARAMETER_BOOL) {
      treat_unknown_as_free_ = p.as_bool();
    } else if (n == name_ + ".require_same_frame" && t == ParameterType::PARAMETER_BOOL) {
      require_same_frame_ = p.as_bool();
    } else if (n == name_ + ".data_timeout" && t == ParameterType::PARAMETER_DOUBLE) {
      data_timeout_ = p.as_double();
    } else if (n == name_ + ".grid_index" && t == ParameterType::PARAMETER_INTEGER) {
      grid_index_ = std::max<int>(0, static_cast<int>(p.as_int())); // NEU
    }
  
  }
  return res;
}

// --- Geometrie-Helfer -------------------------------------------------------

bool PredictionLayer::buildAffineFromGrid(const nav_msgs::msg::OccupancyGrid &grid, Affine2D &A) const
{
  const double yaw = tf2::getYaw(grid.info.origin.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;

  // R_origin^T (Grid-Achsen -> Welt)
  const double Rtx00 =  c, Rtx01 =  s;
  const double Rtx10 = -s, Rtx11 =  c;

  // TF global->grid (optional)
  double T00 = 1.0, T01 = 0.0, T10 = 0.0, T11 = 1.0, Tx = 0.0, Ty = 0.0;
  if (!require_same_frame_ && grid.header.frame_id != global_frame_) {
    try {
      auto tf = tf_->lookupTransform(grid.header.frame_id, global_frame_,
                                     clock_->now(), rclcpp::Duration::from_seconds(0.2));
      const double tyaw = tf2::getYaw(tf.transform.rotation);
      const double ct = std::cos(tyaw), st = std::sin(tyaw);
      T00 =  ct; T01 = -st;
      T10 =  st; T11 =  ct;
      Tx  =  tf.transform.translation.x;
      Ty  =  tf.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "PredictionLayer TF lookup failed (%s -> %s): %s. Using same-frame assumption.",
        global_frame_.c_str(), grid.header.frame_id.c_str(), ex.what());
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

  A.res = grid.info.resolution;
  return true;
}

void PredictionLayer::computeGridAABBGlobal(const nav_msgs::msg::OccupancyGrid &grid,
                                            double &minx, double &miny, double &maxx, double &maxy) const
{
  const double yaw = tf2::getYaw(grid.info.origin.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  const double w = grid.info.width * grid.info.resolution;
  const double h = grid.info.height * grid.info.resolution;

  auto rot = [&](double x, double y) -> std::pair<double,double> {
    return { ox + c * x - s * y, oy + s * x + c * y };
  };

  std::array<std::pair<double,double>,4> corners = {
    rot(0.0, 0.0), rot(w, 0.0), rot(0.0, h), rot(w, h)
  };

  // ggf. grid->global TF
  double T00 = 1.0, T01 = 0.0, T10 = 0.0, T11 = 1.0, Tx = 0.0, Ty = 0.0;
  if (!require_same_frame_ && grid.header.frame_id != global_frame_) {
    try {
      auto tf = tf_->lookupTransform(global_frame_, grid.header.frame_id,
                                     clock_->now(), rclcpp::Duration::from_seconds(0.2));
      const double tyaw = tf2::getYaw(tf.transform.rotation);
      const double ct = std::cos(tyaw), st = std::sin(tyaw);
      T00 =  ct; T01 = -st;
      T10 =  st; T11 =  ct;
      Tx  =  tf.transform.translation.x;
      Ty  =  tf.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "PredictionLayer TF (grid->global) failed: %s. Using grid frame as global.", ex.what());
    }
  }

  auto to_global = [&](double x, double y) -> std::pair<double,double> {
    return { T00 * x + T01 * y + Tx, T10 * x + T11 * y + Ty };
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

// --- Layer-Schnittstelle -----------------------------------------------------

// void PredictionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
//                                    double* min_x, double* min_y, double* max_x, double* max_y)
// {
//   std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
//   if (!enabled_) { return; }

//   // Pose puffern für updateCosts()
//   rb_x_ = robot_x; rb_y_ = robot_y; rb_yaw_ = robot_yaw;

//   if (rolling_window_) {
//     updateOrigin(robot_x - getSizeInMetersX() / 2.0,
//                  robot_y - getSizeInMetersY() / 2.0);
//   }

//   std::shared_ptr<GridArrayMsg> arr;
//   { std::scoped_lock lk(grid_mutex_); arr = last_grid_array_; }
//   if (!arr) { current_ = false; return; }
//   if ((clock_->now() - last_stamp_).seconds() > data_timeout_) { current_ = false; return; }
//   current_ = true;

//   if (have_bounds_) {
//     touch(gmin_x_, gmin_y_, min_x, min_y, max_x, max_y);
//     touch(gmax_x_, gmax_y_, min_x, min_y, max_x, max_y);
//   }
// }

void PredictionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) { return; }

  // Pose puffern für updateCosts()
  rb_x_ = robot_x; rb_y_ = robot_y; rb_yaw_ = robot_yaw;

  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0,
      robot_y - getSizeInMetersY() / 2.0);
  }

  std::shared_ptr<GridArrayMsg> arr;
  { std::scoped_lock lk(grid_mutex_); arr = last_grid_array_; }
  if (!arr) { current_ = false; return; }
  if ((clock_->now() - last_stamp_).seconds() > data_timeout_) { current_ = false; return; }
  current_ = true;

  if (have_bounds_) {
    touch(gmin_x_, gmin_y_, min_x, min_y, max_x, max_y);
    touch(gmax_x_, gmax_y_, min_x, min_y, max_x, max_y);
  }
}




// void PredictionLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
//                                   int min_i, int min_j, int max_i, int max_j)
// {
//   std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
//   if (!enabled_) { return; }

//   nav_msgs::msg::OccupancyGrid::SharedPtr grid;
//   {
//     std::scoped_lock lk(grid_mutex_);
//     grid = last_grid_;
//   }
//   if (!grid) { return; }

//   Affine2D A;
//   buildAffineFromGrid(*grid, A);

//   const unsigned int gw = grid->info.width;
//   const unsigned int gh = grid->info.height;

//   for (int j = min_j; j < max_j; ++j) {
//     for (int i = min_i; i < max_i; ++i) {

//       double wx, wy;
//       master_grid.mapToWorld(i, j, wx, wy);

//       int gi, gj;
//       if (!A.worldToCell(wx, wy, gi, gj, gw, gh)) {
//         continue;
//       }

//       const size_t g_idx = static_cast<size_t>(gj) * gw + gi;
//       const int8_t occ = grid->data[g_idx];

//       if (occ >= 0 && static_cast<double>(occ) < min_apply_cost_) {
//         continue;
//       }

//       const unsigned char new_cost = occToCost(occ);

//       // Unknown durchreichen
//       if (new_cost == NO_INFORMATION && !treat_unknown_as_free_) {
//         if (master_grid.getCost(i, j) == NO_INFORMATION) {
//           master_grid.setCost(i, j, NO_INFORMATION);
//         }
//         continue;
//       }

//       const unsigned char old_cost = master_grid.getCost(i, j);

//       // nur MAX (oder overwrite, falls explizit gesetzt)
//       switch (combination_method_) {
//         case 0: // overwrite
//           if (old_cost != nav2_costmap_2d::NO_INFORMATION) {
//             master_grid.setCost(i, j, new_cost);
//           }
//           break;
//         case 1: // max
//         default:
//           if (old_cost != nav2_costmap_2d::NO_INFORMATION) {
//             master_grid.setCost(i, j, std::max(old_cost, new_cost));
//           } else {
//             master_grid.setCost(i, j, new_cost);
//           }
//           break;
//       }
//     }
//   }

//   current_ = true;
// }

void PredictionLayer::updateCosts(nav2_costmap_2d::Costmap2D& master,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) { return; }

  std::shared_ptr<GridArrayMsg> arr;
  std::vector<Affine2D> aff; std::vector<double> xgrid;
  { // Thread-sicher kopieren
    std::scoped_lock lk(grid_mutex_);
    arr   = last_grid_array_;
    aff   = affines_;
    xgrid = x_of_grid_;
  }
  if (!arr || arr->grids.empty()) { return; }

  const size_t H = arr->grids.size();
  // Annahme: alle Grids haben gleiche Dimension (H_map x W_map):
  const unsigned gw = arr->grids[0].info.width;
  const unsigned gh = arr->grids[0].info.height;

  // aktuelle (sichere) Geschwindigkeit
  const double v = std::max(v_lin_, v_min_);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {

      // Weltkoordinate der Masterzelle
      double wx, wy; master.mapToWorld(i, j, wx, wy);

      // Reisezeit abschätzen: euklidische Distanz / v
      const double dx = wx - rb_x_;
      const double dy = wy - rb_y_;
      const double dist = std::hypot(dx, dy);
      const double t_travel = dist / v;

      if (t_travel > max_time_) { continue; } // außerhalb des Horizonts

      // passenden Zeitschritt finden: Index k mit minimalem |x_k - t_travel|
      size_t k_best = 0;
      if (use_time_interp_) {
        // Interp: finde k0<=t<k1
        // schnelle Variante bei gleichmäßigem dt:
        if (!use_grid_header_times_) {
          const double idx_f = t_travel / forecast_dt_;
          size_t k0 = static_cast<size_t>(std::floor(idx_f));
          // ---------------------- CLAMP START ----------------------
          if (k0 >= H) k0 = H - 1;                      // <<< CLAMP k0 in [0, H-1]
          size_t k1 = (k0 + 1 < H) ? (k0 + 1) : k0;     // <<< CLAMP k1 = k0 oder k0+1
          // ---------------------- CLAMP END ------------------------
          const double x0 = xgrid[k0];
          const double x1 = xgrid[k1];
          double w1 = (x1 > x0) ? (t_travel - x0) / (x1 - x0) : 0.0;
          w1 = std::clamp(w1, 0.0, 1.0);                // <<< CLAMP w1 in [0,1]
          const double w0 = 1.0 - w1;

          // Rasterindex in k0/k1 nach Welt → Zellkoordinate transformieren
          int gi0, gj0, gi1, gj1;
          bool ok0 = aff[k0].worldToCell(wx, wy, gi0, gj0, gw, gh);
          bool ok1 = aff[k1].worldToCell(wx, wy, gi1, gj1, gw, gh);
          if (!ok0 && !ok1) { continue; }

          double occ_lin = 0.0;
          if (ok0) {
            const int8_t occ0 = arr->grids[k0].data[static_cast<size_t>(gj0) * gw + gi0];
            if (!(occ0 >= 0 && static_cast<double>(occ0) < min_apply_cost_)) {
              occ_lin += w0 * static_cast<double>(occ0);
            }
          }
          if (ok1) {
            const int8_t occ1 = arr->grids[k1].data[static_cast<size_t>(gj1) * gw + gi1];
            if (!(occ1 >= 0 && static_cast<double>(occ1) < min_apply_cost_)) {
              occ_lin += w1 * static_cast<double>(occ1);
            }
          }

          // In Costmap-Kosten mappen
          occ_lin = std::clamp(occ_lin, 0.0, 100.0);     // <<< CLAMP Occ in [0,100]
          const unsigned char new_cost =
              occToCost(static_cast<int8_t>(std::lround(occ_lin)));

          // Unknown-Handhabung & Kombination wie gehabt
          if (new_cost == NO_INFORMATION && !treat_unknown_as_free_) {
            if (master.getCost(i, j) == NO_INFORMATION) {
              master.setCost(i, j, NO_INFORMATION);
            }
            continue;
          }
          const unsigned char old_cost = master.getCost(i, j);
          if (combination_method_ == 0) {
            if (old_cost != nav2_costmap_2d::NO_INFORMATION) master.setCost(i, j, new_cost);
          } else {
            if (old_cost != nav2_costmap_2d::NO_INFORMATION) {
              master.setCost(i, j, std::max(old_cost, new_cost));
            } else {
              master.setCost(i, j, new_cost);
            }
          }
          continue; // Interpolationspfad abgeschlossen
        }
      }

      // Kein Interp (oder ungleichmäßige Zeiten): bestes k per nächster Zeit
      {
        // lineare Suche (H meist klein). Optional: binäre Suche.
        double best = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k < H; ++k) {
          const double d = std::abs(xgrid[k] - t_travel);
          if (d < best) { best = d; k_best = k; }
        }

        int gi, gj;
        if (!aff[k_best].worldToCell(wx, wy, gi, gj, gw, gh)) { continue; }
        const int8_t occ = arr->grids[k_best].data[static_cast<size_t>(gj) * gw + gi];

        if (occ >= 0 && static_cast<double>(occ) < min_apply_cost_) { continue; }

        const unsigned char new_cost = occToCost(occ);
        if (new_cost == NO_INFORMATION && !treat_unknown_as_free_) {
          if (master.getCost(i, j) == NO_INFORMATION) master.setCost(i, j, NO_INFORMATION);
          continue;
        }

        const unsigned char old_cost = master.getCost(i, j);
        if (combination_method_ == 0) {
          if (old_cost != nav2_costmap_2d::NO_INFORMATION) master.setCost(i, j, new_cost);
        } else {
          if (old_cost != nav2_costmap_2d::NO_INFORMATION) {
            master.setCost(i, j, std::max(old_cost, new_cost));
          } else {
            master.setCost(i, j, new_cost);
          }
        }
      }
    }
  }

  current_ = true;
}



}  // namespace nav2_oaa_costmap_plugin

PLUGINLIB_EXPORT_CLASS(nav2_oaa_costmap_plugin::PredictionLayer, nav2_costmap_2d::Layer)
