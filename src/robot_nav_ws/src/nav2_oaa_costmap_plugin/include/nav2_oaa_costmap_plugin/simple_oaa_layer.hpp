#pragma once

#include <mutex>
#include <string>
#include <memory>
#include <vector>
#include <atomic>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "prediction_msgs/msg/occupancy_grid_array.hpp"

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_oaa_costmap_plugin
{

/**
 * @brief Simple OaA (Occupancy-as-Action) layer that consumes a time-sequence
 *        of predicted occupancy grids and stamps a local/global costmap
 *        according to the ETA (distance / smoothed_speed).
 *
 * Design goals:
 *  - minimal parameter set
 *  - easy-to-read code
 *  - comments in English
 */
class SimpleOaaLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SimpleOaaLayer() = default;
  ~SimpleOaaLayer() override = default;

  // CostmapLayer interface
  void onInitialize() override;
  void matchSize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void activate() override { current_ = true; }
  void deactivate() override {}
  void reset() override;

  bool isClearable() override { return false; }

private:
  using Grid = nav_msgs::msg::OccupancyGrid;
  using GridArray = prediction_msgs::msg::OccupancyGridArray;

  // -------- Parameters (kept minimal) --------
  std::string topic_{"/predictions/costmaps"}; // array of future grids
  std::string odom_topic_{"/odom"};
  bool unknown_is_free_{true};                 // -1 treated as FREE
  double v_min_{0.05};                         // [m/s] floor to avoid div/0
  double speed_smoothing_alpha_{0.3};          // EMA factor in (0,1]
  std::string scaling_method_{"linear"};       // extensible; currently: "linear"
  double data_timeout_{2.0};                   // [s] mark layer stale

  // -------- Subscriptions --------
  rclcpp::Subscription<GridArray>::SharedPtr sub_array_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // -------- Last prediction set --------
  std::mutex mtx_;
  std::shared_ptr<GridArray> last_array_;
  std::vector<double> rel_times_sec_;          // t_k relative to t0 [s], monotonic
  double horizon_sec_{0.0};                    // = rel_times_sec_.back() (auto)
  rclcpp::Time last_recv_time_;
  bool have_bounds_{false};
  double bmin_x_{0.0}, bmin_y_{0.0}, bmax_x_{0.0}, bmax_y_{0.0};

  // -------- Robot state --------
  double rb_x_{0.0}, rb_y_{0.0};               // buffered pose for ETA
  std::atomic<bool> have_speed_{false};
  double v_inst_{0.0};                         // instantaneous planar speed
  double v_ema_{0.0};                          // smoothed speed

  // --- Costmap / frame bookkeeping (we store them after onInitialize) ---
  std::string global_frame_;
  bool rolling_window_{false};

  // -------- Helpers --------
  struct Affine2D {
    // World (global_frame_) -> grid local cell coordinates (in meters)
    // cell = floor( (M * world + C) / res )
    double M00{1}, M01{0}, M10{0}, M11{1}, C0{0}, C1{0}, res{1.0};
    inline bool worldToCell(double wx, double wy, int &gi, int &gj,
                            unsigned gw, unsigned gh) const
    {
      const double xl = M00 * wx + M01 * wy + C0;
      const double yl = M10 * wx + M11 * wy + C1;
      gi = static_cast<int>(std::floor(xl / res));
      gj = static_cast<int>(std::floor(yl / res));
      return (gi >= 0 && gj >= 0 && gi < static_cast<int>(gw) && gj < static_cast<int>(gh));
    }
  };

  std::vector<Affine2D> affines_;              // one per grid slice (same order)

  // Building blocks
  void arrayCallback(const GridArray::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool buildAffineFromGrid(const Grid &g, Affine2D &A) const;
  void computeGridAABBGlobal(const Grid &g, double &minx, double &miny,
                             double &maxx, double &maxy) const;

  inline unsigned char occToCost(int8_t occ) const;
  size_t nearestTimeSlice(const std::vector<double>& times, double t_sec) const;
};

} // namespace nav2_oaa_costmap_plugin
