#pragma once

#include <mutex>
#include <string>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <atomic>                          // NEU (für std::atomic)
#include "nav_msgs/msg/odometry.hpp"       // NEU (Odometry-Typ)


// WICHTIG: von CostmapLayer erben, nicht nur Layer
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "prediction_msgs/msg/occupancy_grid_array.hpp"

namespace nav2_oaa_costmap_plugin
{

class PredictionLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  PredictionLayer();
  ~PredictionLayer() override;

  void onInitialize() override;
  void matchSize() override;

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void activate() override;
  void deactivate() override;
  void reset() override;

  bool isClearable() override { return false; }

protected:

  struct Affine2D {
    double M00{1}, M01{0}, M10{0}, M11{1}, C0{0}, C1{0}, res{1.0};
    inline bool worldToCell(double wx, double wy, int &gi, int &gj,
                            unsigned int gw, unsigned int gh) const {
      const double xl = M00 * wx + M01 * wy + C0;
      const double yl = M10 * wx + M11 * wy + C1;
      gi = static_cast<int>(std::floor(xl / res));
      gj = static_cast<int>(std::floor(yl / res));
      return (gi >= 0 && gj >= 0 && gi < static_cast<int>(gw) && gj < static_cast<int>(gh));
    }
  };

  // Parameter
  bool enabled_{true};
  int combination_method_{1};              // 0=overwrite, 1=max
  double min_apply_cost_{1.0};             // min. Occ (0..100), sonst ignorieren
  bool treat_unknown_as_free_{false};      // -1 -> NO_INFORMATION (Default: false)
  bool require_same_frame_{true};          // true: Frames müssen gleich sein
  double data_timeout_{2.0};               // [s]
  std::string topic_{"/predictions/costmaps"};
  std::string odom_topic_{"odom"};
  double forecast_dt_{0.2}, t0_offset_{0.0}, v_min_{0.05}, max_time_{5.0};
  bool use_grid_header_times_{false}, use_time_interp_{true};

  // Robot State
  double rb_x_{0.0}, rb_y_{0.0}, rb_yaw_{0.0};
  double v_lin_{0.0}; // Betrag in der Ebene
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_{nullptr};
  std::atomic<bool> have_odom_{false};

  using GridArrayMsg = prediction_msgs::msg::OccupancyGridArray;

  rclcpp::Subscription<GridArrayMsg>::SharedPtr sub_array_{nullptr}; // NEU
  int grid_index_{0};                                                // NEU

  void gridArrayCallback(const GridArrayMsg::SharedPtr msg);         // NEU


  // Array & Zeitachsen
  std::shared_ptr<GridArrayMsg> last_grid_array_;
  std::vector<Affine2D> affines_;        // eine Affine je Grid
  std::vector<double> x_of_grid_;        // Zeitmarken je Grid [s] relativ t0


  // Abo & Daten
  std::mutex grid_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_grid_{nullptr};
  rclcpp::Time last_stamp_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;

  // Cached
  std::string global_frame_;
  bool rolling_window_{false};
  double resolution_{0.0};

  // dyn params
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Bounds des letzten Grids in globalen Koordinaten
  bool have_bounds_{false};
  double gmin_x_{0.0}, gmin_y_{0.0}, gmax_x_{0.0}, gmax_y_{0.0};

  // Helpers
  unsigned char occToCost(int8_t occ) const;
  inline bool within(double v, double a, double b) const { return (v >= a && v <= b); }

  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Precompute affine transform global (wx,wy) -> Grid-Achsen
  bool buildAffineFromGrid(const nav_msgs::msg::OccupancyGrid &grid, Affine2D &A) const;
  void computeGridAABBGlobal(const nav_msgs::msg::OccupancyGrid &grid,
                             double &minx, double &miny, double &maxx, double &maxy) const;
};

}  // namespace nav2_oaa_costmap_plugin
