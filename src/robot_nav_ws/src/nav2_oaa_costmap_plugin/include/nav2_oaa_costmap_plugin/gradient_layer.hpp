#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include <mutex>

namespace nav2_oaa_costmap_plugin
{

class GradientLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  GradientLayer();

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void reset() override { return; }
  void onFootprintChanged() override;
  bool isClearable() override { return false; }

private:
  // Subscription
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::mutex msg_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
  bool new_data_{false};

  // Weltkoordinaten-Bounds der letzten Nachricht
  double grid_min_x_{0.0}, grid_min_y_{0.0}, grid_max_x_{0.0}, grid_max_y_{0.0};

  // Parameter
  std::string topic_{"/predictions/costmap_t05s"};
  bool scale_to_lethal_{true}; // 100 -> LETHAL_OBSTACLE
};

}  // namespace nav2_oaa_costmap_plugin

#endif  // GRADIENT_LAYER_HPP_
