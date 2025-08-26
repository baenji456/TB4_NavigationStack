// Copyright 2025 Example User
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Implementation of the HumanTrajectoryLayer costmap plugin.  The
/// plugin listens for probabilistic human trajectory predictions and
/// paints them onto the costmap using anisotropic Gaussian kernels.
/// Each incoming prediction contains a mean pose, a confidence level
/// and standard deviations in x and y.  The confidence level scales
/// the cost and the standard deviations define the spatial extent of
/// the Gaussian.  This demonstrates how to integrate the confidence
/// sets proposed in the “Reliable Probabilistic Human Trajectory
/// Prediction for Autonomous Applications” paper into a Nav2 costmap
///【567447048113654†L319-L331】.

#include "human_trajectory_costmap_layer/human_trajectory_layer.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace human_trajectory_costmap_layer
{

HumanTrajectoryLayer::HumanTrajectoryLayer()
: gaussian_radius_(0.5),
  max_cost_(254.0)
{
}

HumanTrajectoryLayer::~HumanTrajectoryLayer() = default;

void HumanTrajectoryLayer::onInitialize()
{
  auto node = node_.lock();                 // <-- WeakPtr locken
  if (!node) {
    throw std::runtime_error("HumanTrajectoryLayer: node_ expired");
  }

  // Parameter deklarieren/lesen
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  declareParameter("gaussian_radius", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".gaussian_radius", gaussian_radius_);

  declareParameter("max_cost", rclcpp::ParameterValue(254.0));
  node->get_parameter(name_ + ".max_cost", max_cost_);

  // Subscription mit dem gelockten Node
  sub_ = node->create_subscription<human_trajectory_costmap_layer::msg::PredictedPositionArray>(
    "predicted_human_positions",
    rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&HumanTrajectoryLayer::predictionCallback, this, std::placeholders::_1));

  current_ = true;
}

void HumanTrajectoryLayer::predictionCallback(
  const human_trajectory_costmap_layer::msg::PredictedPositionArray::SharedPtr msg)
{
  predicted_positions_.assign(msg->positions.begin(), msg->positions.end());
  // Kein need_recalculation_ nötig; updateBounds/Costs arbeiten über Bounds.
}

bool HumanTrajectoryLayer::isClearable()
{
  // Diese Layer trägt nur zusätzliche Kosten ein; nicht "clearable".
  return false;
}

void
HumanTrajectoryLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                   double * min_x, double * min_y,
                                   double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  // Only expand bounds if there are predicted positions
  if (predicted_positions_.empty()) {
    return;
  }
  // Expand the update window around each predicted pose by the configured radius
  for (const auto & pred : predicted_positions_) {
    const double px = pred.pose.position.x;
    const double py = pred.pose.position.y;
    // Use the provided sigma values as the spatial extent of uncertainty.
    // A larger sigma implies a wider confidence region【567447048113654†L319-L331】.
    const double sigma_x = pred.sigma_x;
    const double sigma_y = pred.sigma_y;
    *min_x = std::min(*min_x, px - sigma_x);
    *min_y = std::min(*min_y, py - sigma_y);
    *max_x = std::max(*max_x, px + sigma_x);
    *max_y = std::max(*max_y, py + sigma_y);
  }
}

void
HumanTrajectoryLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                                  int min_i, int min_j,
                                  int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  if (predicted_positions_.empty()) {
    return;
  }
  // Iterate through the window and compute cost values based on proximity to predicted poses
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      // Convert map indices to world coordinates
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      // Compute the maximum gaussian response from all predicted positions
      double max_prob = 0.0;
      for (const auto & pred : predicted_positions_) {
        const double dx = wx - pred.pose.position.x;
        const double dy = wy - pred.pose.position.y;
        // Use anisotropic Gaussian based on sigma_x and sigma_y.  Smaller
        // sigmas yield a sharper peak and higher probability density near
        // the mean【567447048113654†L319-L331】.
        const double sigma_x = std::max(pred.sigma_x, 1e-3f);
        const double sigma_y = std::max(pred.sigma_y, 1e-3f);
        const double exponent = -0.5 * ((dx * dx) / (sigma_x * sigma_x) + (dy * dy) / (sigma_y * sigma_y));
        const double prob = std::exp(exponent);
        // Scale by confidence level so that higher confidence produces larger cost
        const double weighted_prob = prob * static_cast<double>(pred.confidence_level);
        if (weighted_prob > max_prob) {
          max_prob = weighted_prob;
        }
      }
      // Scale the probability to a cost value [0, max_cost_]
      if (max_prob > 0.0) {
        const unsigned char cost = static_cast<unsigned char>(
          std::min(max_cost_, max_prob * max_cost_));
        // Set the cost on the master grid
        master_grid.setCost(i, j, cost);
      }
    }
  }
}

void
HumanTrajectoryLayer::reset()
{
  // Clear the predicted positions when resetting the plugin
  predicted_positions_.clear();
}

}  // namespace human_trajectory_costmap_layer

// Register this plugin with pluginlib. This allows the layer to be loaded
// dynamically at runtime as a nav2_costmap_2d::Layer【635006320119696†L1105-L1112】.
PLUGINLIB_EXPORT_CLASS(human_trajectory_costmap_layer::HumanTrajectoryLayer,
                       nav2_costmap_2d::Layer)