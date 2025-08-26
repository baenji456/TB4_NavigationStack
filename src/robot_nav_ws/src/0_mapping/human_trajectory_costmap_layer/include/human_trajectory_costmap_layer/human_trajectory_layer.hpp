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
/// \brief Definition of the HumanTrajectoryLayer class, a custom costmap
/// plugin for Nav2 that paints predicted human positions onto the
/// costmap.  The plugin subscribes to a PredictedPositionArray of
/// probabilistic human trajectory predictions and uses anisotropic
/// Gaussian kernels around each predicted mean to assign costs within
/// the costmap.  Each prediction in the array includes a confidence
/// level and standard deviations (sigma_x, sigma_y).  The plugin
/// scales the cost according to the confidence level and spreads the
/// cost according to the sigmas, modelling the confidence sets
/// described in the paper on reliable probabilistic human trajectory
/// prediction【567447048113654†L319-L331】.

#ifndef HUMAN_TRAJECTORY_COSTMAP_LAYER__HUMAN_TRAJECTORY_LAYER_HPP_
#define HUMAN_TRAJECTORY_COSTMAP_LAYER__HUMAN_TRAJECTORY_LAYER_HPP_

#include <vector>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "human_trajectory_costmap_layer/msg/predicted_position_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace human_trajectory_costmap_layer
{

/// \class HumanTrajectoryLayer
/// \brief Custom Nav2 costmap layer for painting predicted pedestrian
/// positions onto the costmap.  The layer subscribes to a
/// `PredictedPositionArray` topic and generates costs using
/// anisotropic Gaussian kernels around each predicted mean.  Each
/// prediction carries a confidence level and standard deviations; the
/// kernel radius along x and y is derived from the sigma values and
/// the cost is scaled by the confidence level.  This allows the Nav2
/// planner to account for probabilistic human motion as dynamic
/// obstacles.
class HumanTrajectoryLayer : public nav2_costmap_2d::Layer
{
public:
  /// \brief Constructor.
  HumanTrajectoryLayer();

  /// \brief Destructor.
  ~HumanTrajectoryLayer() override;

  /// \brief Initialize the layer. Called after parameters are declared.
  void onInitialize() override;

  bool isClearable() override;

  /// \brief Update the bounds of the area that this layer needs to update.
  ///
  /// This method expands the update window to include the area around each
  /// predicted pedestrian position so that the costmap knows which
  /// regions require recalculation. The algorithm here follows the
  /// Nav2 documentation: the plugin is asked which area of the costmap
  /// needs to be updated and returns the minimum and maximum world
  /// coordinates for that window【635006320119696†L988-L1007】.
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double * min_x, double * min_y,
                    double * max_x, double * max_y) override;

  /// \brief Update the costmap within the specified bounds.
  ///
  /// This method iterates over each cell in the bounding window and
  /// computes a cost based on the proximity to predicted pedestrian
  /// positions. Costs are computed using an unnormalized Gaussian
  /// distribution and scaled to the configured maximum cost value. The
  /// resulting cost values are written directly into the master grid
  /// according to the Nav2 API【635006320119696†L998-L1007】.
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j,
                   int max_i, int max_j) override;

  /// \brief Reset the plugin. Clears internal state such as cached
  /// predicted positions.
  void reset() override;

protected:
  /// \brief Callback for predicted human positions.
  /// Copies the incoming poses into internal storage and flags the
  /// costmap for recalculation.
  void predictionCallback(const human_trajectory_costmap_layer::msg::PredictedPositionArray::SharedPtr msg);

  /// Stored list of predicted pedestrian positions with confidence and
  /// uncertainty. Updated whenever predictionCallback is invoked.
  std::vector<human_trajectory_costmap_layer::msg::PredictedPosition> predicted_positions_;

  /// Subscription to predicted positions topic.
  rclcpp::Subscription<human_trajectory_costmap_layer::msg::PredictedPositionArray>::SharedPtr sub_;

  /// Gaussian radius (legacy).  This parameter is kept for backwards
  /// compatibility but is no longer used; the spatial extent of each
  /// prediction is instead derived from the sigma_x and sigma_y
  /// values contained in the prediction message.
  double gaussian_radius_;

  /// Maximum cost value to assign within the Gaussian kernel. Must be
  /// between 0 and 254; higher values indicate stronger obstacles.
  double max_cost_;
};

}  // namespace human_trajectory_costmap_layer

#endif  // HUMAN_TRAJECTORY_COSTMAP_LAYER__HUMAN_TRAJECTORY_LAYER_HPP_