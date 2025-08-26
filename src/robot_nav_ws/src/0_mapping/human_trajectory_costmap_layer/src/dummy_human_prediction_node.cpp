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
/// \brief Implements a dummy prediction node that publishes synthetic human
/// trajectory predictions.  This node mimics the input and output of
/// the probabilistic trajectory prediction network described in the
/// referenced paper.  Instead of a simple PoseArray, it publishes
/// `PredictedPositionArray` messages, where each prediction contains
/// the mean pose of the pedestrian, a confidence level and
/// corresponding standard deviations.  The sequence of predicted
/// positions follows a circular trajectory with two confidence levels
/// (68 % and 95 %) to demonstrate how uncertainty grows with
/// confidence.  The Nav2 costmap layer can subscribe to this topic
/// to paint the predictions onto the costmap.

#include "rclcpp/rclcpp.hpp"
// Use custom messages to include confidence levels and uncertainties in
// each prediction.  The probabilistic trajectory prediction network
// described in the paper yields confidence sets with associated
// uncertainties【567447048113654†L319-L331】.  Our dummy publisher
// simulates such output by publishing a PredictedPositionArray.
#include "human_trajectory_costmap_layer/msg/predicted_position_array.hpp"
#include "human_trajectory_costmap_layer/msg/predicted_position.hpp"

#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

/// \class DummyHumanPredictionNode
/// \brief Node that publishes synthetic pedestrian trajectory predictions.
class DummyHumanPredictionNode : public rclcpp::Node
{
public:
  DummyHumanPredictionNode()
  : Node("dummy_human_prediction_node"),
    t_(0.0)
  {
    // Publisher for predicted positions. The costmap layer subscribes to
    // this.  We publish our custom PredictedPositionArray messages.
    pub_ = this->create_publisher<human_trajectory_costmap_layer::msg::PredictedPositionArray>(
      "predicted_human_positions", 10);
    // Timer callback to periodically publish predictions
    timer_ = this->create_wall_timer(500ms, std::bind(&DummyHumanPredictionNode::onTimer, this));
  }

private:
  /// \brief Timer callback generates and publishes a set of predicted poses.
  void onTimer()
  {
    human_trajectory_costmap_layer::msg::PredictedPositionArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    // Simulate 12 predicted future positions (4.8s at 0.4s intervals).
    const int n_predictions = 12;
    const double radius = 5.0;           // radius of the circular path
    const double angular_velocity = 0.1; // rad/s
    // Define confidence levels and associated sigma multipliers.  Lower
    // confidence corresponds to a narrower distribution, whereas higher
    // confidence corresponds to a wider confidence set【567447048113654†L319-L331】.
    struct Level { float conf; float sigma_mult; } levels[] = {
      {0.68f, 0.5f}, {0.95f, 1.0f}
    };
    for (int i = 0; i < n_predictions; ++i) {
      const double dt = static_cast<double>(i) * 0.4;  // 0.4 s per step
      const double angle = t_ * angular_velocity + dt;
      // Mean pose for this time step
      const double px = radius * std::cos(angle);
      const double py = radius * std::sin(angle);
      for (const auto & lvl : levels) {
        human_trajectory_costmap_layer::msg::PredictedPosition pred;
        pred.pose.position.x = px;
        pred.pose.position.y = py;
        pred.pose.position.z = 0.0;
        pred.pose.orientation.w = 1.0;
        pred.pose.orientation.x = 0.0;
        pred.pose.orientation.y = 0.0;
        pred.pose.orientation.z = 0.0;
        pred.confidence_level = lvl.conf;
        // Base sigma values for the 68% confidence level.  For higher
        // confidence levels we scale the sigmas accordingly.
        const float base_sigma_x = 0.3f;
        const float base_sigma_y = 0.2f;
        pred.sigma_x = base_sigma_x * lvl.sigma_mult;
        pred.sigma_y = base_sigma_y * lvl.sigma_mult;
        msg.positions.push_back(pred);
      }
    }
    pub_->publish(msg);
    // Advance time index for the next prediction
    t_ += 0.1;
  }

  rclcpp::Publisher<human_trajectory_costmap_layer::msg::PredictedPositionArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
};

/// \brief Entry point for the dummy prediction node.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyHumanPredictionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}