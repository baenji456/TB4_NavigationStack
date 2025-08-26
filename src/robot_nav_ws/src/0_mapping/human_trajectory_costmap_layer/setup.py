#!/usr/bin/env python3
# Copyright 2025 Example User
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS 2 node to simulate probabilistic human trajectory forecasting.

This node publishes synthetic predictions that mimic the output of the
Mixture Density Network (MDN) described in the associated paper on
reliable probabilistic human trajectory prediction.  The true
inference pipeline would take a sequence of past positions and feed
them through a stacked LSTM with an MDN head to obtain a mixture of
Gaussian distributions for future time steps【735962216640055†L133-L146】.  Each
mixture component yields a mean position, anisotropic covariances and
mixing weights.  Downstream tasks then derive confidence sets (e.g.,
68 % or 95 %) from these distributions by evaluating the cumulative
density function【735962216640055†L279-L320】.  Instead of loading a trained model
and performing an actual forward pass, this node synthesizes
predictions for demonstration purposes.  It generates a simple motion
pattern and assigns increasing uncertainty as the forecast horizon
grows, exposing multiple confidence levels per prediction.  The
resulting messages are of type `PredictedPositionArray`, with each
`PredictedPosition` containing a mean pose, a confidence level and
anisotropic standard deviations along the x and y axes.  The Nav2
costmap plugin in this package subscribes to the topic published by
this node and paints Gaussian cost kernels into the costmap according
to the provided mean, confidence and sigma values【735962216640055†L279-L320】.
"""

import math
from typing import List

try:
    # Matplotlib is used for visualisation if enabled.  We import it
    # conditionally so that the node can run on systems without a
    # display.  If the import fails, visualisation will be disabled
    # automatically.
    import matplotlib
    # Use a non‑interactive backend if no display is available.  This
    # prevents Matplotlib from trying to connect to an X server when
    # running headless.  Users can override the backend by setting
    # MPLBACKEND in the environment.
    if not matplotlib.get_backend().lower().startswith('agg'):
        # switch to Agg if DISPLAY is not set
        import os
        if os.environ.get('DISPLAY', '') == '':
            matplotlib.use('Agg')
    import matplotlib.pyplot as plt  # noqa: E402
    _MATPLOTLIB_AVAILABLE = True
except Exception:  # pragma: no cover
    # Fall back gracefully if Matplotlib cannot be imported
    _MATPLOTLIB_AVAILABLE = False

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from human_trajectory_costmap_layer.msg import PredictedPosition, PredictedPositionArray


class MdnDummyPredictionNode(Node):
    """Publish dummy human trajectory predictions with uncertainty."""

    def __init__(self) -> None:
        super().__init__('mdn_dummy_prediction_node')
        # Declare parameters to configure the simulation.  The forecast
        # horizon and time step define how many future positions are
        # emitted and at what temporal resolution.  Confidence levels
        # correspond to the quantiles of the predicted distributions; the
        # default values reflect the one and two sigma levels of a
        # Gaussian【735962216640055†L279-L320】.  Base sigma values set the spread for
        # the first horizon and are scaled linearly with the forecast
        # horizon and a multiplier derived from the confidence level.
        self.declare_parameter('forecast_horizon', 12)
        self.declare_parameter('delta_t', 0.4)
        self.declare_parameter('confidence_levels', [0.68, 0.95])
        self.declare_parameter('base_sigma_x', 0.3)
        self.declare_parameter('base_sigma_y', 0.2)
        self.declare_parameter('motion_model', 'line')
        self.declare_parameter('velocity', 0.5)
        self.declare_parameter('publish_frequency', 2.0)
        # Control whether predictions should be published on a topic and/or
        # visualised locally.  Setting these parameters allows the user
        # to choose between publishing only, visualising only or doing
        # both.  Both are enabled by default.
        self.declare_parameter('publish_enabled', True)
        self.declare_parameter('visualize_enabled', False)

        # Read back parameters
        self.forecast_horizon: int = self.get_parameter('forecast_horizon').get_parameter_value().integer_value
        self.delta_t: float = self.get_parameter('delta_t').get_parameter_value().double_value
        # Fetch confidence levels as a list of doubles.  ROS parameters
        # store arrays generically; we convert them to floats here.
        conf_param = self.get_parameter('confidence_levels').get_parameter_value()
        if conf_param.type_ == conf_param.TYPE_DOUBLE_ARRAY:
            self.confidence_levels: List[float] = list(conf_param.double_array_value)
        elif conf_param.type_ == conf_param.TYPE_INTEGER_ARRAY:
            self.confidence_levels = [float(x) for x in conf_param.integer_array_value]
        else:
            # Fallback to default 68 % and 95 % confidence levels
            self.confidence_levels = [0.68, 0.95]
        self.base_sigma_x: float = self.get_parameter('base_sigma_x').get_parameter_value().double_value
        self.base_sigma_y: float = self.get_parameter('base_sigma_y').get_parameter_value().double_value
        self.motion_model: str = self.get_parameter('motion_model').get_parameter_value().string_value
        self.velocity: float = self.get_parameter('velocity').get_parameter_value().double_value
        self.publish_frequency: float = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Determine whether publishing and/or visualisation are enabled.
        publish_enabled_param = self.get_parameter('publish_enabled').get_parameter_value()
        self.publish_enabled: bool = bool(publish_enabled_param.bool_value) if publish_enabled_param.type_ == publish_enabled_param.TYPE_BOOL else True
        visualize_enabled_param = self.get_parameter('visualize_enabled').get_parameter_value()
        self.visualize_enabled: bool = bool(visualize_enabled_param.bool_value) if visualize_enabled_param.type_ == visualize_enabled_param.TYPE_BOOL else False

        # Publisher for PredictedPositionArray messages.  Other nodes can
        # subscribe to this topic to receive the dummy predictions.  The
        # queue size of 10 is sufficient for low publish rates.  Only
        # create the publisher if publishing is enabled; otherwise this
        # attribute will remain None and no messages will be sent.
        self.publisher_ = None
        if self.publish_enabled:
            self.publisher_ = self.create_publisher(
                PredictedPositionArray,
                'predicted_human_positions',
                10)

        # Timer to periodically invoke the prediction routine.  The
        # period is the reciprocal of the publish frequency; ensure it
        # defaults to a sane value (> 0).
        period = 1.0 / max(self.publish_frequency, 1e-3)
        self.timer_ = self.create_timer(period, self.timer_callback)

        # Internal time index.  Used only by the circular motion model.
        self.t_ = 0.0

        # Setup visualisation if enabled and Matplotlib is available.  We
        # initialise a figure and axis here so that they persist
        # between timer callbacks.  If visualisation is enabled but
        # Matplotlib could not be imported, we warn the user and
        # disable visualisation.
        self.figure = None
        self.ax = None
        if self.visualize_enabled:
            if not _MATPLOTLIB_AVAILABLE:
                self.get_logger().warn(
                    'Visualisation requested but Matplotlib is not available; disabling visualisation.')
                self.visualize_enabled = False
            else:
                # Enable interactive mode so the figure updates on each
                # timer tick without blocking the ROS thread
                plt.ion()
                self.figure, self.ax = plt.subplots(figsize=(6, 6))
                self.ax.set_title('MDN Dummy Predictions')
                self.ax.set_xlabel('x position [m]')
                self.ax.set_ylabel('y position [m]')

        self.get_logger().info(
            f"MDN dummy prediction node initialised with horizon={self.forecast_horizon}, "
            f"dt={self.delta_t}, confidence_levels={self.confidence_levels}, "
            f"base_sigma=({self.base_sigma_x},{self.base_sigma_y}), motion_model={self.motion_model}, "
            f"velocity={self.velocity}, publish_frequency={self.publish_frequency}, "
            f"publish_enabled={self.publish_enabled}, visualize_enabled={self.visualize_enabled}")

    def timer_callback(self) -> None:
        """Create and publish a set of dummy trajectory predictions."""
        # Prepare a container for all predicted positions so that we
        # can both publish and visualise the same data without
        # recomputing it twice.
        predicted_positions: List[PredictedPosition] = []
        # Compute predictions for each horizon
        for horizon_index in range(self.forecast_horizon):
            time_offset = (horizon_index + 1) * self.delta_t
            # Determine the mean of the future position based on the
            # selected motion model
            if self.motion_model == 'line':
                px = self.velocity * time_offset
                py = 0.0
            elif self.motion_model == 'circle':
                radius = 2.0
                omega = self.velocity / max(radius, 1e-3)
                angle = self.t_ + time_offset * omega
                px = radius * math.cos(angle)
                py = radius * math.sin(angle)
            else:
                px = 0.0
                py = 0.0

            for conf in self.confidence_levels:
                sigma_mult = 1.0
                if conf >= 0.9:
                    sigma_mult = 2.0
                elif conf >= 0.68:
                    sigma_mult = 1.0
                sigma_x = self.base_sigma_x * (horizon_index + 1) * sigma_mult
                sigma_y = self.base_sigma_y * (horizon_index + 1) * sigma_mult
                pred = PredictedPosition()
                pred.pose.position.x = float(px)
                pred.pose.position.y = float(py)
                pred.pose.position.z = 0.0
                pred.pose.orientation.w = 1.0
                pred.pose.orientation.x = 0.0
                pred.pose.orientation.y = 0.0
                pred.pose.orientation.z = 0.0
                pred.confidence_level = float(conf)
                pred.sigma_x = float(sigma_x)
                pred.sigma_y = float(sigma_y)
                predicted_positions.append(pred)

        # Publish predictions if enabled
        if self.publish_enabled and self.publisher_ is not None:
            msg = PredictedPositionArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.positions = predicted_positions
            self.publisher_.publish(msg)

        # Visualise predictions if enabled.  Draw the mean positions as
        # points; uncertainty could optionally be represented as
        # ellipses but is omitted here for clarity.  We group points by
        # confidence level so that different confidences can be coloured
        # differently.
        if self.visualize_enabled and self.ax is not None:
            # Clear existing data
            self.ax.cla()
            self.ax.set_title('MDN Dummy Predictions')
            self.ax.set_xlabel('x position [m]')
            self.ax.set_ylabel('y position [m]')
            # Extract positions grouped by confidence for colouring
            grouped = {}
            for pred in predicted_positions:
                cl = pred.confidence_level
                grouped.setdefault(cl, []).append(pred)
            colours = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
            for idx, (cl, preds) in enumerate(sorted(grouped.items(), reverse=True)):
                xs = [p.pose.position.x for p in preds]
                ys = [p.pose.position.y for p in preds]
                label = f'{cl*100:.0f}%'
                colour = colours[idx % len(colours)]
                self.ax.scatter(xs, ys, c=colour, label=label)
            self.ax.legend(loc='upper left')
            # Draw and flush the figure
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()

        # Advance internal time index
        self.t_ += self.delta_t


def main(args: List[str] | None = None) -> None:
    """Entry point for the MDN dummy prediction node."""
    rclpy.init(args=args)
    node = MdnDummyPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()