import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

class TrackingDummy(Node):
    """
    Publishes the last-N positions (x,y) of a moving object as PoseArray.
    Modes:
      - 'linear'     : constant-velocity along start->end direction
      - 'oscillate'  : ping-pong between (start_x,start_y) and (end_x,end_y)
    Heading is derived from direction of travel (no heading parameter).
    """

    def __init__(self):
        super().__init__('tracking_dummy')

        # --- Parameters (declare + defaults) ---
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('end_x', 1.0)
        self.declare_parameter('end_y', 0.0)
        self.declare_parameter('mode', 'linear')       # 'linear' or 'oscillate'
        self.declare_parameter('speed', 0.5)           # m/s
        self.declare_parameter('window_size', 20)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('publish_topic', '/tracked_object/poses')
        self.declare_parameter('frame_id', 'map')

        # --- Read parameter values ---
        self.x0 = float(self.get_parameter('start_x').value)
        self.y0 = float(self.get_parameter('start_y').value)
        self.x1 = float(self.get_parameter('end_x').value)
        self.y1 = float(self.get_parameter('end_y').value)
        self.mode = str(self.get_parameter('mode').value)
        self.speed = float(self.get_parameter('speed').value)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('publish_topic').value)
        self.window_size = int(self.get_parameter('window_size').value)
        self.dt = 1.0 / float(self.get_parameter('publish_rate_hz').value)

        # --- Publisher & state ---
        self.pub = self.create_publisher(PoseArray, self.topic, 10)
        self.buffer = deque(maxlen=self.window_size)
        self.t0 = self.get_clock().now()

        # Segment & motion state
        self._compute_segment()
        # linear mode: velocity along segment (or +X if degenerate)
        if self.mode == 'linear':
            if self.seg_L > 0.0:
                self.vx, self.vy = self.speed * self.dir_x, self.speed * self.dir_y
            else:
                self.vx, self.vy = self.speed, 0.0
        else:
            self.vx = self.vy = 0.0  # not used in oscillate
        # oscillate progress
        self.s = 0.0
        self.sign = +1

        # Timer loop
        self.timer = self.create_timer(self.dt, self._on_timer)

        # Dynamic param updates
        self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info(
            f"tracking_dummy started: mode={self.mode}, "
            f"start=({self.x0:.2f},{self.y0:.2f}), end=({self.x1:.2f},{self.y1:.2f}), "
            f"speed={self.speed:.2f} m/s, N={self.window_size}, topic={self.topic}, "
            f"rate={1.0/self.dt:.1f} Hz"
        )

    # --- helpers ---
    def _compute_segment(self):
        self.seg_dx = self.x1 - self.x0
        self.seg_dy = self.y1 - self.y0
        self.seg_L  = math.hypot(self.seg_dx, self.seg_dy)
        if self.seg_L > 0.0:
            self.dir_x = self.seg_dx / self.seg_L
            self.dir_y = self.seg_dy / self.seg_L
        else:
            self.dir_x, self.dir_y = 1.0, 0.0  # default +X if degenerate

    def _clamp_and_bounce(self, s):
        L = self.seg_L
        sign = self.sign
        if L == 0.0:
            return 0.0, sign
        while s > L or s < 0.0:
            if s > L:
                s = 2.0 * L - s
                sign *= -1
            elif s < 0.0:
                s = -s
                sign *= -1
        return s, sign

    # --- param callback ---
    def _on_param_update(self, params):
        from rcl_interfaces.msg import SetParametersResult
        try:
            for p in params:
                if p.name == 'start_x':
                    self.x0 = float(p.value); self._compute_segment(); self.s = 0.0
                elif p.name == 'start_y':
                    self.y0 = float(p.value); self._compute_segment(); self.s = 0.0
                elif p.name == 'end_x':
                    self.x1 = float(p.value); self._compute_segment()
                elif p.name == 'end_y':
                    self.y1 = float(p.value); self._compute_segment()
                elif p.name == 'mode':
                    self.mode = str(p.value)
                    self.t0 = self.get_clock().now()
                    self.s = max(0.0, min(self.s, self.seg_L)); self.sign = +1
                elif p.name == 'speed':
                    self.speed = float(p.value)

                elif p.name == 'window_size':
                    newN = int(p.value); old = list(self.buffer)
                    self.buffer = deque(old[-newN:], maxlen=newN)

            # Recompute velocity for linear mode after any relevant change
            if self.mode == 'linear':
                if self.seg_L > 0.0:
                    self.vx, self.vy = self.speed * self.dir_x, self.speed * self.dir_y
                else:
                    self.vx, self.vy = self.speed, 0.0

            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    # --- main timer ---
    def _on_timer(self):
        now = self.get_clock().now()

        if self.mode == 'oscillate':
            ds = self.speed * self.dt * self.sign
            s_new = self.s + ds
            self.s, self.sign = self._clamp_and_bounce(s_new)
            x = self.x0 + self.dir_x * self.s
            y = self.y0 + self.dir_y * self.s
        else:
            t = (now - self.t0).nanoseconds / 1e9
            x = self.x0 + self.vx * t
            y = self.y0 + self.vy * t

        # append new point
        self.buffer.append((x, y))

        # Build PoseArray with per-pose heading from neighbor differences
        msg = PoseArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        poses = []
        n = len(self.buffer)
        for i, (xi, yi) in enumerate(self.buffer):
            # Use forward diff for first point, backward diff otherwise
            if i == 0 and n > 1:
                xj, yj = self.buffer[i+1]
                dx, dy = (xj - xi), (yj - yi)
            elif i > 0:
                xj, yj = self.buffer[i-1]
                dx, dy = (xi - xj), (yi - yj)
            else:
                dx, dy = (1.0, 0.0)  # fallback if only one point

            yaw = math.atan2(dy, dx)  # radians
            qz = math.sin(0.5 * yaw)
            qw = math.cos(0.5 * yaw)

            p = Pose()
            p.position.x = float(xi)
            p.position.y = float(yi)
            p.position.z = 0.0
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = float(qz)
            p.orientation.w = float(qw)
            poses.append(p)

        msg.poses = poses
        self.pub.publish(msg)
        

def main():
    rclpy.init()
    node = TrackingDummy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
