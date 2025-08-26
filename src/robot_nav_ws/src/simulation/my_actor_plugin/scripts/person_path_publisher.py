#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import time

class PersonPathPublisher(Node):
    def __init__(self):
        super().__init__('person_path_publisher')
        self.pub = self.create_publisher(Path, '/person_path', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        path = Path()
        path.header.frame_id = 'map'
        now = self.get_clock().now()

        for i in range(50):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            t = i * 0.1
            pose.header.stamp = now.to_msg()
            pose.pose.position.x = 3 + math.sin(0.2 * t)
            pose.pose.position.y = 3 + math.cos(0.2 * t)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = PersonPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
