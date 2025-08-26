# launch/image_subscriber_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_detection_node',
            output='screen'
        )
    ])
