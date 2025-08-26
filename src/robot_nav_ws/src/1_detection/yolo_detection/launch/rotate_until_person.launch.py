from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory('yolo_detection')

    bt_xml_file = "/home/appuser/src/robot_nav_ws/src/behavior_trees/rotate_person_detect.xml"
    plugins_xml = os.path.join(package_share_dir, 'resource', 'yolo_detection_plugins.xml')

    # Nav2 Behavior Tree engine node (BT Navigator) or your own BT engine node
    bt_node = Node(
        package='nav2_behavior_tree',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'default_bt_xml_filename': bt_xml_file,
            'plugin_lib_names': plugins_xml,  # Register plugin XML here if your engine supports it
        }]
    )

    return LaunchDescription([
        bt_node
    ])
