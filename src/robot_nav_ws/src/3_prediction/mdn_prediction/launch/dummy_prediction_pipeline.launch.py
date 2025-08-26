from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mdn_prediction')
    default_params = os.path.join(pkg_share, 'config', 'prediction_demo.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with parameters for ALL nodes started here'
        ),

        # --- Tracking dummy (object_tracking) ---
        Node(
            package='object_tracking',
            executable='dummy_tracker_node',      # falls dein console_script anders heißt (z.B. dummy_tracker_node), hier anpassen
            name='dummy_tracker_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),

        # --- Dummy MDN Predictor ---
        Node(
            package='mdn_prediction',
            executable='dummy_mdn_pred_node',  # ggf. auf dummy_mdn_pred_node anpassen
            name='dummy_mdn_pred_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),

        # --- MDN Sampler (Single) ---
        Node(
            package='mdn_prediction',
            executable='mdn_sampler',
            name='mdn_sampler_single',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),

        # --- MDN Sampler (Array) ---
        Node(
            package='mdn_prediction',
            executable='mdn_sampler',
            name='mdn_sampler_array',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
