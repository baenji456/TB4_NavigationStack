ros2 launch turtlebot4_ignition_bringup tb4_ign_nav2.launch.py

ros2 launch mdn_prediction dummy_prediction_pipeline.launch.py   params_file:=src/3_prediction/mdn_prediction/config/dummy_prediction_pipeline.yaml


# Start Basic HuNavSim
ros2 launch hunav_gazebo_fortress_wrapper simulation_fortress.launch.py environment_name:=cafe configuration_file:=agents_cafe.yaml
