### Setup Docker Container
In docker-compse.yaml all parameters are defined.
```bash
# Enable xhost in the terminal
sudo xhost +

# Add user to environment
sh setup.sh

# Build the image from scratch using Dockerfile, can be skipped if image already exists or is loaded from docker registry
docker-compose build --no-cache

# Start the container
docker-compose up -d

# Stop the container
docker compose down
```
> [!CAUTION]
> xhost + is not a save operation!

## First steps
### Generate Map using SLAM

This is a combination of the [SLAM](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html) and [turtlebot4 simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) tutorials. Visit those websites for further information on the commands.

In separate terminals do:

```bash
# Start turtlebot4 ignition simulator
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py 
```

```bash
# Start SLAM
ros2 launch turtlebot4_navigation slam.launch.py
```

```bash
# Start RVIZ
# You can also start rviz by adding "rviz:=true" to the simulation startup
ros2 launch turtlebot4_viz view_robot.launch.py
```

```bash
# Drive around the simulated world to create a map
# Save map
# TODO: Save map to specific path
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```
> [!WARNING]
> Saving map doesnt always work. Some, but not all reasons can be:
> - File name already exists
> - ROS environment is not sourced
> - Map isnt done generating (wait a few seconds after last robot movement)

### Navigate a map using Nav2
This is based on the Turtlebot4 [navigation tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)

```bash
# Start Localization
ros2 launch turtlebot4_navigation localization.launch.py map:=map_name.yaml
```

```bash
# Start Nav2
ros2 launch turtlebot4_navigation nav2.launch.py
```

```bash
# Start RVIZ
# You can also start rviz by adding "rviz:=true" to the simulation startup
ros2 launch turtlebot4_viz view_robot.launch.py
```

In RVIZ:
- Set initial pose with "2D Pose Estimate"
- Set Nav2 Goal with "Nav2 Goal"
- Nav2 stack will plan a path to the goal pose and attempt to drive the robot there
> [!NOTE]
> Click for position and drag for orientation


### Navigation while mapping
ToDO: Make this example, based on [this tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)


# Mapping
## Mapping with RTAB-Map
### Simple TB4 Nav2, 2D LiDAR and RGB-D SLAM
```bash
# Launch simulator
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=false nav2:=true rviz:=true
```

```bash
# Launch SLAM
ros2 launch rtabmap_demos turtlebot4_slam.launch.py use_sim_time:=true
```

# RoadMap
## General
- [x] Easy End-to-End tutorials from Nav2 Website
## Simulation
- [x] Basic Turtlebot4 simulation
- [ ] Add dynamic objects to maps
- [ ] Add new sensors to TB4
## Mapping and Localization
- [x] Include RTAB-Map
- [x] Simple RTAB-Map Tutorial
- [ ] RTAB-Map with 3D-Sensors
