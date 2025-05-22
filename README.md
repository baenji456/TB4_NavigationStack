# Basic Setup

## Switching between Setups

In this project, we use three distinct ROS 2 network setups, each configured for a specific use case.  
**Only one setup should be active in a terminal session at a time.** Mixing them will lead to communication issues.

> [!CAUTION]
> The scripts can be found in the [setup folder](setup). For reproducability of the tutorials, copy them to your host machines /etc folder.
> Also some adjustments may have to be made, especially for the discovery server setup, if not used with our Jazzy TB4

### Available Setups

| Setup Name                        | Script Path                            | Description                                                                 |
|----------------------------------|----------------------------------------|-----------------------------------------------------------------------------|
| TB4 Simple Discovery Mode        | `etc/turtlebot/setup.bash`             | For communication with the Humble TB4 using default Fast DDS discovery.     |
| TB4 Discovery Server Mode        | `etc/turtlebot_discovery/setup.bash`   | For communication with the Jazzy TB4 via a centralized Discovery Server.    |
| TB4 Simulation Mode (Localhost)  | `etc/simulation/setup.bash`            | For running simulations locally with ROS_LOCALHOST_ONLY (no network usage). |

### How to Use

Each setup script configures the necessary environment variables and unsets any conflicting ones from previous setups.

In your terminal:

```bash
source etc/turtlebot4/setup.bash            # For Humble TB4 communication
source etc/turtlebot4_discovery/setup.bash  # For Discovery Server setup
source etc/turtlebot4_simulation/setup.bash           # For running local simulation

ros2 daemon stop; ros2 daemon start        # sometimes you will have to restart, for changes to be applied
```

## Setup Docker container
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


In Docker Container:

```bash
# Build workspace from source
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
TODO: automate this with entrypoint.sh


# First steps
## Generate Map using TB4 SLAM

This is a combination of the [SLAM](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html) and [turtlebot4 simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) tutorials. Visit those websites for further information on the commands.

In separate terminals do:

```bash
# IF USING SIMULATOR: Start turtlebot4 ignition simulator
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

## Navigate a map using Nav2
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

We use RTAB-Map for Mapping in this Navigation Framework. In comparison to the standard SLAM Toolbox it is a bit more complex, but also more flexible with other sensors. While SLAM Toolbox only supports 2D LiDAR-based SLAM, RTAB-Map supports 2D and 3D LiDAR SLAM as well as visual SLAM. the full comparison ist shown in the following table:

| Aspect | RTAB-Map | SLAM Toolbox |
|:------|:---------|:-------------|
| **Type** | Visual and Lidar SLAM (2D/3D) | 2D Lidar-based SLAM |
| **Focus** | General-purpose, flexible, sensor fusion | Large, dynamic environments |
| **Mapping** | 2D/3D occupancy grids, dense point clouds | 2D occupancy grids |
| **Multi-session** | Supported; merges sessions online | Strong multi-session support, serialization |
| **Localization** | Robust localization after mapping | Pure localization with dynamic environment adaptation |
| **Loop closure** | Visual and lidar-based, memory-managed | Strong loop closure, manual graph editing |
| **Performance** | Higher compute demand, scalable with management | Highly optimized, runs on mobile CPUs |
| **ROS Integration** | ROS1 + ROS2, flexible package | ROS1 + ROS2, default in ROS2 Navigation2 |
| **Ease of Use** | Flexible but complex to tune | Simple setup, ready-to-use modes |
| **Best For** | Complex 2D/3D SLAM tasks, sensor fusion | Fast 2D mapping in large or changing spaces |

### Simple TB4 Nav2, 2D LiDAR and RGB-D SLAM

There are a lot of [sensor integration](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch) and [robot integration](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos) examples already in the ROS2 RTAB-Map package included. The following example shows the use of RTAB-Map with our Turtlebot4.

In separate Terminals do
```bash
# Launch simulator
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=false nav2:=true rviz:=true
```

```bash
# Launch SLAM
ros2 launch rtabmap_demos turtlebot4_slam.launch.py use_sim_time:=true
```

You can now steer the TB4 in the Gazebo Ignition simulation and create a map with the RTAB-Map SLAM-method.

# RoadMap

![Navigation Framework](images/Navigationsframework_Poster_breiteres_Formet.drawio.png)

#### PRIOs
- [ ] TB4 Jazzy Simulation: LiDAR Problem

#### General
- [x] Docker Environment for Humble and Jazzy
- [x] Easy End-to-End tutorials from Nav2 Website
- [ ] Migrate this repo to RIOT Gitlab

#### Simulation
- [x] Basic Turtlebot4 simulation
- [ ] Add dynamic objects to maps
- [ ] Integrate 3D LiDAR

#### Real
- [x] Communication with real TB4 using Discovery Server
- [x] Different setup.bash files for easy mode switching
- [ ] Integrate 3D LiDAR

#### Mapping and Localization
- [x] Include RTAB-Map SLAM
- [ ] RTAB-Map with 3D-Sensors

##### Sim
- [x] Basic Mapping with Nav2
- [x] Simple RTAB-Map Tutorial
##### Real
- [x] Basic mapping with Nav2
- [ ] Simple RTAB-Map

#### Global Path Planning
- [x] Nav2
- [ ] A*
- [ ] Djikstra

#### Local Path Planning
- [ ] Dynamic Window
- [ ] Artificial Potential Fields
- [ ] PFs with probabilistic predictions
