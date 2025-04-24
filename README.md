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

### Inside Docker Container
#### Generate Map using SLAM

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
> - Map isnt big enough (this one is speculation by now, will be looking further into this)

## Training:
### Train Semantic Kitti
