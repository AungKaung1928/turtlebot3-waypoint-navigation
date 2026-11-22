# TurtleBot3 Waypoint Navigation

A TurtleBot3 burger loops 8 waypoints in the Gazebo TurtleBot3 world with Nav2 and SLAM Toolbox (map built while driving), until you stop it. ROS 2 Humble, Python.

> **Simulator note (2026-09-05):** this project targets **Gazebo Classic 11** (`turtlebot3_gazebo`), which reached end-of-life in January 2025, and it is kept as-is rather than ported. For the same TurtleBot3 setup on **Gazebo Harmonic** (`ros_gz`), see [turtlebot3-wall-follower](https://github.com/AungKaung1928/turtlebot3-wall-follower).

## Quick start

Requires Ubuntu 22.04, ROS 2 Humble and Gazebo Classic 11 (`sudo apt install ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs`).

```bash
mkdir -p ~/turtlebot_navigation_ws/src && cd ~/turtlebot_navigation_ws/src
git clone <this repo URL>
cd .. && colcon build --symlink-install && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch autonomous_navigator full_navigation_launch.py   # Gazebo + Nav2 + SLAM + navigator; Ctrl+C stops the loop
```

A 3-terminal launch (Gazebo, Nav2, navigator separately) is in [docs/DETAILS.md](docs/DETAILS.md).

## How it works

- `autonomous_navigator/waypoint_navigator.py` sends the 8 waypoints (hard-coded in the node) to Nav2 in order and starts again at the first.
- A goal that times out clears the local costmap and counts as a failure; after 2 failures the waypoint is skipped.
- Nav2 speed caps in `autonomous_navigator/config/nav2_params.yaml`: 0.8 m/s, 2.5 rad/s (simulation only; the real burger tops out at 0.22 m/s).

## Limits

Observed in earlier runs, not recorded by a script: waypoints 1-2 are often rejected as off the costmap and skipped, and a loop takes about 135 s. No measured success rate.

More: waypoint and Nav2 configuration, troubleshooting and the file layout in [docs/DETAILS.md](docs/DETAILS.md).

## License

MIT, see LICENSE.
