# TurtleBot3 Autonomous Navigation

ROS2 autonomous navigation system for TurtleBot3 with waypoint navigation and obstacle avoidance.

## Overview

This project implements autonomous navigation for TurtleBot3 in Gazebo simulation. The robot follows predefined waypoints while avoiding obstacles using Nav2 stack and SLAM.

## Features

- Autonomous waypoint navigation
- Real-time obstacle avoidance
- SLAM mapping
- Recovery behaviors when stuck
- RViz visualization

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- TurtleBot3 packages
- Nav2 navigation stack

## Installation

Install dependencies:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-* ros-humble-cartographer ros-humble-cartographer-ros
```

Set up environment:

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

Build the project:

```bash
mkdir -p ~/turtlebot_navigation_ws/src
cd ~/turtlebot_navigation_ws/src
# Place autonomous_navigator package here
cd ~/turtlebot_navigation_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Quick Start

```bash
cd ~/turtlebot_navigation_ws
source install/setup.bash
ros2 launch autonomous_navigator full_navigation_launch.py
```

### Manual Launch (3 terminals)

**Terminal 1 - Gazebo:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Navigation** (wait for Gazebo to load):

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
```

**Terminal 3 - Waypoint Navigator** (wait for "Managed nodes are active"):

```bash
export TURTLEBOT3_MODEL=burger
ros2 run autonomous_navigator waypoint_navigator
```

## Configuration

### Waypoints

Edit waypoints in `maps/waypoint_navigator.py`:

```python
self.waypoints = [
    (1.8, 0.0, 0.0),      # (x, y, yaw in radians)
    (1.8, 1.8, 1.57),
    (0.0, 1.8, 3.14),
    # add more waypoints...
]
```

### Navigation Parameters

Key settings in `config/nav2_params.yaml`:

- `robot_radius`: Safety radius around robot
- `inflation_radius`: How far to inflate obstacles  
- `xy_goal_tolerance`: Position accuracy needed
- `BaseObstacle.scale`: Obstacle avoidance strength

## Default Path

The robot follows these waypoints in a rectangular path:

1. (1.8, 0.0) - Forward
2. (1.8, 1.8) - Top-right corner
3. (0.0, 1.8) - Top center
4. (-1.8, 1.8) - Top-left corner
5. (-1.8, 0.0) - Left center
6. (-1.8, -1.8) - Bottom-left corner
7. (0.0, -1.8) - Bottom center
8. (1.8, -1.8) - Bottom-right corner
9. (1.0, 0.0) - Intermediate position
10. (0.0, 0.0) - Back to start

## Troubleshooting

**Robot not moving:**

```bash
ros2 node list  # Check all nodes running
ros2 topic list | grep cmd_vel  # Check velocity commands
```

**Goals rejected:**

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

**Robot too close to obstacles:**

- Increase `robot_radius` and `inflation_radius` in nav2_params.yaml

**Robot too cautious:**

- Decrease `BaseObstacle.scale` in nav2_params.yaml

**Gazebo issues:**

```bash
killall gzserver gzclient
```

## Project Structure

```
autonomous_navigator/
├── config/nav2_params.yaml          # Navigation settings
├── launch/
│   ├── full_navigation_launch.py    # Complete system launcher
│   ├── gazebo_world_launch.py       # Gazebo only
│   └── navigation_launch.py         # Nav2 + RViz
├── maps/
│   ├── goal_publisher.py            # Single goal utility
│   └── waypoint_navigator.py        # Main navigation node
└── setup.py                         # Package setup
```
