# Details

Long-form notes moved out of the README.

### Manual Launch (3 Terminals) - Alternative

**Terminal 1 - Gazebo:**

    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

**Terminal 2 - Navigation** (wait for Gazebo to load):

    export TURTLEBOT3_MODEL=burger
    cd ~/turtlebot_navigation_ws
    source install/setup.bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True

**Terminal 3 - Waypoint Navigator** (wait for "Managed nodes are active"):

    export TURTLEBOT3_MODEL=burger
    cd ~/turtlebot_navigation_ws
    source install/setup.bash
    ros2 run autonomous_navigator waypoint_navigator

## Configuration

### Waypoints

The robot navigates through 8 waypoints in TurtleBot3 World, continuously looping forever:

1. **(2.0, 0.5)** - Entry hall
2. **(2.0, -1.5)** - Living room
3. **(0.5, -2.0)** - Kitchen corridor
4. **(-1.5, -1.5)** - SW bedroom
5. **(-2.0, 0.5)** - NW storage
6. **(-1.0, 2.0)** - N bedroom
7. **(1.0, 2.0)** - Corridor
8. **(0.0, 0.0)** - Home base

After completing waypoint 8, the robot automatically returns to waypoint 1 and continues infinitely.

Edit waypoints in `maps/waypoint_navigator.py`:

    self.waypoints = [
        (2.0, 0.5, 0.0),      # (x, y, yaw in radians)
        (2.0, -1.5, -1.57),
        (0.5, -2.0, 3.14),
        (-1.5, -1.5, 2.36),
        (-2.0, 0.5, 1.57),
        (-1.0, 2.0, 0.79),
        (1.0, 2.0, 0.0),
        (0.0, 0.0, 0.0),
    ]

### Navigation Parameters

Optimized settings in `config/nav2_params.yaml`:

**Speed Settings:**
- `max_vel_x`: 0.8 m/s (3.6x faster than default)
- `max_vel_theta`: 2.5 rad/s
- `min_vel_x`: 0.0 m/s
- `min_vel_theta`: 0.4 rad/s

**Obstacle Avoidance:**
- `BaseObstacle.scale`: 0.02 (balanced avoidance)
- `inflation_radius`: 1.0 m
- `cost_scaling_factor`: 3.0

**Goal Tolerance:**
- `xy_goal_tolerance`: 0.1 m
- `yaw_goal_tolerance`: 0.1 rad


## Troubleshooting

**Robot not moving:**

    ros2 node list  # Check all nodes running
    ros2 topic list | grep cmd_vel  # Check velocity commands

**Goals rejected:**

    ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap

**First waypoints always fail:**
- Normal behavior - waypoints 1-2 often rejected as "off costmap"
- Robot automatically moves to next waypoint
- Navigation succeeds from waypoint 3 onwards

**Robot too close to obstacles:**
- Increase `inflation_radius` in nav2_params.yaml

**Robot too cautious:**
- Decrease `BaseObstacle.scale` in nav2_params.yaml

**Gazebo issues:**

    killall gzserver gzclient

## Project Structure

    autonomous_navigator/
    ├── config/
    │   └── nav2_params.yaml          # Optimized navigation parameters (0.8 m/s)
    ├── launch/
    │   └── full_navigation_launch.py # Complete system launcher
    ├── maps/
    │   └── waypoint_navigator.py     # Main navigation node (infinite loop)
    └── setup.py                      # Package setup

## Key Features Explained

### Infinite Loop Operation
The waypoint navigator runs indefinitely:
- Completes waypoints 1-8
- Automatically resets to waypoint 1
- Tracks loop count
- Continues until manual stop (Ctrl+C)

### Speed Optimization
Navigation is 3.6x faster than default TurtleBot3 settings:
- Linear velocity: 0.22 m/s → **0.8 m/s**
- Angular velocity: 2.84 rad/s → **2.5 rad/s**

### Failure Handling
- Clears costmap on first failure
- Skips waypoint after 2 consecutive failures
- Logs all attempts and successes
- Never stops due to individual waypoint failures

