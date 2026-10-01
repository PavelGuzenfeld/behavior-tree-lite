# ROS 2 example

`examples/bt_ros2_examples/src/patrol_robot_node.cpp` is a robot patrolling a cross pattern, with
a battery, a laser and an emergency stop. Nav2 and odometry are left out:
the node integrates its own pose so RViz has something to draw.

```bash
colcon build --base-paths src/behavior-tree-lite src/behavior-tree-lite/examples/bt_ros2_examples --packages-select behavior_tree_lite bt_ros2_examples
source install/setup.bash
ros2 launch bt_ros2_examples patrol_demo.launch.py
```

The examples are their own package, `bt_ros2_examples`, inside the library's
directory. colcon does not look inside a package for other packages, so name
both paths with `--base-paths`. The library on its own builds with a plain
`colcon build --packages-select behavior_tree_lite`.

It shows:

- several event types (`TickEvent`, `BatteryUpdate`, `LaserUpdate`,
  `EmergencyStop`) fed from subscriptions and a timer
- a tree written with the operators
- RViz markers and TF for the robot and the active node
- a status display in the terminal

## The tree

```cpp
auto tree =
    (!CheckEmergency{} && Halt{})
    || (!CheckBattery{} && GoToCharger{} && Charge{})
    || (CheckBattery{} && ((CheckObstacle{} && Navigate{}) || Avoid{}))
    || Idle{};
```

Each line is a priority: emergency stop, then low battery, then patrol, then
idle.

## Poking it

```bash
ros2 topic pub /battery std_msgs/Float32 "{data: 15.0}" --once
ros2 topic pub /scan sensor_msgs/LaserScan "{ranges: [0.3]}" --once
ros2 topic pub /estop std_msgs/Bool "{data: true}" --once
```
