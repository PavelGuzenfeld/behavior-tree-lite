# ROS 2 example

`examples/patrol_robot_node.cpp` is a robot patrolling a cross pattern, with
a battery, a laser and an emergency stop. Nav2 and odometry are left out:
the node integrates its own pose so RViz has something to draw.

```bash
colcon build --packages-select behavior_tree_lite
source install/setup.bash
ros2 launch behavior_tree_lite patrol_demo.launch.py
```

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
