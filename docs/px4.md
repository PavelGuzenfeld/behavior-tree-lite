# PX4 SITL example

`examples/px4_vehicle_node.cpp` flies a four-waypoint mission in PX4 SITL: arm, take off, fly the square, land. A geofence and two battery thresholds can cut in at any point.

## Prerequisites

### 1. PX4-Autopilot

```bash
# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install dependencies
bash ./Tools/setup/ubuntu.sh

# Build SITL with Gazebo Classic
make px4_sitl gazebo-classic
```

### 2. Micro-XRCE-DDS Agent

PX4 communicates with ROS2 via Micro-XRCE-DDS:

```bash
# Install from source
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Or via snap:
```bash
sudo snap install micro-xrce-dds-agent --edge
```

### 3. px4_msgs Package

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build --packages-select px4_msgs
source install/setup.bash
```

### 4. Build behavior_tree_lite with PX4 Support

```bash
cd ~/ros2_ws
colcon build --packages-select behavior_tree_lite
source install/setup.bash
```

## Running the Demo

### Terminal 1: Start PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

Wait for "Ready for takeoff!".

### Terminal 2: Start Micro-XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

Wait for "Session established".

### Terminal 3: Launch the Behavior Tree Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch behavior_tree_lite px4_sitl_demo.launch.py
```

## Behavior Tree Structure

```
Selector (Root)
├── Sequence (Emergency)
│   ├── OR
│   │   ├── Inverter(CheckGeofence)    # Geofence violation
│   │   └── Inverter(CheckBattery 15%) # Critical battery
│   └── EmergencyLand
│
├── Sequence (Low Battery RTL)
│   ├── Inverter(CheckBattery 25%)
│   └── ReturnToLaunch
│
├── Sequence (Mission)
│   ├── CheckBattery 25%
│   ├── CheckGeofence
│   ├── Selector (Ensure Armed)
│   │   ├── CheckArmed
│   │   └── Sequence
│   │       ├── CheckPreflightOk
│   │       └── Arm
│   ├── Selector (Ensure Airborne)
│   │   ├── CheckAirborne
│   │   └── Sequence
│   │       ├── EnableOffboard
│   │       └── Takeoff
│   └── Selector (Navigate/Avoid)
│       ├── Sequence
│       │   ├── CheckObstacleClear
│       │   └── NavigateToWaypoint
│       └── AvoidObstacle
│
├── Sequence (Mission Complete)
│   ├── CheckMissionComplete
│   ├── Land
│   └── Disarm
│
└── Idle
```

## DSL Code

```cpp
auto tree =
    // Emergency: geofence violation OR critical battery
    ((!CheckGeofence{} || !CheckBattery{0.15f}) && EmergencyLand{})
    ||
    // Low battery: RTL
    (!CheckBattery{0.25f} && ReturnToLaunch{})
    ||
    // Normal mission
    (CheckBattery{0.25f} && CheckGeofence{} &&
        // Ensure armed
        (CheckArmed{} || (CheckPreflightOk{} && Arm{})) &&
        // Ensure airborne
        (CheckAirborne{} || (EnableOffboard{} && Takeoff{})) &&
        // Navigate or avoid
        ((CheckObstacleClear{} && NavigateToWaypoint{}) || AvoidObstacle{}))
    ||
    // Mission complete
    (CheckMissionComplete{} && Land{} && Disarm{})
    ||
    // Fallback
    Idle{};
```

## Testing Scenarios

### Simulate Low Battery

```bash
ros2 topic pub /fmu/out/battery_status px4_msgs/msg/BatteryStatus \
  "{voltage_v: 14.0, remaining: 0.20, current_a: 5.0}" --once
```

### Monitor Status

```bash
# Watch behavior tree status
ros2 topic echo /vehicle/bt_status

# Watch vehicle status
ros2 topic echo /fmu/out/vehicle_status
```

## Customizing the Mission

Edit the `setup_mission()` function in `px4_vehicle_node.cpp`:

```cpp
void setup_mission()
{
    ctx_.waypoints = {
        {20.0f, 0.0f, -10.0f, 0.0f, "WP1_East"},
        {20.0f, 20.0f, -10.0f, M_PI_2, "WP2_NE"},
        {0.0f, 20.0f, -10.0f, M_PI, "WP3_North"},
        {0.0f, 0.0f, -10.0f, -M_PI_2, "WP4_Home"},
    };
    ctx_.current_waypoint = 0;
    ctx_.mission_complete = false;
}
```

Waypoints are `{x, y, z, yaw, name}` in local NED metres: x north, y east, z down, so `-10` is 10 m up.

## Tuning

These are fields of the context struct in `px4_vehicle_node.cpp`, not ROS parameters. Change them in the source and rebuild.

| Field | Default | Description |
|-----------|---------|-------------|
| `geofence_radius` | 100m | Max horizontal distance from home |
| `geofence_max_alt` | 50m | Maximum altitude AGL |
| `battery_critical` | 15% | Emergency land threshold |
| `battery_low` | 25% | RTL threshold |
| `waypoint_radius` | 1.5m | Waypoint acceptance radius |
| `obstacle_threshold` | 3m | Obstacle avoidance trigger (no obstacle source is wired in yet, so this branch never triggers) |
| `takeoff_alt` | -10 (NED) | Takeoff altitude, 10 m up |

## Adding Custom Nodes

```cpp
struct MyCustomCheck : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    Status process(Event const& e, VehicleContext& ctx)
    {
        ctx.active_node = "MyCustomCheck";
        
        // Your logic here
        if (some_condition)
            return Status::Success;
        return Status::Failure;
    }
    
};
```

## Troubleshooting

### "No executable found"

px4_msgs has to be built and sourced:
```bash
colcon build --packages-select px4_msgs behavior_tree_lite
source install/setup.bash
```

### "Session not established"

1. Check PX4 SITL is running
2. Verify Micro-XRCE-DDS Agent is on correct port (8888)
3. Check `XRCE_DOMAIN_ID_OVERRIDE` environment variable

### Vehicle doesn't arm

1. Check "pre_flight_checks_pass" in vehicle_status
2. Wait for GPS lock, about 30 s after SITL starts
3. Check RC connection in QGroundControl

### Offboard mode rejected

PX4 only accepts offboard mode once setpoints are streaming. `EnableOffboard` sends 10 setpoints at the current position first, then switches.

## References

- [PX4 User Guide](https://docs.px4.io/)
- [PX4 ROS2 Interface](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [Micro-XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/)
