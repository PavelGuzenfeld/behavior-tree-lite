/// @file px4_drone_node.cpp
/// @brief PX4 SITL Behavior Tree Integration Example
///
/// Autonomous drone with:
/// - Waypoint navigation
/// - Battery management & RTL
/// - Geofence monitoring
/// - Obstacle avoidance
/// - Emergency handling
///
/// Requires: px4_msgs, px4_ros_com, PX4-Autopilot SITL

#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/debug.hpp"
#include "behavior_tree_lite/dsl.hpp"

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <string>
#include <variant>
#include <vector>

using namespace bt;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// ============================================================================
// EVENTS
// ============================================================================

struct TickEvent
{
    double dt = 0.1;
};

struct BatteryUpdate
{
    float voltage;
    float remaining; // 0.0 - 1.0
    float current;
};

struct PositionUpdate
{
    double lat;
    double lon;
    float alt;
    float relative_alt;
};

struct LocalPositionUpdate
{
    float x, y, z;
    float vx, vy, vz;
};

struct VehicleStatusUpdate
{
    uint8_t arming_state;
    uint8_t nav_state;
    bool pre_flight_checks_pass;
};

struct ObstacleUpdate
{
    float min_distance;
    float angle; // Direction of closest obstacle
};

using Event = std::variant<
    TickEvent,
    BatteryUpdate,
    PositionUpdate,
    LocalPositionUpdate,
    VehicleStatusUpdate,
    ObstacleUpdate>;

// ============================================================================
// CONTEXT (Drone Blackboard)
// ============================================================================

struct Waypoint
{
    float x, y, z; // NED coordinates (z is negative up)
    float yaw;
    std::string name;
};

struct DroneContext
{
    // Vehicle state
    uint8_t arming_state = 0;
    uint8_t nav_state = 0;
    bool pre_flight_ok = false;

    // Position (NED frame)
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;

    // Global position
    double lat = 0.0, lon = 0.0;
    float alt = 0.0f, rel_alt = 0.0f;

    // Battery
    float battery_voltage = 16.8f;
    float battery_remaining = 1.0f;
    float battery_current = 0.0f;

    // Obstacle avoidance
    float obstacle_distance = 100.0f;
    float obstacle_angle = 0.0f;

    // Mission state
    std::vector<Waypoint> waypoints;
    size_t current_waypoint = 0;
    bool mission_complete = false;

    // Geofence (simple cylinder)
    float geofence_radius = 100.0f; // meters
    float geofence_max_alt = 50.0f; // meters AGL
    float geofence_min_alt = 2.0f;  // meters AGL

    // Thresholds
    float battery_critical = 0.15f;  // 15%
    float battery_low = 0.25f;       // 25%
    float waypoint_radius = 1.5f;    // meters
    float obstacle_threshold = 3.0f; // meters
    float takeoff_alt = -10.0f;      // NED (negative = up)

    // Home position (set on arm)
    float home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;
    bool home_set = false;

    // Command outputs (read by ROS node)
    TrajectorySetpoint setpoint{};
    VehicleCommand pending_command{};
    bool has_pending_command = false;

    // Active node tracking
    std::string active_node;
    std::deque<std::string> log_buffer;

    // Offboard control
    uint64_t offboard_setpoint_counter = 0;
    bool offboard_enabled = false;

    void log(std::string const &msg)
    {
        log_buffer.push_back(msg);
        if (log_buffer.size() > 15)
        {
            log_buffer.pop_front();
        }
    }

    void send_command(uint32_t cmd, float p1 = 0.0f, float p2 = 0.0f,
                      float p3 = 0.0f, float p4 = 0.0f, float p5 = 0.0f,
                      float p6 = 0.0f, float p7 = 0.0f)
    {
        pending_command.command = cmd;
        pending_command.param1 = p1;
        pending_command.param2 = p2;
        pending_command.param3 = p3;
        pending_command.param4 = p4;
        pending_command.param5 = p5;
        pending_command.param6 = p6;
        pending_command.param7 = p7;
        pending_command.target_system = 1;
        pending_command.target_component = 1;
        pending_command.source_system = 1;
        pending_command.source_component = 1;
        pending_command.from_external = true;
        has_pending_command = true;
    }

    [[nodiscard]] float distance_to(float tx, float ty, float tz) const
    {
        float const dx = tx - x;
        float const dy = ty - y;
        float const dz = tz - z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    [[nodiscard]] float horizontal_distance_to(float tx, float ty) const
    {
        float const dx = tx - x;
        float const dy = ty - y;
        return std::sqrt(dx * dx + dy * dy);
    }

    [[nodiscard]] bool is_armed() const
    {
        return arming_state == VehicleStatus::ARMING_STATE_ARMED;
    }

    [[nodiscard]] bool is_airborne() const
    {
        return -z > 1.0f && is_armed(); // NED: negative z = up
    }

    [[nodiscard]] bool in_geofence() const
    {
        float const horiz_dist = std::sqrt(x * x + y * y);
        float const agl = -z; // Convert NED to altitude
        return horiz_dist < geofence_radius &&
               agl < geofence_max_alt &&
               agl > 0.0f;
    }
};

// ============================================================================
// PX4 COMMAND CONSTANTS
// ============================================================================

namespace px4_cmd
{
    constexpr uint32_t ARM = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    constexpr uint32_t TAKEOFF = VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    constexpr uint32_t LAND = VehicleCommand::VEHICLE_CMD_NAV_LAND;
    constexpr uint32_t RTL = VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
    constexpr uint32_t SET_MODE = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    constexpr uint32_t OFFBOARD = 6; // Custom mode for offboard
}

// ============================================================================
// CONDITION NODES
// ============================================================================

struct CheckArmed : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    Status process(Event const &e, DroneContext &ctx)
    {
        std::visit(overloaded{[&](VehicleStatusUpdate const &s)
                              {
                                  ctx.arming_state = s.arming_state;
                                  ctx.nav_state = s.nav_state;
                                  ctx.pre_flight_ok = s.pre_flight_checks_pass;
                              },
                              [](auto const &) {}},
                   e);

        ctx.active_node = "CheckArmed";
        return ctx.is_armed() ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckAirborne : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    Status process(Event const &, DroneContext &ctx)
    {
        ctx.active_node = "CheckAirborne";
        return ctx.is_airborne() ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckBattery : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    float threshold;
    explicit CheckBattery(float thresh = 0.25f) : threshold(thresh) {}

    Status process(Event const &e, DroneContext &ctx)
    {
        std::visit(overloaded{[&](BatteryUpdate const &b)
                              {
                                  ctx.battery_voltage = b.voltage;
                                  ctx.battery_remaining = b.remaining;
                                  ctx.battery_current = b.current;
                              },
                              [](auto const &) {}},
                   e);

        ctx.active_node = "CheckBattery";
        return ctx.battery_remaining >= threshold ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckGeofence : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    Status process(Event const &e, DroneContext &ctx)
    {
        std::visit(overloaded{[&](LocalPositionUpdate const &p)
                              {
                                  ctx.x = p.x;
                                  ctx.y = p.y;
                                  ctx.z = p.z;
                                  ctx.vx = p.vx;
                                  ctx.vy = p.vy;
                                  ctx.vz = p.vz;
                              },
                              [](auto const &) {}},
                   e);

        ctx.active_node = "CheckGeofence";
        return ctx.in_geofence() ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckObstacleClear : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    Status process(Event const &e, DroneContext &ctx)
    {
        std::visit(overloaded{[&](ObstacleUpdate const &o)
                              {
                                  ctx.obstacle_distance = o.min_distance;
                                  ctx.obstacle_angle = o.angle;
                              },
                              [](auto const &) {}},
                   e);

        ctx.active_node = "CheckObstacle";
        return ctx.obstacle_distance > ctx.obstacle_threshold
                   ? Status::Success
                   : Status::Failure;
    }
    void reset() {}
};

struct CheckMissionComplete : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    Status process(Event const &, DroneContext &ctx)
    {
        ctx.active_node = "CheckMission";
        return ctx.mission_complete ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckPreflightOk : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    Status process(Event const &, DroneContext &ctx)
    {
        ctx.active_node = "CheckPreflight";
        return ctx.pre_flight_ok ? Status::Success : Status::Failure;
    }
    void reset() {}
};

// ============================================================================
// ACTION NODES
// ============================================================================

struct Arm : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;
    static constexpr int kTimeout = 50; // 5 seconds at 10Hz

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Arm";

        if (ctx.is_armed())
        {
            // Set home position on first arm
            if (!ctx.home_set)
            {
                ctx.home_x = ctx.x;
                ctx.home_y = ctx.y;
                ctx.home_z = ctx.z;
                ctx.home_set = true;
                ctx.log("Home set: (" + std::to_string(ctx.x) + ", " +
                        std::to_string(ctx.y) + ")");
            }
            ticks = 0;
            return Status::Success;
        }

        if (++ticks > kTimeout)
        {
            ctx.log("ARM TIMEOUT!");
            ticks = 0;
            return Status::Failure;
        }

        if (ticks == 1)
        {
            ctx.log("Arming...");
            ctx.send_command(px4_cmd::ARM, 1.0f);
        }

        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct Disarm : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Disarm";

        if (!ctx.is_armed())
        {
            ticks = 0;
            return Status::Success;
        }

        if (++ticks == 1)
        {
            ctx.log("Disarming...");
            ctx.send_command(px4_cmd::ARM, 0.0f);
        }

        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct Takeoff : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;
    static constexpr int kTimeout = 100;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Takeoff";

        float const target_alt = -ctx.takeoff_alt; // Convert to AGL
        float const current_alt = -ctx.z;

        if (current_alt >= target_alt - 0.5f)
        {
            ctx.log("Takeoff complete at " + std::to_string(current_alt) + "m");
            ticks = 0;
            return Status::Success;
        }

        if (++ticks > kTimeout)
        {
            ctx.log("TAKEOFF TIMEOUT!");
            ticks = 0;
            return Status::Failure;
        }

        // Set trajectory setpoint for climbing
        ctx.setpoint.position[0] = ctx.x;
        ctx.setpoint.position[1] = ctx.y;
        ctx.setpoint.position[2] = ctx.takeoff_alt;
        ctx.setpoint.yaw = 0.0f;

        if (ticks % 10 == 1)
        {
            ctx.log("Climbing: " + std::to_string(current_alt) + "/" +
                    std::to_string(target_alt) + "m");
        }

        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct Land : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;
    static constexpr int kTimeout = 200;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Land";

        float const current_alt = -ctx.z;

        if (current_alt < 0.3f && std::abs(ctx.vz) < 0.1f)
        {
            ctx.log("Landed!");
            ticks = 0;
            return Status::Success;
        }

        if (++ticks > kTimeout)
        {
            ctx.log("LAND TIMEOUT - forcing disarm");
            ticks = 0;
            return Status::Failure;
        }

        // Descend at current position
        ctx.setpoint.position[0] = ctx.x;
        ctx.setpoint.position[1] = ctx.y;
        ctx.setpoint.position[2] = 0.0f; // Ground level
        ctx.setpoint.yaw = 0.0f;

        if (ticks % 20 == 1)
        {
            ctx.log("Landing: " + std::to_string(current_alt) + "m AGL");
        }

        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct ReturnToLaunch : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    enum class Phase
    {
        GoHome,
        Descend,
        Done
    };
    Phase phase = Phase::GoHome;
    int ticks = 0;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "RTL";

        float const dist_to_home = ctx.horizontal_distance_to(ctx.home_x, ctx.home_y);
        float const current_alt = -ctx.z;

        switch (phase)
        {
        case Phase::GoHome:
            if (dist_to_home < 2.0f)
            {
                ctx.log("Over home, descending");
                phase = Phase::Descend;
            }
            else
            {
                // Fly to home at safe altitude
                ctx.setpoint.position[0] = ctx.home_x;
                ctx.setpoint.position[1] = ctx.home_y;
                ctx.setpoint.position[2] = std::min(ctx.z, -15.0f); // At least 15m
                ctx.setpoint.yaw = std::atan2(ctx.home_y - ctx.y, ctx.home_x - ctx.x);

                if (++ticks % 20 == 1)
                {
                    ctx.log("RTL: " + std::to_string(dist_to_home) + "m to home");
                }
            }
            break;

        case Phase::Descend:
            if (current_alt < 0.5f)
            {
                phase = Phase::Done;
                return Status::Success;
            }
            ctx.setpoint.position[0] = ctx.home_x;
            ctx.setpoint.position[1] = ctx.home_y;
            ctx.setpoint.position[2] = 0.0f;
            ctx.setpoint.yaw = 0.0f;

            if (++ticks % 20 == 1)
            {
                ctx.log("RTL descend: " + std::to_string(current_alt) + "m");
            }
            break;

        case Phase::Done:
            return Status::Success;
        }

        return Status::Running;
    }

    void reset()
    {
        phase = Phase::GoHome;
        ticks = 0;
    }
};

struct NavigateToWaypoint : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Navigate";

        if (ctx.waypoints.empty() || ctx.current_waypoint >= ctx.waypoints.size())
        {
            ctx.mission_complete = true;
            ctx.log("Mission complete!");
            return Status::Success;
        }

        auto const &wp = ctx.waypoints[ctx.current_waypoint];
        float const dist = ctx.distance_to(wp.x, wp.y, wp.z);

        if (dist < ctx.waypoint_radius)
        {
            ctx.log("Reached " + wp.name);
            ctx.current_waypoint++;
            ticks = 0;

            if (ctx.current_waypoint >= ctx.waypoints.size())
            {
                ctx.mission_complete = true;
                return Status::Success;
            }
            return Status::Running;
        }

        // Set trajectory setpoint
        ctx.setpoint.position[0] = wp.x;
        ctx.setpoint.position[1] = wp.y;
        ctx.setpoint.position[2] = wp.z;
        ctx.setpoint.yaw = wp.yaw;

        if (++ticks % 30 == 1)
        {
            ctx.log("Nav to " + wp.name + ": " + std::to_string(dist) + "m");
        }

        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct HoldPosition : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;
    float hold_x = 0, hold_y = 0, hold_z = 0;
    bool position_captured = false;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Hold";

        if (!position_captured)
        {
            hold_x = ctx.x;
            hold_y = ctx.y;
            hold_z = ctx.z;
            position_captured = true;
            ctx.log("Holding at (" + std::to_string(hold_x) + ", " +
                    std::to_string(hold_y) + ")");
        }

        ctx.setpoint.position[0] = hold_x;
        ctx.setpoint.position[1] = hold_y;
        ctx.setpoint.position[2] = hold_z;
        ctx.setpoint.yaw = 0.0f;

        // Hold for obstacle clearance (check happens elsewhere)
        if (++ticks % 50 == 0)
        {
            ctx.log("Holding... obstacle at " +
                    std::to_string(ctx.obstacle_distance) + "m");
        }

        // Stay in hold mode (Running) until obstacle clears
        return Status::Running;
    }

    void reset()
    {
        ticks = 0;
        position_captured = false;
    }
};

struct AvoidObstacle : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;
    float avoid_x = 0, avoid_y = 0;
    bool avoidance_started = false;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Avoid";

        if (!avoidance_started)
        {
            // Move perpendicular to obstacle
            float const avoid_angle = ctx.obstacle_angle + M_PI_2;
            float const avoid_dist = 5.0f;
            avoid_x = ctx.x + avoid_dist * std::cos(avoid_angle);
            avoid_y = ctx.y + avoid_dist * std::sin(avoid_angle);
            avoidance_started = true;
            ctx.log("Avoiding obstacle, moving to (" +
                    std::to_string(avoid_x) + ", " + std::to_string(avoid_y) + ")");
        }

        float const dist = ctx.horizontal_distance_to(avoid_x, avoid_y);

        if (dist < 1.0f)
        {
            ctx.log("Avoidance complete");
            return Status::Success;
        }

        ctx.setpoint.position[0] = avoid_x;
        ctx.setpoint.position[1] = avoid_y;
        ctx.setpoint.position[2] = ctx.z;
        ctx.setpoint.yaw = std::atan2(avoid_y - ctx.y, avoid_x - ctx.x);

        ++ticks;
        return Status::Running;
    }

    void reset()
    {
        ticks = 0;
        avoidance_started = false;
    }
};

struct EmergencyLand : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "EMERGENCY";

        // Immediate descent at current position
        ctx.setpoint.position[0] = ctx.x;
        ctx.setpoint.position[1] = ctx.y;
        ctx.setpoint.position[2] = 0.0f;
        ctx.setpoint.yaw = 0.0f;

        float const alt = -ctx.z;

        if (alt < 0.3f)
        {
            ctx.log("Emergency land complete");
            return Status::Success;
        }

        if (++ticks % 10 == 1)
        {
            ctx.log("!!! EMERGENCY LANDING !!! Alt: " + std::to_string(alt) + "m");
        }

        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct Idle : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    Status process(Event const &, DroneContext &ctx)
    {
        ctx.active_node = "Idle";

        // Hold current position if airborne
        if (ctx.is_airborne())
        {
            ctx.setpoint.position[0] = ctx.x;
            ctx.setpoint.position[1] = ctx.y;
            ctx.setpoint.position[2] = ctx.z;
            ctx.setpoint.yaw = 0.0f;
        }

        return Status::Success;
    }

    void reset() {}
};

struct EnableOffboard : NodeBase
{
    using EventType = Event;
    using ContextType = DroneContext;

    int ticks = 0;
    static constexpr int kSetpointsBeforeSwitch = 10;

    Status process(Event const &e, DroneContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "EnableOffboard";

        // Need to send setpoints before switching to offboard
        ctx.offboard_setpoint_counter++;

        // Initialize setpoint to current position
        ctx.setpoint.position[0] = ctx.x;
        ctx.setpoint.position[1] = ctx.y;
        ctx.setpoint.position[2] = ctx.z;
        ctx.setpoint.yaw = 0.0f;

        if (++ticks < kSetpointsBeforeSwitch)
        {
            ctx.log("Offboard prep: " + std::to_string(ticks) + "/" +
                    std::to_string(kSetpointsBeforeSwitch));
            return Status::Running;
        }

        if (!ctx.offboard_enabled)
        {
            ctx.log("Switching to OFFBOARD mode");
            // Mode: 1=MAIN, 6=OFFBOARD
            ctx.send_command(px4_cmd::SET_MODE, 1.0f, px4_cmd::OFFBOARD);
            ctx.offboard_enabled = true;
        }

        ticks = 0;
        return Status::Success;
    }

    void reset() { ticks = 0; }
};

// ============================================================================
// ROS2 NODE
// ============================================================================

class PX4DroneNode : public rclcpp::Node
{
public:
    PX4DroneNode() : Node("px4_drone_bt")
    {
        // QoS for PX4
        auto qos = rclcpp::QoS(10)
                       .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                       .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // Publishers
        offboard_pub_ = create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        setpoint_pub_ = create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        command_pub_ = create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/drone/bt_status", 10);

        // Subscribers
        battery_sub_ = create_subscription<BatteryStatus>(
            "/fmu/out/battery_status", qos,
            [this](BatteryStatus::SharedPtr msg)
            {
                events_.push_back(BatteryUpdate{
                    msg->voltage_v,
                    msg->remaining,
                    msg->current_a});
            });

        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            [this](VehicleLocalPosition::SharedPtr msg)
            {
                events_.push_back(LocalPositionUpdate{
                    msg->x, msg->y, msg->z,
                    msg->vx, msg->vy, msg->vz});
            });

        global_pos_sub_ = create_subscription<VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos,
            [this](VehicleGlobalPosition::SharedPtr msg)
            {
                events_.push_back(PositionUpdate{
                    msg->lat, msg->lon, msg->alt, msg->alt_ellipsoid});
            });

        status_sub_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](VehicleStatus::SharedPtr msg)
            {
                events_.push_back(VehicleStatusUpdate{
                    msg->arming_state,
                    msg->nav_state,
                    msg->pre_flight_checks_pass});
            });

        // Setup mission waypoints (square pattern)
        setup_mission();

        // Main loop at 10Hz
        timer_ = create_wall_timer(100ms, [this]()
                                   { tick(); });

        print_tree_structure();

        RCLCPP_INFO(get_logger(), "PX4 Drone BT Node started");
        RCLCPP_INFO(get_logger(), "Waiting for PX4 connection...");
    }

private:
    void setup_mission()
    {
        // Square patrol pattern (NED coordinates)
        ctx_.waypoints = {
            {20.0f, 0.0f, -10.0f, 0.0f, "WP1_East"},
            {20.0f, 20.0f, -10.0f, M_PI_2, "WP2_NE"},
            {0.0f, 20.0f, -10.0f, M_PI, "WP3_North"},
            {0.0f, 0.0f, -10.0f, -M_PI_2, "WP4_Home"},
        };
        ctx_.current_waypoint = 0;
        ctx_.mission_complete = false;
    }

    void tick()
    {
        // Process sensor events
        for (auto const &e : events_)
        {
            tree_.process(e, ctx_);
        }
        events_.clear();

        // Main tick
        tree_.process(TickEvent{0.1}, ctx_);

        // Publish offboard control mode
        publish_offboard_mode();

        // Publish setpoint
        publish_setpoint();

        // Publish pending commands
        if (ctx_.has_pending_command)
        {
            ctx_.pending_command.timestamp = get_clock()->now().nanoseconds() / 1000;
            command_pub_->publish(ctx_.pending_command);
            ctx_.has_pending_command = false;
        }

        // Publish status
        publish_status();

        // Terminal display
        print_status();
    }

    void publish_offboard_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_pub_->publish(msg);
    }

    void publish_setpoint()
    {
        ctx_.setpoint.timestamp = get_clock()->now().nanoseconds() / 1000;
        setpoint_pub_->publish(ctx_.setpoint);
    }

    void publish_status()
    {
        std_msgs::msg::String msg;
        msg.data = ctx_.active_node;
        status_pub_->publish(msg);
    }

    void print_tree_structure()
    {
        RCLCPP_INFO(get_logger(), R"(
╔═══════════════════════════════════════════════════════════════════════════╗
║                    PX4 DRONE BEHAVIOR TREE (DSL)                          ║
╠═══════════════════════════════════════════════════════════════════════════╣
║                                                                           ║
║  auto tree =                                                              ║
║      // Emergency: Geofence violation or critical battery                 ║
║      ((!CheckGeofence{} || !CheckBattery{0.15f}) && EmergencyLand{})      ║
║      ||                                                                   ║
║      // Low battery: Return to launch                                     ║
║      (!CheckBattery{0.25f} && ReturnToLaunch{})                           ║
║      ||                                                                   ║
║      // Mission execution                                                 ║
║      (CheckBattery{0.25f} && CheckGeofence{} &&                           ║
║          // Ensure armed and airborne                                     ║
║          (CheckArmed{} || (CheckPreflightOk{} && Arm{})) &&               ║
║          (CheckAirborne{} || (EnableOffboard{} && Takeoff{})) &&          ║
║          // Navigate or avoid                                             ║
║          ((CheckObstacleClear{} && NavigateToWaypoint{})                  ║
║              || AvoidObstacle{}))                                         ║
║      ||                                                                   ║
║      // Mission complete: Land                                            ║
║      (CheckMissionComplete{} && Land{} && Disarm{})                       ║
║      ||                                                                   ║
║      Idle{};                                                              ║
║                                                                           ║
╠═══════════════════════════════════════════════════════════════════════════╣
║  Legend:  && = Sequence    || = Selector    ! = Inverter                  ║
╚═══════════════════════════════════════════════════════════════════════════╝
        )");
    }

    void print_status()
    {
        std::cout << "\033[2J\033[H";
        std::cout << "┌────────────────────────────────────────────────────────────────┐\n";
        std::cout << "│              PX4 DRONE - BEHAVIOR TREE DEMO                    │\n";
        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        // Active node
        std::cout << "│ Active: " << std::setw(54) << std::left
                  << ctx_.active_node << "│\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        // State
        std::string arm_state = ctx_.is_armed() ? "ARMED" : "DISARMED";
        std::string air_state = ctx_.is_airborne() ? "AIRBORNE" : "GROUND";
        std::cout << "│ State: " << std::setw(12) << arm_state
                  << " | " << std::setw(10) << air_state
                  << " | Offboard: " << (ctx_.offboard_enabled ? "ON " : "OFF")
                  << "          │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        // Battery
        std::cout << "│ Battery: [";
        int const bars = static_cast<int>(ctx_.battery_remaining * 20);
        for (int i = 0; i < 20; ++i)
        {
            if (i < bars)
                std::cout << (ctx_.battery_remaining < 0.25f ? "▓" : "█");
            else
                std::cout << "░";
        }
        std::cout << "] " << std::setw(3) << static_cast<int>(ctx_.battery_remaining * 100)
                  << "% " << std::fixed << std::setprecision(1)
                  << ctx_.battery_voltage << "V        │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        // Position
        std::cout << "│ Position (NED): x=" << std::setw(7) << std::setprecision(2) << ctx_.x
                  << " y=" << std::setw(7) << ctx_.y
                  << " z=" << std::setw(7) << ctx_.z << "           │\n";

        std::cout << "│ Altitude AGL:   " << std::setw(6) << (-ctx_.z) << "m"
                  << "    Velocity: " << std::setw(5)
                  << std::sqrt(ctx_.vx * ctx_.vx + ctx_.vy * ctx_.vy + ctx_.vz * ctx_.vz)
                  << " m/s          │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        // Mission
        std::cout << "│ Mission: WP " << ctx_.current_waypoint << "/"
                  << ctx_.waypoints.size();
        if (!ctx_.waypoints.empty() && ctx_.current_waypoint < ctx_.waypoints.size())
        {
            auto const &wp = ctx_.waypoints[ctx_.current_waypoint];
            std::cout << " [" << wp.name << "] dist="
                      << std::setw(5) << ctx_.distance_to(wp.x, wp.y, wp.z) << "m";
        }
        std::cout << std::setw(20) << " " << "│\n";

        // Geofence & Obstacle
        float const horiz_dist = std::sqrt(ctx_.x * ctx_.x + ctx_.y * ctx_.y);
        std::cout << "│ Geofence: " << std::setw(5) << horiz_dist << "/"
                  << std::setw(5) << ctx_.geofence_radius << "m"
                  << "   Obstacle: " << std::setw(5) << ctx_.obstacle_distance << "m"
                  << "              │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        // Log
        std::cout << "│ Log:                                                           │\n";
        for (auto const &l : ctx_.log_buffer)
        {
            std::cout << "│  " << std::setw(61) << std::left << l.substr(0, 61) << "│\n";
        }
        for (size_t i = ctx_.log_buffer.size(); i < 8; ++i)
        {
            std::cout << "│" << std::setw(64) << " " << "│\n";
        }

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";
        std::cout << "│ Commands:                                                      │\n";
        std::cout << "│  Start SITL:  make px4_sitl gazebo-classic                     │\n";
        std::cout << "│  microDDS:    MicroXRCEAgent udp4 -p 8888                       │\n";
        std::cout << "│  Low battery: ros2 topic pub /sim/battery std_msgs/Float32 ... │\n";
        std::cout << "└────────────────────────────────────────────────────────────────┘\n";
    }

    // =========================================================================
    // BEHAVIOR TREE
    // =========================================================================

    // Emergency handling (geofence or critical battery)
    // Low battery RTL
    // Normal mission with arm/takeoff/navigate
    // Mission complete landing
    // Fallback idle

    decltype(
        // Emergency: geofence violation OR critical battery
        ((!CheckGeofence{} || !CheckBattery{0.15f}) && EmergencyLand{}) ||
        // Low battery: RTL
        (!CheckBattery{0.25f} && ReturnToLaunch{}) ||
        // Normal mission
        (CheckBattery{0.25f} && CheckGeofence{} &&
         // Ensure armed
         (CheckArmed{} || (CheckPreflightOk{} && Arm{})) &&
         // Ensure airborne
         (CheckAirborne{} || (EnableOffboard{} && Takeoff{})) &&
         // Navigate or avoid
         ((CheckObstacleClear{} && NavigateToWaypoint{}) || AvoidObstacle{})) ||
        // Mission complete
        (CheckMissionComplete{} && Land{} && Disarm{}) ||
        // Fallback
        Idle{}) tree_ =
        ((!CheckGeofence{} || !CheckBattery{0.15f}) && EmergencyLand{}) ||
        (!CheckBattery{0.25f} && ReturnToLaunch{}) ||
        (CheckBattery{0.25f} && CheckGeofence{} &&
         (CheckArmed{} || (CheckPreflightOk{} && Arm{})) &&
         (CheckAirborne{} || (EnableOffboard{} && Takeoff{})) &&
         ((CheckObstacleClear{} && NavigateToWaypoint{}) || AvoidObstacle{})) ||
        (CheckMissionComplete{} && Land{} && Disarm{}) ||
        Idle{};

    DroneContext ctx_{};
    std::vector<Event> events_{};

    // ROS2 interfaces
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_pos_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4DroneNode>());
    rclcpp::shutdown();
    return 0;
}