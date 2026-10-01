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

struct TickEvent
{
    double dt = 0.1;
};

struct BatteryUpdate
{
    float voltage;
    float remaining;
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
    float angle;
};

using Event =
    std::variant<TickEvent, BatteryUpdate, PositionUpdate, LocalPositionUpdate, VehicleStatusUpdate, ObstacleUpdate>;

struct Waypoint
{
    float x, y, z;
    float yaw;
    std::string name;
};

/// Positions, setpoints and waypoints are PX4 local NED in metres (z negative is up).
/// geofence_*_alt are metres AGL; battery values are fractions of full charge (0..1).
struct VehicleContext
{
    uint8_t arming_state = 0;
    uint8_t nav_state = 0;
    bool pre_flight_ok = false;

    float x = 0.0f, y = 0.0f, z = 0.0f;
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;

    double lat = 0.0, lon = 0.0;
    float alt = 0.0f, rel_alt = 0.0f;

    float battery_voltage = 16.8f;
    float battery_remaining = 1.0f;
    float battery_current = 0.0f;

    float obstacle_distance = 100.0f;
    float obstacle_angle = 0.0f;

    std::vector<Waypoint> waypoints;
    size_t current_waypoint = 0;
    bool mission_complete = false;

    float geofence_radius = 100.0f;
    float geofence_max_alt = 50.0f;
    float geofence_min_alt = 2.0f;

    float battery_critical = 0.15f;
    float battery_low = 0.25f;
    float waypoint_radius = 1.5f;
    float obstacle_threshold = 3.0f;
    float takeoff_alt = -10.0f;

    float home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;
    bool home_set = false;

    TrajectorySetpoint setpoint{};
    VehicleCommand pending_command{};
    bool has_pending_command = false;

    std::string active_node;
    std::deque<std::string> log_buffer;

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

    void send_command(uint32_t cmd, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f, float p4 = 0.0f, float p5 = 0.0f,
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

    [[nodiscard]] bool is_armed() const { return arming_state == VehicleStatus::ARMING_STATE_ARMED; }

    [[nodiscard]] bool is_airborne() const { return -z > 1.0f && is_armed(); }

    [[nodiscard]] bool in_geofence() const
    {
        float const horiz_dist = std::sqrt(x * x + y * y);
        float const agl = -z;
        return horiz_dist < geofence_radius && agl < geofence_max_alt && agl > 0.0f;
    }
};

namespace px4_cmd
{
    constexpr uint32_t ARM = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    constexpr uint32_t TAKEOFF = VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    constexpr uint32_t LAND = VehicleCommand::VEHICLE_CMD_NAV_LAND;
    constexpr uint32_t RTL = VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
    constexpr uint32_t SET_MODE = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    constexpr float CUSTOM_MODE_ENABLED = 1.0f;
    constexpr uint32_t OFFBOARD_MAIN_MODE = 6;
} // namespace px4_cmd

struct CheckArmed : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    Status process(Event const &e, VehicleContext &ctx)
    {
        std::visit(overloaded{[&](VehicleStatusUpdate const &s)
                              {
                                  ctx.arming_state = s.arming_state;
                                  ctx.nav_state = s.nav_state;
                                  ctx.pre_flight_ok = s.pre_flight_checks_pass;
                              },
                              [](auto const &) {
                              }},
                   e);

        ctx.active_node = "CheckArmed";
        return ctx.is_armed() ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckAirborne : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    Status process(Event const &, VehicleContext &ctx)
    {
        ctx.active_node = "CheckAirborne";
        return ctx.is_airborne() ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckBattery : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    float threshold;
    explicit CheckBattery(float thresh = 0.25f) : threshold(thresh) {}

    Status process(Event const &e, VehicleContext &ctx)
    {
        std::visit(overloaded{[&](BatteryUpdate const &b)
                              {
                                  ctx.battery_voltage = b.voltage;
                                  ctx.battery_remaining = b.remaining;
                                  ctx.battery_current = b.current;
                              },
                              [](auto const &) {
                              }},
                   e);

        ctx.active_node = "CheckBattery";
        return ctx.battery_remaining >= threshold ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckGeofence : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    Status process(Event const &e, VehicleContext &ctx)
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
                              [](auto const &) {
                              }},
                   e);

        ctx.active_node = "CheckGeofence";
        return ctx.in_geofence() ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckObstacleClear : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    Status process(Event const &e, VehicleContext &ctx)
    {
        std::visit(overloaded{[&](ObstacleUpdate const &o)
                              {
                                  ctx.obstacle_distance = o.min_distance;
                                  ctx.obstacle_angle = o.angle;
                              },
                              [](auto const &) {
                              }},
                   e);

        ctx.active_node = "CheckObstacle";
        return ctx.obstacle_distance > ctx.obstacle_threshold ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckMissionComplete : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    Status process(Event const &, VehicleContext &ctx)
    {
        ctx.active_node = "CheckMission";
        return ctx.mission_complete ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct CheckPreflightOk : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    Status process(Event const &, VehicleContext &ctx)
    {
        ctx.active_node = "CheckPreflight";
        return ctx.pre_flight_ok ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct Arm : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    int ticks = 0;
    static constexpr int kTimeoutTicksAt10Hz = 50;

    Status process(Event const &e, VehicleContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Arm";

        if (ctx.is_armed())
        {
            if (!ctx.home_set)
            {
                ctx.home_x = ctx.x;
                ctx.home_y = ctx.y;
                ctx.home_z = ctx.z;
                ctx.home_set = true;
                ctx.log("Home set: (" + std::to_string(ctx.x) + ", " + std::to_string(ctx.y) + ")");
            }
            ticks = 0;
            return Status::Success;
        }

        if (++ticks > kTimeoutTicksAt10Hz)
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
    using ContextType = VehicleContext;

    int ticks = 0;

    Status process(Event const &e, VehicleContext &ctx)
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
    using ContextType = VehicleContext;

    int ticks = 0;
    static constexpr int kTimeoutTicksAt10Hz = 100;

    Status process(Event const &e, VehicleContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Takeoff";

        float const target_alt = -ctx.takeoff_alt;
        float const current_alt = -ctx.z;

        if (current_alt >= target_alt - 0.5f)
        {
            ctx.log("Takeoff complete at " + std::to_string(current_alt) + "m");
            ticks = 0;
            return Status::Success;
        }

        if (++ticks > kTimeoutTicksAt10Hz)
        {
            ctx.log("TAKEOFF TIMEOUT!");
            ticks = 0;
            return Status::Failure;
        }

        ctx.setpoint.position[0] = ctx.x;
        ctx.setpoint.position[1] = ctx.y;
        ctx.setpoint.position[2] = ctx.takeoff_alt;
        ctx.setpoint.yaw = 0.0f;

        if (ticks % 10 == 1)
        {
            ctx.log("Climbing: " + std::to_string(current_alt) + "/" + std::to_string(target_alt) + "m");
        }

        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct Land : NodeBase
{
    using EventType = Event;
    using ContextType = VehicleContext;

    int ticks = 0;
    static constexpr int kTimeoutTicksAt10Hz = 200;

    Status process(Event const &e, VehicleContext &ctx)
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

        if (++ticks > kTimeoutTicksAt10Hz)
        {
            ctx.log("LAND TIMEOUT - forcing disarm");
            ticks = 0;
            return Status::Failure;
        }

        ctx.setpoint.position[0] = ctx.x;
        ctx.setpoint.position[1] = ctx.y;
        ctx.setpoint.position[2] = 0.0f;
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
    using ContextType = VehicleContext;

    enum class Phase
    {
        GoHome,
        Descend,
        Done
    };
    Phase phase = Phase::GoHome;
    int ticks = 0;
    static constexpr float kMinReturnAltitudeNed = -15.0f;

    Status process(Event const &e, VehicleContext &ctx)
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
                ctx.setpoint.position[0] = ctx.home_x;
                ctx.setpoint.position[1] = ctx.home_y;
                ctx.setpoint.position[2] = std::min(ctx.z, kMinReturnAltitudeNed);
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
    using ContextType = VehicleContext;

    int ticks = 0;

    Status process(Event const &e, VehicleContext &ctx)
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
    using ContextType = VehicleContext;

    int ticks = 0;
    float hold_x = 0, hold_y = 0, hold_z = 0;
    bool position_captured = false;

    Status process(Event const &e, VehicleContext &ctx)
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
            ctx.log("Holding at (" + std::to_string(hold_x) + ", " + std::to_string(hold_y) + ")");
        }

        ctx.setpoint.position[0] = hold_x;
        ctx.setpoint.position[1] = hold_y;
        ctx.setpoint.position[2] = hold_z;
        ctx.setpoint.yaw = 0.0f;

        if (++ticks % 50 == 0)
        {
            ctx.log("Holding... obstacle at " + std::to_string(ctx.obstacle_distance) + "m");
        }

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
    using ContextType = VehicleContext;

    int ticks = 0;
    float avoid_x = 0, avoid_y = 0;
    bool avoidance_started = false;

    Status process(Event const &e, VehicleContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "Avoid";

        if (!avoidance_started)
        {
            float const avoid_angle = ctx.obstacle_angle + M_PI_2;
            float const avoid_dist = 5.0f;
            avoid_x = ctx.x + avoid_dist * std::cos(avoid_angle);
            avoid_y = ctx.y + avoid_dist * std::sin(avoid_angle);
            avoidance_started = true;
            ctx.log("Avoiding obstacle, moving to (" + std::to_string(avoid_x) + ", " + std::to_string(avoid_y) + ")");
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
    using ContextType = VehicleContext;

    int ticks = 0;

    Status process(Event const &e, VehicleContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "EMERGENCY";

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
    using ContextType = VehicleContext;

    Status process(Event const &, VehicleContext &ctx)
    {
        ctx.active_node = "Idle";

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
    using ContextType = VehicleContext;

    int ticks = 0;
    static constexpr int kSetpointsBeforeSwitch = 10;

    Status process(Event const &e, VehicleContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
            return Status::Running;

        ctx.active_node = "EnableOffboard";

        ctx.offboard_setpoint_counter++;

        ctx.setpoint.position[0] = ctx.x;
        ctx.setpoint.position[1] = ctx.y;
        ctx.setpoint.position[2] = ctx.z;
        ctx.setpoint.yaw = 0.0f;

        if (++ticks < kSetpointsBeforeSwitch)
        {
            ctx.log("Offboard prep: " + std::to_string(ticks) + "/" + std::to_string(kSetpointsBeforeSwitch));
            return Status::Running;
        }

        if (!ctx.offboard_enabled)
        {
            ctx.log("Switching to OFFBOARD mode");
            ctx.send_command(px4_cmd::SET_MODE, px4_cmd::CUSTOM_MODE_ENABLED, px4_cmd::OFFBOARD_MAIN_MODE);
            ctx.offboard_enabled = true;
        }

        ticks = 0;
        return Status::Success;
    }

    void reset() { ticks = 0; }
};

class PX4VehicleNode : public rclcpp::Node
{
  public:
    PX4VehicleNode() : Node("px4_vehicle_bt")
    {
        auto qos = rclcpp::QoS(10)
                       .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                       .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/vehicle/bt_status", 10);

        battery_sub_ = create_subscription<BatteryStatus>(
            "/fmu/out/battery_status", qos, [this](BatteryStatus::SharedPtr msg)
            { events_.push_back(BatteryUpdate{msg->voltage_v, msg->remaining, msg->current_a}); });

        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, [this](VehicleLocalPosition::SharedPtr msg)
            { events_.push_back(LocalPositionUpdate{msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz}); });

        global_pos_sub_ = create_subscription<VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos, [this](VehicleGlobalPosition::SharedPtr msg)
            { events_.push_back(PositionUpdate{msg->lat, msg->lon, msg->alt, msg->alt_ellipsoid}); });

        status_sub_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](VehicleStatus::SharedPtr msg) {
                events_.push_back(VehicleStatusUpdate{msg->arming_state, msg->nav_state, msg->pre_flight_checks_pass});
            });

        setup_mission();

        timer_ = create_wall_timer(100ms, [this]() { tick(); });

        print_tree_structure();

        RCLCPP_INFO(get_logger(), "PX4 Vehicle BT Node started");
        RCLCPP_INFO(get_logger(), "Waiting for PX4 connection...");
    }

  private:
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

    void tick()
    {
        for (auto const &e : events_)
        {
            tree_.process(e, ctx_);
        }
        events_.clear();

        tree_.process(TickEvent{0.1}, ctx_);

        publish_offboard_mode();

        publish_setpoint();

        if (ctx_.has_pending_command)
        {
            ctx_.pending_command.timestamp = get_clock()->now().nanoseconds() / 1000;
            command_pub_->publish(ctx_.pending_command);
            ctx_.has_pending_command = false;
        }

        publish_status();

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
║                    PX4 VEHICLE BEHAVIOR TREE (DSL)                          ║
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
        std::cout << "│              PX4 VEHICLE - BEHAVIOR TREE DEMO                    │\n";
        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        std::cout << "│ Active: " << std::setw(54) << std::left << ctx_.active_node << "│\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        std::string arm_state = ctx_.is_armed() ? "ARMED" : "DISARMED";
        std::string air_state = ctx_.is_airborne() ? "AIRBORNE" : "GROUND";
        std::cout << "│ State: " << std::setw(12) << arm_state << " | " << std::setw(10) << air_state
                  << " | Offboard: " << (ctx_.offboard_enabled ? "ON " : "OFF") << "          │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        std::cout << "│ Battery: [";
        int const bars = static_cast<int>(ctx_.battery_remaining * 20);
        for (int i = 0; i < 20; ++i)
        {
            if (i < bars)
                std::cout << (ctx_.battery_remaining < 0.25f ? "▓" : "█");
            else
                std::cout << "░";
        }
        std::cout << "] " << std::setw(3) << static_cast<int>(ctx_.battery_remaining * 100) << "% " << std::fixed
                  << std::setprecision(1) << ctx_.battery_voltage << "V        │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        std::cout << "│ Position (NED): x=" << std::setw(7) << std::setprecision(2) << ctx_.x << " y=" << std::setw(7)
                  << ctx_.y << " z=" << std::setw(7) << ctx_.z << "           │\n";

        std::cout << "│ Altitude AGL:   " << std::setw(6) << (-ctx_.z) << "m" << "    Velocity: " << std::setw(5)
                  << std::sqrt(ctx_.vx * ctx_.vx + ctx_.vy * ctx_.vy + ctx_.vz * ctx_.vz) << " m/s          │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

        std::cout << "│ Mission: WP " << ctx_.current_waypoint << "/" << ctx_.waypoints.size();
        if (!ctx_.waypoints.empty() && ctx_.current_waypoint < ctx_.waypoints.size())
        {
            auto const &wp = ctx_.waypoints[ctx_.current_waypoint];
            std::cout << " [" << wp.name << "] dist=" << std::setw(5) << ctx_.distance_to(wp.x, wp.y, wp.z) << "m";
        }
        std::cout << std::setw(20) << " " << "│\n";

        float const horiz_dist = std::sqrt(ctx_.x * ctx_.x + ctx_.y * ctx_.y);
        std::cout << "│ Geofence: " << std::setw(5) << horiz_dist << "/" << std::setw(5) << ctx_.geofence_radius << "m"
                  << "   Obstacle: " << std::setw(5) << ctx_.obstacle_distance << "m" << "              │\n";

        std::cout << "├────────────────────────────────────────────────────────────────┤\n";

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

    decltype(((!CheckGeofence{} || !CheckBattery{0.15f}) && EmergencyLand{}) ||
             (!CheckBattery{0.25f} && ReturnToLaunch{}) ||
             (CheckBattery{0.25f} && CheckGeofence{} && (CheckArmed{} || (CheckPreflightOk{} && Arm{})) &&
              (CheckAirborne{} || (EnableOffboard{} && Takeoff{})) &&
              ((CheckObstacleClear{} && NavigateToWaypoint{}) || AvoidObstacle{})) ||
             (CheckMissionComplete{} && Land{} && Disarm{}) || Idle{}) tree_ =
        ((!CheckGeofence{} || !CheckBattery{0.15f}) && EmergencyLand{}) || (!CheckBattery{0.25f} && ReturnToLaunch{}) ||
        (CheckBattery{0.25f} && CheckGeofence{} && (CheckArmed{} || (CheckPreflightOk{} && Arm{})) &&
         (CheckAirborne{} || (EnableOffboard{} && Takeoff{})) &&
         ((CheckObstacleClear{} && NavigateToWaypoint{}) || AvoidObstacle{})) ||
        (CheckMissionComplete{} && Land{} && Disarm{}) || Idle{};

    VehicleContext ctx_{};
    std::vector<Event> events_{};

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4VehicleNode>());
    rclcpp::shutdown();
    return 0;
}