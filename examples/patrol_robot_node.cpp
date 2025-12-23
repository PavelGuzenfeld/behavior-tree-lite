/// @file patrol_robot_node.cpp
/// @brief Complex ROS2 behavior tree example with DSL and visualization
///
/// this file originally published markers in frame "map" while publishing exactly zero tf.
/// rviz can't draw imaginary coordinate frames. shocking, i know.

#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/debug.hpp"
#include "behavior_tree_lite/dsl.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// tf so rviz can stop complaining like it's paid per warning
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <string>
#include <variant>
#include <vector>

using namespace bt;
using namespace std::chrono_literals;

// ============================================================================
// EVENTS
// ============================================================================

struct TickEvent
{
    double dt = 0.1;
};
struct BatteryUpdate
{
    float level;
};
struct LaserUpdate
{
    float min_distance;
};
struct EmergencyStop
{
};

using Event = std::variant<TickEvent, BatteryUpdate, LaserUpdate, EmergencyStop>;

// ============================================================================
// CONTEXT (Blackboard)
// ============================================================================

struct RobotContext
{
    float battery_level = 100.0f;
    float obstacle_distance = 10.0f;
    bool emergency = false;
    int current_waypoint = 0;
    int total_waypoints = 4;

    geometry_msgs::msg::Twist cmd_vel{};
    std::string active_node;
    std::deque<std::string> log_buffer;

    void log(std::string const &msg)
    {
        log_buffer.push_back(msg);
        if (log_buffer.size() > 10U)
        {
            log_buffer.pop_front();
        }
    }
};

// ============================================================================
// LEAF NODES (DSL-compatible)
// ============================================================================

struct CheckBattery : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    float threshold = 20.0f;

    Status process(Event const &e, RobotContext &ctx)
    {
        std::visit(overloaded{[&](BatteryUpdate const &b)
                              { ctx.battery_level = b.level; },
                              [](auto const &) {}},
                   e);

        ctx.active_node = "CheckBattery";
        return (ctx.battery_level >= threshold) ? Status::Success : Status::Failure;
    }

    void reset() {}
};

struct CheckObstacle : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    float safe_dist = 0.5f;

    Status process(Event const &e, RobotContext &ctx)
    {
        std::visit(overloaded{[&](LaserUpdate const &l)
                              { ctx.obstacle_distance = l.min_distance; },
                              [](auto const &) {}},
                   e);

        ctx.active_node = "CheckObstacle";
        return (ctx.obstacle_distance > safe_dist) ? Status::Success : Status::Failure;
    }

    void reset() {}
};

struct CheckEmergency : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    Status process(Event const &e, RobotContext &ctx)
    {
        std::visit(overloaded{[&](EmergencyStop const &)
                              { ctx.emergency = true; },
                              [](auto const &) {}},
                   e);

        ctx.active_node = "CheckEmergency";
        return ctx.emergency ? Status::Failure : Status::Success;
    }

    void reset() {}
};

struct Navigate : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    int ticks = 0;

    Status process(Event const &e, RobotContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
        {
            return Status::Running;
        }

        ctx.active_node = "Navigate";
        ctx.cmd_vel.linear.x = 0.5;
        ctx.cmd_vel.angular.z = 0.0;

        ctx.log("Nav to WP" + std::to_string(ctx.current_waypoint) + " [" +
                std::to_string(++ticks) + "/10]");

        if (ticks >= 10)
        {
            ctx.current_waypoint = (ctx.current_waypoint + 1) % ctx.total_waypoints;
            ticks = 0;
            return Status::Success;
        }
        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct Avoid : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    int ticks = 0;

    Status process(Event const &e, RobotContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
        {
            return Status::Running;
        }

        ctx.active_node = "Avoid";
        ctx.cmd_vel.linear.x = 0.0;
        ctx.cmd_vel.angular.z = 0.8;

        ctx.log("Avoiding [" + std::to_string(++ticks) + "/5]");

        if (ticks >= 5)
        {
            ticks = 0;
            return Status::Success;
        }
        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct GoToCharger : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    int ticks = 0;

    Status process(Event const &e, RobotContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
        {
            return Status::Running;
        }

        ctx.active_node = "GoToCharger";
        ctx.cmd_vel.linear.x = 0.3;
        ctx.cmd_vel.angular.z = 0.0;

        ctx.log("To charger [" + std::to_string(++ticks) + "/15]");

        if (ticks >= 15)
        {
            ticks = 0;
            return Status::Success;
        }
        return Status::Running;
    }

    void reset() { ticks = 0; }
};

struct Charge : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    Status process(Event const &e, RobotContext &ctx)
    {
        if (!std::holds_alternative<TickEvent>(e))
        {
            return Status::Running;
        }

        ctx.active_node = "Charge";
        ctx.cmd_vel.linear.x = 0.0;
        ctx.cmd_vel.angular.z = 0.0;

        ctx.battery_level = std::min(100.0f, ctx.battery_level + 5.0f);
        ctx.log("Charging " + std::to_string(static_cast<int>(ctx.battery_level)) + "%");

        return (ctx.battery_level >= 95.0f) ? Status::Success : Status::Running;
    }

    void reset() {}
};

struct Halt : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    Status process(Event const &, RobotContext &ctx)
    {
        ctx.active_node = "HALT";
        ctx.cmd_vel.linear.x = 0.0;
        ctx.cmd_vel.angular.z = 0.0;
        ctx.log("!!! EMERGENCY HALT !!!");
        return Status::Success;
    }

    void reset() {}
};

struct Idle : NodeBase
{
    using EventType = Event;
    using ContextType = RobotContext;

    Status process(Event const &, RobotContext &ctx)
    {
        ctx.active_node = "Idle";
        ctx.cmd_vel.linear.x = 0.0;
        ctx.cmd_vel.angular.z = 0.0;
        return Status::Success;
    }

    void reset() {}
};

// ============================================================================
// ROS2 NODE
// ============================================================================

class PatrolRobotNode : public rclcpp::Node
{
public:
    PatrolRobotNode() : Node("patrol_robot")
    {
        // publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/robot_status", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/bt_markers", 10);

        // tf broadcaster so frames actually exist
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // subscribers
        battery_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/battery", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            { events_.push_back(BatteryUpdate{msg->data}); });

        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                if (msg->ranges.empty())
                {
                    return;
                }
                float const min_d = *std::min_element(msg->ranges.begin(), msg->ranges.end());
                events_.push_back(LaserUpdate{min_d});
            });

        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/estop", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                if (msg->data)
                {
                    events_.push_back(EmergencyStop{});
                }
            });

        timer_ = create_wall_timer(100ms, [this]()
                                   { tick_(); });

        print_tree_structure_();
    }

private:
    void tick_()
    {
        // process sensor events
        for (auto const &e : events_)
        {
            tree_.process(e, ctx_);
        }
        events_.clear();

        // tick tree
        tree_.process(TickEvent{}, ctx_);

        // battery drain
        ctx_.battery_level = std::max(0.0f, ctx_.battery_level - 0.1f);

        // fake motion integration so rviz has something to move
        integrate_fake_motion_(0.1);

        // publish cmd_vel + status
        cmd_vel_pub_->publish(ctx_.cmd_vel);

        std_msgs::msg::String status;
        status.data = ctx_.active_node;
        status_pub_->publish(status);

        // publish tf + markers
        publish_tf_();
        publish_markers_();

        // terminal spam UI
        print_status_();
    }

    void integrate_fake_motion_(double const dt) noexcept
    {
        // this is not physics. it's "enough for a demo so you can sleep."
        yaw_ += static_cast<double>(ctx_.cmd_vel.angular.z) * dt;

        double const v = static_cast<double>(ctx_.cmd_vel.linear.x);
        x_ += v * std::cos(yaw_) * dt;
        y_ += v * std::sin(yaw_) * dt;
    }

    void publish_tf_()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        // yaw-only quaternion
        double const half = yaw_ * 0.5;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = std::sin(half);
        t.transform.rotation.w = std::cos(half);

        tf_broadcaster_->sendTransform(t);
    }

    void print_tree_structure_()
    {
        RCLCPP_INFO(get_logger(), R"(
╔═══════════════════════════════════════════════════════════════╗
║                    BEHAVIOR TREE (DSL)                        ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  auto tree =                                                  ║
║      (!CheckEmergency{} && Halt{})                            ║
║      ||                                                       ║
║      (!CheckBattery{} && GoToCharger{} && Charge{})           ║
║      ||                                                       ║
║      (CheckBattery{} && ((CheckObstacle{} && Navigate{})      ║
║                          || Avoid{}))                         ║
║      ||                                                       ║
║      Idle{};                                                  ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  Legend:  && = Sequence    || = Selector    ! = Inverter      ║
╚═══════════════════════════════════════════════════════════════╝
        )");
    }

    void print_status_()
    {
        std::cout << "\033[2J\033[H";
        std::cout << "┌────────────────────────────────────────────────────┐\n";
        std::cout << "│          PATROL ROBOT - BT DSL DEMO                │\n";
        std::cout << "├────────────────────────────────────────────────────┤\n";
        std::cout << "│ Active: " << std::setw(42) << std::left << ctx_.active_node << "│\n";
        std::cout << "├────────────────────────────────────────────────────┤\n";

        // battery bar
        std::cout << "│ Battery: [";
        int const bars = static_cast<int>(ctx_.battery_level / 5.0f);
        for (int i = 0; i < 20; ++i)
        {
            std::cout << ((i < bars) ? "█" : "░");
        }
        std::cout << "] " << std::setw(3) << static_cast<int>(ctx_.battery_level) << "%     │\n";

        std::cout << "│ Obstacle: " << std::fixed << std::setprecision(2) << std::setw(5)
                  << ctx_.obstacle_distance << "m" << std::setw(33) << " " << "│\n";
        std::cout << "│ Waypoint: " << ctx_.current_waypoint << "/" << ctx_.total_waypoints
                  << std::setw(37) << " " << "│\n";
        std::cout << "│ Cmd: lin=" << std::setw(4) << ctx_.cmd_vel.linear.x << " ang=" << std::setw(4)
                  << ctx_.cmd_vel.angular.z << std::setw(25) << " " << "│\n";
        std::cout << "│ Pose: x=" << std::setw(6) << std::setprecision(2) << std::fixed << x_
                  << " y=" << std::setw(6) << y_ << " yaw=" << std::setw(6) << yaw_
                  << std::setw(6) << " " << "│\n";

        std::cout << "├────────────────────────────────────────────────────┤\n";
        std::cout << "│ Log:                                               │\n";
        for (auto const &l : ctx_.log_buffer)
        {
            std::cout << "│  " << std::setw(48) << std::left << l.substr(0, 48) << "│\n";
        }
        for (size_t i = ctx_.log_buffer.size(); i < 5U; ++i)
        {
            std::cout << "│" << std::setw(52) << " " << "│\n";
        }
        std::cout << "└────────────────────────────────────────────────────┘\n";
        std::cout << "\nTest commands:\n";
        std::cout << "  ros2 topic pub /battery std_msgs/Float32 \"{data: 15.0}\" --once\n";
        std::cout << "  ros2 topic pub /scan sensor_msgs/LaserScan \"{ranges: [0.3]}\" --once\n";
        std::cout << "  ros2 topic pub /estop std_msgs/Bool \"{data: true}\" --once\n";
    }

    void publish_markers_()
    {
        visualization_msgs::msg::MarkerArray markers;

        auto make_marker = [&](int const id, float const x, float const y, std::string const &text,
                               bool const active)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map"; // now this exists because we publish tf. you're welcome.
            m.header.stamp = now();
            m.ns = "bt";
            m.id = id;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.pose.position.x = x;
            m.pose.position.y = y;
            m.pose.position.z = 0.5;

            m.scale.z = 0.3;

            m.color.a = 1.0;
            m.color.r = active ? 0.0f : 0.7f;
            m.color.g = active ? 1.0f : 0.7f;
            m.color.b = active ? 0.0f : 0.7f;

            m.text = text;
            return m;
        };

        int id = 0;
        markers.markers.push_back(make_marker(id++, 0.0f, 3.0f, "ROOT (||)", false));
        markers.markers.push_back(make_marker(id++, -3.0f, 2.0f, "Emergency", ctx_.active_node == "HALT"));
        markers.markers.push_back(make_marker(
            id++, -1.0f, 2.0f, "LowBatt",
            (ctx_.active_node == "Charge") || (ctx_.active_node == "GoToCharger")));
        markers.markers.push_back(make_marker(
            id++, 1.0f, 2.0f, "Patrol",
            (ctx_.active_node == "Navigate") || (ctx_.active_node == "Avoid")));
        markers.markers.push_back(make_marker(id++, 3.0f, 2.0f, "Idle", ctx_.active_node == "Idle"));
        markers.markers.push_back(make_marker(id++, 0.0f, 0.0f, "[" + ctx_.active_node + "]", true));

        marker_pub_->publish(markers);
    }

    // =========================================================================
    // BEHAVIOR TREE - DSL DEFINITION
    // =========================================================================
    decltype((!CheckEmergency{} && Halt{}) ||
             (!CheckBattery{} && GoToCharger{} && Charge{}) ||
             (CheckBattery{} && ((CheckObstacle{} && Navigate{}) || Avoid{})) ||
             Idle{}) tree_ =
        (!CheckEmergency{} && Halt{}) ||
        (!CheckBattery{} && GoToCharger{} && Charge{}) ||
        (CheckBattery{} && ((CheckObstacle{} && Navigate{}) || Avoid{})) ||
        Idle{};

    RobotContext ctx_{};
    std::vector<Event> events_{};

    // fake pose (because you don't have nav2, slam, or any odom)
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    // ros2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrolRobotNode>());
    rclcpp::shutdown();
    return 0;
}
