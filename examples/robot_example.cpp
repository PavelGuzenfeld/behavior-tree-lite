/// @file robot_example.cpp
/// @brief Example: Robot combat behavior tree

#include <behavior_tree_lite/behavior_tree.hpp>
#include <iostream>
#include <variant>

using namespace bt;

// ==========================================
// EVENT DEFINITIONS
// ==========================================

struct TickEvent
{
    double dt = 0.1;
};
struct BatteryEvent
{
    int voltage;
};
struct EnemyEvent
{
    int dist;
    int id;
};

using Event = std::variant<TickEvent, BatteryEvent, EnemyEvent>;

struct EventNameVisitor
{
    std::string_view operator()(const TickEvent &) { return "Tick"; }
    std::string_view operator()(const BatteryEvent &) { return "Battery"; }
    std::string_view operator()(const EnemyEvent &) { return "Enemy"; }
};

// ==========================================
// CONTEXT
// ==========================================

struct Context
{
    int battery = 100;
    bool alarm_active = false;
};

// ==========================================
// LEAF NODES
// ==========================================

struct CheckBattery : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &e, Context &ctx)
    {
        std::visit(overloaded{[&](const BatteryEvent &b)
                              {
                                  ctx.battery = b.voltage;
                                  if (ctx.battery < 20)
                                  {
                                      std::cout << "  [Battery] CRITICAL: " << ctx.battery << "%\n";
                                  }
                              },
                              [](const auto &) {}},
                   e);
        return (ctx.battery < 20) ? Status::Failure : Status::Success;
    }
    void reset() {}
};

struct ScanForEnemy : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &e, Context &)
    {
        return std::visit(overloaded{[](const EnemyEvent &en)
                                     {
                                         std::cout << "  [Scan] Enemy detected at " << en.dist << "m!\n";
                                         return Status::Success;
                                     },
                                     [](const TickEvent &)
                                     { return Status::Running; },
                                     [](const auto &)
                                     { return Status::Running; }},
                          e);
    }
    void reset() {}
};

struct FireWeapon : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    int shots = 0;

    Status process(const Event &e, Context &)
    {
        if (std::holds_alternative<TickEvent>(e))
        {
            shots++;
            std::cout << "  [Fire] Bang! (" << shots << "/3)\n";
            if (shots >= 3)
                return Status::Success;
        }
        return Status::Running;
    }
    void reset() { shots = 0; }
};

struct MoveToCover : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &e, Context &)
    {
        if (std::holds_alternative<TickEvent>(e))
        {
            std::cout << "  [Move] Dash to cover!\n";
            return Status::Success;
        }
        return Status::Running;
    }
    void reset() {}
};

struct EmergencySiren : NodeBase
{
    Status process(const Event &, Context &)
    {
        std::cout << "  [Fallback] *** EMERGENCY SIREN ***\n";
        return Status::Success;
    }
    void reset() {}
};

// ==========================================
// MAIN
// ==========================================

int main()
{
    std::cout << "=== Behavior Tree Lite - Robot Example ===\n\n";

    /*
       Tree Structure:
       ---------------
       Selector (Root):
         1. Sequence (Combat Routine):
             - CheckBattery (Must be > 20%)
             - Retry(2) -> ScanForEnemy
             - Parallel (Attack Maneuver):
                  - MoveToCover
                  - FireWeapon (3 ticks)
         2. EmergencySiren (Fallback)
    */

    Selector<Event, Context,
             Sequence<Event, Context, CheckBattery,
                      Retry<Event, Context, ScanForEnemy>,
                      Parallel<Event, Context, MoveToCover, FireWeapon>>,
             EmergencySiren>
        root(
            Sequence<Event, Context,
                     CheckBattery,
                     Retry<Event, Context, ScanForEnemy>,
                     Parallel<Event, Context, MoveToCover, FireWeapon>>(
                CheckBattery{},
                Retry<Event, Context, ScanForEnemy>(2, ScanForEnemy{}),
                Parallel<Event, Context, MoveToCover, FireWeapon>(
                    MoveToCover{},
                    FireWeapon{})),
            EmergencySiren{});

    Context ctx;

    auto dispatch = [&](Event e)
    {
        std::string_view name = std::visit(EventNameVisitor{}, e);
        std::cout << "\n--- Event: " << name << " ---\n";
        Status s = root.process(e, ctx);
        std::cout << "Tree Status: " << to_string(s) << "\n";
    };

    // Simulation
    dispatch(TickEvent{});       // Battery OK, scanning...
    dispatch(TickEvent{});       // Still scanning...
    dispatch(EnemyEvent{10, 1}); // Enemy! Parallel starts
    dispatch(TickEvent{});       // Fire again
    dispatch(BatteryEvent{10});  // Battery critical! Fallback
    dispatch(BatteryEvent{100}); // Battery restored
    dispatch(BatteryEvent{5});   // Battery critical again
    dispatch(TickEvent{});       // Fallback activates
    dispatch(TickEvent{});

    std::cout << "\n=== Simulation Complete ===\n";
    return 0;
}
