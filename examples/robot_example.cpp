
#include <behavior_tree_lite/behavior_tree.hpp>
#include <iostream>
#include <variant>

using namespace bt;

struct TickEvent
{
    double dt = 0.1;
};
struct BatteryEvent
{
    int voltage;
};
struct ObjectEvent
{
    int dist;
    int id;
};

using Event = std::variant<TickEvent, BatteryEvent, ObjectEvent>;

struct EventNameVisitor
{
    std::string_view operator()(const TickEvent &) { return "Tick"; }
    std::string_view operator()(const BatteryEvent &) { return "Battery"; }
    std::string_view operator()(const ObjectEvent &) { return "Object"; }
};

struct Context
{
    int battery = 100;
    bool alarm_active = false;
};

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
                              [](const auto &) {
                              }},
                   e);
        return (ctx.battery < 20) ? Status::Failure : Status::Success;
    }
    void reset() {}
};

struct ScanForObject : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &e, Context &)
    {
        return std::visit(overloaded{[](const ObjectEvent &en)
                                     {
                                         std::cout << "  [Scan] Object detected at " << en.dist << "m!\n";
                                         return Status::Success;
                                     },
                                     [](const TickEvent &) { return Status::Running; },
                                     [](const auto &)
                                     {
                                         return Status::Running;
                                     }},
                          e);
    }
    void reset() {}
};

struct TakeSample : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    int samples = 0;

    Status process(const Event &e, Context &)
    {
        if (std::holds_alternative<TickEvent>(e))
        {
            samples++;
            std::cout << "  [Sample] Taken (" << samples << "/3)\n";
            if (samples >= 3)
                return Status::Success;
        }
        return Status::Running;
    }
    void reset() { samples = 0; }
};

struct MoveToStation : NodeBase
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

// NOLINTNEXTLINE(bugprone-exception-escape)
int main()
{
    std::cout << "=== Behavior Tree Lite - Robot Example ===\n\n";

    Selector<Event, Context,
             Sequence<Event, Context, CheckBattery, Retry<Event, Context, ScanForObject>,
                      Parallel<Event, Context, MoveToStation, TakeSample>>,
             EmergencySiren>
        root(Sequence<Event, Context, CheckBattery, Retry<Event, Context, ScanForObject>,
                      Parallel<Event, Context, MoveToStation, TakeSample>>(
                 CheckBattery{}, Retry<Event, Context, ScanForObject>(2, ScanForObject{}),
                 Parallel<Event, Context, MoveToStation, TakeSample>(MoveToStation{}, TakeSample{})),
             EmergencySiren{});

    Context ctx;

    auto dispatch = [&](Event e)
    {
        std::string_view name = std::visit(EventNameVisitor{}, e);
        std::cout << "\n--- Event: " << name << " ---\n";
        Status s = root.process(e, ctx);
        std::cout << "Tree Status: " << to_string(s) << "\n";
    };

    dispatch(TickEvent{});
    dispatch(TickEvent{});
    dispatch(ObjectEvent{10, 1});
    dispatch(TickEvent{});
    dispatch(BatteryEvent{10});
    dispatch(BatteryEvent{100});
    dispatch(BatteryEvent{5});
    dispatch(TickEvent{});
    dispatch(TickEvent{});

    std::cout << "\n=== Simulation Complete ===\n";
    return 0;
}
