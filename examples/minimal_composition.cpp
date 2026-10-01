#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/debug.hpp"
#include "behavior_tree_lite/dsl.hpp"
#include <iostream>
#include <variant>

using namespace bt;

struct Tick
{
};
struct Danger
{
};
using Event = std::variant<Tick, Danger>;

struct Context
{
    int battery = 100;
    bool object_visible = false;
};

struct CheckBattery : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &, Context &ctx)
    {
        if (ctx.battery > 20)
        {
            std::cout << "[CheckBattery] OK (" << ctx.battery << "%)\n";
            return Status::Success;
        }
        std::cout << "[CheckBattery] LOW (" << ctx.battery << "%)\n";
        return Status::Failure;
    }
    void reset() {}
};

struct Scan : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &, Context &ctx)
    {
        if (ctx.object_visible)
        {
            std::cout << "[Scan] Object spotted\n";
            return Status::Success;
        }
        std::cout << "[Scan] Scanning...\n";
        return Status::Running;
    }
    void reset() {}
};

struct Inspect : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &, Context &)
    {
        std::cout << "[Inspect] Pow!\n";
        return Status::Success;
    }
    void reset() {}
};

struct RunAway : NodeBase
{
    using EventType = Event;
    using ContextType = Context;
    Status process(const Event &, Context &)
    {
        std::cout << "[RunAway] Running away!\n";
        return Status::Success;
    }
    void reset() {}
};

int main()
{
    Context ctx;

    auto tree = (CheckBattery{} && (Scan{} || Inspect{})) || RunAway{};

    std::cout << "=== Behavior Tree Structure ===\n";
    print_tree(tree);
    std::cout << "===============================\n\n";

    std::cout << "--- Tick 1: Initial State ---\n";
    tree.process(Tick{}, ctx);

    std::cout << "\n--- Tick 2: Object appears ---\n";
    ctx.object_visible = true;
    tree.process(Tick{}, ctx);

    std::cout << "\n--- Tick 3: Low Battery ---\n";
    ctx.battery = 10;
    ctx.object_visible = false;
    tree.process(Tick{}, ctx);

    return 0;
}