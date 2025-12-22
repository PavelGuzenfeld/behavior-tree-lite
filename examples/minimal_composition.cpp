#include <behavior_tree_lite/behavior_tree.hpp>
#include <behavior_tree_lite/dsl.hpp>
#include <iostream>
#include <variant>

using namespace bt;

// --- Domain ---
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
    bool enemy_visible = false;
};

// --- Nodes ---

struct CheckBattery : NodeBase
{
    Status process(const Event &, Context &ctx)
    {
        // ADDED: Print status
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
    Status process(const Event &, Context &ctx)
    {
        // ADDED: Print status
        if (ctx.enemy_visible)
        {
            std::cout << "[Scan] Enemy Spotted!\n";
            return Status::Success;
        }
        std::cout << "[Scan] Scanning...\n";
        return Status::Running;
    }
    void reset() {}
};

struct Attack : NodeBase
{
    Status process(const Event &, Context &)
    {
        std::cout << "[Attack] Pow!\n";
        return Status::Success;
    }
    void reset() {}
};

struct RunAway : NodeBase
{
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

    // Tree: (CheckBattery -> (Scan ? Attack)) ? RunAway
    auto tree =
        (CheckBattery{} && (Scan{} || Attack{})) || RunAway{};

    // Grouping is implicit: (CheckBattery && (Scan || Attack)) || RunAway

    std::cout << "--- Tick 1: Initial State ---\n";
    tree.process(Tick{}, ctx); // Prints: Battery OK, Scanning...

    std::cout << "\n--- Tick 2: Enemy Appears ---\n";
    ctx.enemy_visible = true;
    tree.process(Tick{}, ctx); // Prints: Enemy Spotted! (Sequence completes)

    std::cout << "\n--- Tick 3: Low Battery ---\n";
    ctx.battery = 10;
    ctx.enemy_visible = false; // Reset enemy
    tree.process(Tick{}, ctx); // Prints: Battery LOW, Running away!

    return 0;
}