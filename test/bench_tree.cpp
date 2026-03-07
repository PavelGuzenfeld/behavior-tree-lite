#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/dsl.hpp"
#include <chrono>
#include <cstdio>
#include <variant>

using namespace bt;

// ==========================================
// BENCHMARK TYPES
// ==========================================

struct TickEvent
{
    double dt = 0.016;
};
struct SensorEvent
{
    int value = 42;
};
using Event = std::variant<TickEvent, SensorEvent>;

struct Context
{
    int counter = 0;
    bool flag = true;
};

// ==========================================
// BENCHMARK NODES
// ==========================================

struct IncrementNode : NodeBase
{
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event &, Context &ctx)
    {
        ctx.counter++;
        return Status::Success;
    }
    void reset() {}
};

struct CheckFlag : NodeBase
{
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event &, Context &ctx) { return ctx.flag ? Status::Success : Status::Failure; }
    void reset() {}
};

struct RunningFor : NodeBase
{
    int ticks_needed;
    int current = 0;

    explicit RunningFor(int n) : ticks_needed(n) {}

    Status process(const Event &, Context &)
    {
        if (++current >= ticks_needed)
        {
            current = 0;
            return Status::Success;
        }
        return Status::Running;
    }
    void reset() { current = 0; }
};

// ==========================================
// BENCHMARK HARNESS
// ==========================================

// Prevent the compiler from optimizing away the result
template <typename T> void do_not_optimize(T const &value)
{
    asm volatile("" : : "r,m"(value) : "memory");
}

struct BenchResult
{
    const char *name;
    std::size_t iterations;
    double ns_per_op;
};

template <typename Fn> BenchResult bench(const char *name, std::size_t iterations, Fn &&fn)
{
    // Warmup
    for (std::size_t i = 0; i < iterations / 10; ++i)
        fn();

    auto start = std::chrono::high_resolution_clock::now();
    for (std::size_t i = 0; i < iterations; ++i)
        fn();
    auto end = std::chrono::high_resolution_clock::now();

    double ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    return {name, iterations, ns / static_cast<double>(iterations)};
}

void print_result(const BenchResult &r)
{
    std::printf("  %-45s %10.1f ns/op  (%zu iterations)\n", r.name, r.ns_per_op, r.iterations);
}

// ==========================================
// BENCHMARKS
// ==========================================

int main()
{
    constexpr std::size_t N = 1'000'000;

    std::printf("behavior_tree_lite v%d.%d.%d — Performance Benchmarks\n", version.major, version.minor, version.patch);
    std::printf("=====================================================\n\n");

    // --- Leaf node throughput ---
    std::printf("Leaf Nodes:\n");
    {
        Context ctx;
        IncrementNode node;
        print_result(bench("Single Action node", N, [&] { do_not_optimize(node.process(TickEvent{}, ctx)); }));
    }
    {
        Context ctx;
        DynamicAction<Event, Context> node(
            [](const Event &, Context &c)
            {
                c.counter++;
                return Status::Success;
            });
        print_result(
            bench("DynamicAction (std::function)", N, [&] { do_not_optimize(node.process(TickEvent{}, ctx)); }));
    }
    {
        Context ctx;
        AlwaysSuccess<Event, Context> node;
        print_result(bench("AlwaysSuccess (stateless)", N, [&] { do_not_optimize(node.process(TickEvent{}, ctx)); }));
    }

    // --- Composite throughput ---
    std::printf("\nComposite Nodes:\n");
    {
        Context ctx;
        auto seq = make_sequence<Event, Context>(IncrementNode{}, IncrementNode{}, IncrementNode{});
        print_result(bench("Sequence(3 children)", N, [&] { do_not_optimize(seq.process(TickEvent{}, ctx)); }));
    }
    {
        Context ctx;
        auto sel = make_selector<Event, Context>(CheckFlag{}, IncrementNode{});
        print_result(
            bench("Selector(2 children, first wins)", N, [&] { do_not_optimize(sel.process(TickEvent{}, ctx)); }));
    }
    {
        Context ctx;
        auto par = make_parallel<Event, Context>(IncrementNode{}, IncrementNode{}, IncrementNode{});
        print_result(bench("Parallel(3 children)", N, [&] { do_not_optimize(par.process(TickEvent{}, ctx)); }));
    }

    // --- Decorator throughput ---
    std::printf("\nDecorator Nodes:\n");
    {
        Context ctx;
        auto inv = make_inverter<Event, Context>(IncrementNode{});
        print_result(bench("Inverter(Action)", N, [&] { do_not_optimize(inv.process(TickEvent{}, ctx)); }));
    }
    {
        Context ctx;
        auto guard = make_guard<Event, Context>([](const Context &c) { return c.flag; }, IncrementNode{});
        print_result(bench("Guard(predicate, Action)", N, [&] { do_not_optimize(guard.process(TickEvent{}, ctx)); }));
    }

    // --- DSL-composed tree ---
    std::printf("\nDSL-Composed Trees:\n");
    {
        Context ctx;
        auto tree = (CheckFlag{} && IncrementNode{}) || IncrementNode{};
        print_result(
            bench("(Check && Action) || Fallback", N, [&] { do_not_optimize(tree.process(TickEvent{}, ctx)); }));
    }

    // --- Deep nesting ---
    std::printf("\nDeep Nesting:\n");
    {
        Context ctx;
        // 5-level deep tree
        auto tree = make_selector<Event, Context>(
            make_sequence<Event, Context>(
                CheckFlag{},
                make_selector<Event, Context>(make_sequence<Event, Context>(IncrementNode{}, IncrementNode{}),
                                              make_inverter<Event, Context>(IncrementNode{}))),
            IncrementNode{});
        print_result(bench("5-level nested tree", N, [&] { do_not_optimize(tree.process(TickEvent{}, ctx)); }));
    }

    // --- Running/resume overhead ---
    std::printf("\nRunning State (resume overhead):\n");
    {
        Context ctx;
        auto seq = make_sequence<Event, Context>(IncrementNode{}, RunningFor(100), IncrementNode{});
        print_result(
            bench("Sequence with Running child (resume)", N, [&] { do_not_optimize(seq.process(TickEvent{}, ctx)); }));
    }

    // --- Variant event dispatch ---
    std::printf("\nVariant Event Dispatch:\n");
    {
        Context ctx;
        struct VariantNode : NodeBase
        {
            Status process(const Event &e, Context &ctx)
            {
                std::visit(overloaded{[&](const TickEvent &) { ctx.counter++; },
                                      [](const SensorEvent &) {
                                      }},
                           e);
                return Status::Success;
            }
            void reset() {}
        };

        VariantNode node;
        Event tick = TickEvent{};
        print_result(bench("std::visit dispatch (TickEvent)", N, [&] { do_not_optimize(node.process(tick, ctx)); }));

        Event sensor = SensorEvent{42};
        print_result(
            bench("std::visit dispatch (SensorEvent)", N, [&] { do_not_optimize(node.process(sensor, ctx)); }));
    }

    std::printf("\nDone.\n");
    return 0;
}
