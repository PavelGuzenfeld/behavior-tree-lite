# behavior_tree_lite

A lightweight, header-only, compile-time behavior tree library for C++23/26.

## Features

- **Header-only**: Zero compile-time dependencies (other than STL).
- **Compile-time Composition**: Tree structure is flattened at compile time.
- **Expressive DSL**: Use `&&`, `||`, and `!` to compose trees naturally.
- **Stack-based**: Events use `std::variant` (no virtual dispatch for events).
- **Debug Tools**: Built-in tree visualizer.
- **ROS2 Ready**: Includes `package.xml`, `ament_cmake` integration, and example nodes.

## Installation

### ROS2 Workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourname/behavior_tree_lite.git
cd ..
colcon build --packages-select behavior_tree_lite
source install/setup.bash
```

### Standalone CMake
```bash
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build .
ctest --output-on-failure
sudo cmake --install .
```

## Quick Start

### 1. Define Your Domain

Define the **Events** your robot reacts to and the **Context** (blackboard) it modifies.

```cpp
#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/dsl.hpp"
#include <variant>

using namespace bt;

struct Tick {};
struct Danger {};
using Event = std::variant<Tick, Danger>;

struct Context { 
    int battery = 100;
};
```

### 2. Define Leaf Nodes

Nodes are simple structs with `process()` and `reset()` methods.

```cpp
struct CheckBattery : NodeBase {
    using EventType = Event;      // Required for DSL
    using ContextType = Context;  // Required for DSL

    Status process(const Event&, Context& ctx) {
        return ctx.battery > 20 ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct Attack : NodeBase {
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event&, Context&) {
        std::cout << "Attack!\n";
        return Status::Success;
    }
    void reset() {}
};

struct RunAway : NodeBase {
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event&, Context&) {
        std::cout << "Fleeing!\n";
        return Status::Success;
    }
    void reset() {}
};
```

### 3. Compose the Tree (DSL)

Use logical operators to build the tree:

* `&&` = **Sequence** (Run left, if Success, run right)
* `||` = **Selector** (Run left, if Failure, run right)
* `!`  = **Inverter** (Flip Success/Failure)

```cpp
int main() {
    Context ctx;

    // Logic: (CheckBattery AND Attack) OR RunAway
    auto tree = (CheckBattery{} && Attack{}) || RunAway{};

    // Visualize
    bt::print_tree(tree);

    // Execute
    tree.process(Tick{}, ctx);
}
```

## DSL Reference

| Operator | Node Type | Logic |
|----------|-----------|-------|
| `A && B` | `Sequence<...>` | Runs `A`. If Success, runs `B`. Fails if any fail. |
| `A \|\| B` | `Selector<...>` | Runs `A`. If Failure, runs `B`. Succeeds if any succeed. |
| `!A` | `Inverter<...>` | Returns Failure if `A` succeeds, and vice versa. |

**Precedence:** `&&` binds tighter than `||`, so `A && B || C` means `(A && B) || C`.

**Flattening:** Consecutive sequences/selectors are automatically flattened:
```cpp
// These are equivalent:
auto tree1 = (A{} && B{}) && C{};  // Flattened to Sequence<A, B, C>
auto tree2 = A{} && B{} && C{};    // Also Sequence<A, B, C>
```

## Type Deduction

The DSL needs to know your `Event` and `Context` types. Three methods are supported:

### Method 1: Explicit Typedefs (Recommended)

```cpp
struct MyNode : NodeBase {
    using EventType = Event;      // DSL reads this
    using ContextType = Context;  // DSL reads this
    
    Status process(const Event&, Context&) { return Status::Success; }
    void reset() {}
};
```

### Method 2: Process Signature Deduction

For simple nodes without templates, the DSL can deduce types from `process()`:

```cpp
struct SimpleNode {
    Status process(const Event&, Context&) { return Status::Success; }
    void reset() {}
};
// Works! Types deduced from process() signature.
auto tree = SimpleNode{} && OtherNode{};
```

### Method 3: Library Nodes

All library-provided composites (`Sequence`, `Selector`, `Inverter`, etc.) carry their types as template parameters and work automatically:

```cpp
auto retry_scan = make_retry<Event, Context>(3, MyScanner{});
auto tree = retry_scan && Attack{};  // Types flow from retry_scan
```

**Note:** Method 2 (signature deduction) does NOT work with C++23 deducing-this (`this auto&&`) or templated `process()` methods. Use Method 1 for those.

## Decorators

```cpp
// Retry up to 3 times on failure
auto retry = make_retry<Event, Context>(3, ScanNode{});

// Repeat 5 times (or -1 for infinite)
auto repeat = make_repeat<Event, Context>(5, PatrolNode{});

// Timeout after 10 ticks
auto timeout = Timeout<Event, Context, ScanNode>(10, ScanNode{});

// Always succeed (ignore child failure)
auto safe = Succeeder<Event, Context, RiskyNode>(RiskyNode{});

// Always fail (ignore child success)  
auto fail = Failer<Event, Context, CheckNode>(CheckNode{});

// Conditional execution
auto guard = Guard<Event, Context, decltype(pred), ActionNode>(
    [](const Context& ctx) { return ctx.battery > 50; },
    ActionNode{}
);
```

## ROS2 Integration

### Patrol Robot Example

A complete ROS2 example is included in `examples/patrol_robot_node.cpp`:

```bash
# Build
colcon build --packages-select behavior_tree_lite

# Run with visualization
ros2 launch behavior_tree_lite patrol_demo.launch.py
```

The example demonstrates:
- Multi-event handling (`TickEvent`, `BatteryUpdate`, `LaserUpdate`, `EmergencyStop`)
- DSL-composed behavior tree
- RViz2 visualization with markers
- Terminal UI with status display

**Tree Structure:**
```cpp
auto tree = 
    (!CheckEmergency{} && Halt{})                              // Emergency stop
    || (!CheckBattery{} && GoToCharger{} && Charge{})          // Low battery
    || (CheckBattery{} && ((CheckObstacle{} && Navigate{}) || Avoid{}))  // Patrol
    || Idle{};                                                  // Fallback
```

**Test Commands:**
```bash
ros2 topic pub /battery std_msgs/Float32 "{data: 15.0}" --once
ros2 topic pub /scan sensor_msgs/LaserScan "{ranges: [0.3]}" --once
ros2 topic pub /estop std_msgs/Bool "{data: true}" --once
```

## Event Handling with `std::variant`

Use `std::visit` with the `overloaded` helper to dispatch events:

```cpp
using Event = std::variant<Tick, BatteryUpdate, EnemySpotted>;

struct CheckBattery : NodeBase {
    using EventType = Event;
    using ContextType = Context;
    
    Status process(const Event& e, Context& ctx) {
        std::visit(overloaded{
            [&](const BatteryUpdate& b) { ctx.battery = b.level; },
            [](const auto&) {}  // Ignore other events
        }, e);
        
        return ctx.battery > 20 ? Status::Success : Status::Failure;
    }
    void reset() {}
};
```

## Debug Visualization

```cpp
#include <behavior_tree_lite/debug.hpp>

auto tree = (A{} && B{}) || C{};
bt::print_tree(tree);
```

Output:
```
Selector
  Sequence
    A
    B
  C
```

## API Reference

### Factory Functions

```cpp
make_sequence<E, C>(children...)   // Create Sequence
make_selector<E, C>(children...)   // Create Selector
make_parallel<E, C>(children...)   // Create Parallel
make_inverter<E, C>(child)         // Create Inverter
make_retry<E, C>(attempts, child)  // Create Retry
make_repeat<E, C>(count, child)    // Create Repeat (-1 = infinite)
```

### Node Types

| Type | Description |
|------|-------------|
| `Sequence` | Runs children in order. Fails fast. |
| `Selector` | Tries children until one succeeds. |
| `Parallel` | Runs all children each tick. Fails if any fail. |
| `Inverter` | Flips Success/Failure. |
| `Retry` | Retries child N times on failure. |
| `Repeat` | Repeats child N times (or forever). |
| `Timeout` | Fails if child doesn't complete in N ticks. |
| `Succeeder` | Always returns Success (unless Running). |
| `Failer` | Always returns Failure (unless Running). |
| `Guard` | Runs child only if predicate is true. |

### Leaf Helpers

```cpp
// Type-erased action (runtime flexibility)
DynamicAction<E, C> action(
    [](const E&, C&) { return Status::Success; },
    []() { /* reset */ }
);

// Always-return nodes
AlwaysSuccess<E, C>{}
AlwaysFailure<E, C>{}
AlwaysRunning<E, C>{}
```

## Requirements

- C++23 or later
- CMake 3.22+
- GTest (for tests)
- ROS2 Humble/Jazzy (optional, for ROS2 integration)

## Roadmap
See [ROADMAP.md](ROADMAP.md) for the detailed development plan. Upcoming v0.0.2 features include:

## License

MIT License. See [LICENSE](LICENSE) for details.