# behavior_tree_lite

A lightweight, header-only, compile-time behavior tree library for C++26.

## Features

- **Header-only**: No compilation required, just include and use
- **Zero overhead**: Compile-time node composition using variadic templates
- **Stack-based polymorphism**: Events use `std::variant` instead of virtual dispatch
- **C++23/26 Features**: Deducing `this`, `[[no_unique_address]]`, `std::to_underlying`
- **C++20 Concepts**: Type-safe node composition with clear error messages
- **ROS2 ready**: Includes `package.xml` and ament CMake integration
- **Standalone support**: Works without ROS2 as a pure CMake project

## Requirements

- C++26 compiler (GCC 15+, Clang 21+) 
- CMake 3.14+
- (Optional) ROS2 Humble/Jazzy

## Installation

### ROS2 Workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/example/behavior_tree_lite.git
cd ~/ros2_ws
colcon build --packages-select behavior_tree_lite
```

### Standalone CMake

```bash
git clone https://github.com/example/behavior_tree_lite.git
cd behavior_tree_lite
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

## Quick Start

```cpp
#include <behavior_tree_lite/behavior_tree.hpp>
#include <variant>

using namespace bt;

// Define your events
struct TickEvent { double dt; };
struct SensorEvent { int value; };
using Event = std::variant<TickEvent, SensorEvent>;

// Define your context (blackboard)
struct Context {
    int battery = 100;
    bool target_found = false;
};

// Create leaf nodes
struct CheckBattery : NodeBase {
    Status process(const Event&, Context& ctx) {
        return (ctx.battery > 20) ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct Search : NodeBase {
    Status process(const Event& e, Context& ctx) {
        return std::visit(overloaded{
            [&](const SensorEvent& s) {
                ctx.target_found = (s.value > 0);
                return ctx.target_found ? Status::Success : Status::Running;
            },
            [](const auto&) { return Status::Running; }
        }, e);
    }
    void reset() {}
};

// Compose the tree
int main() {
    Context ctx;
    
    Sequence<Event, Context, CheckBattery, Search> tree(
        CheckBattery{},
        Search{}
    );
    
    // Tick the tree
    Status s = tree.process(TickEvent{0.1}, ctx);
}
```

## Node Types

### Composite Nodes

| Node | Description |
|------|-------------|
| `Sequence<E,C,...>` | Runs children in order. Fails if any fails. Succeeds when all succeed. |
| `Selector<E,C,...>` | Tries children in order. Succeeds if any succeeds. Fails when all fail. |
| `Parallel<E,C,...>` | Runs all children each tick. Succeeds when all succeed. Fails if any fails. |

### Decorator Nodes

| Node | Description |
|------|-------------|
| `Inverter<E,C,Child>` | Inverts Success↔Failure. Passes through Running. |
| `Retry<E,C,Child>` | Retries child N times on failure. |
| `Repeat<E,C,Child>` | Repeats child N times (or forever if N < 0). |
| `Succeeder<E,C,Child>` | Always returns Success (unless Running). |
| `Failer<E,C,Child>` | Always returns Failure (unless Running). |
| `Timeout<E,C,Child>` | Fails if child doesn't complete within N ticks. |
| `Guard<E,C,Pred,Child>` | Only processes child if predicate(ctx) is true. |

### Leaf Nodes

| Node | Description |
|------|-------------|
| `Action<E,C,F>` | Zero-overhead wrapper for callable. Supports `Status`, `bool`, or `void` return. |
| `StatefulAction<E,C,S,F>` | Action with resetable internal state. |
| `Condition<E,C,Pred>` | Wraps a predicate, returns Success/Failure (never Running). |
| `DynamicAction<E,C>` | Type-erased `std::function` fallback for runtime polymorphism. |
| `AlwaysSuccess<E,C>` | Always returns Success. |
| `AlwaysFailure<E,C>` | Always returns Failure. |
| `AlwaysRunning<E,C>` | Always returns Running. |

## Creating Custom Nodes

Any type satisfying the `IsNode` concept works:

```cpp
template <typename T, typename Event, typename Context>
concept IsNode = requires(T t, const Event& e, Context& ctx) {
    { t.process(e, ctx) } -> std::same_as<Status>;
    { t.reset() } -> std::same_as<void>;
};
```

Example:

```cpp
struct MyAction : bt::NodeBase {
    int state = 0;
    
    bt::Status process(const Event& e, Context& ctx) {
        // Your logic here
        return bt::Status::Success;
    }
    
    void reset() {
        state = 0;
    }
};
```

## Factory Helpers

For cleaner tree construction:

```cpp
auto tree = make_sequence<Event, Context>(
    make_selector<Event, Context>(
        CheckBattery{},
        ChargeBattery{}
    ),
    make_retry<Event, Context>(3, 
        ScanForTarget{}
    ),
    make_inverter<Event, Context>(
        IsEnemyNearby{}
    )
);
```

## Event Handling with std::variant

Use the `overloaded` helper for variant visitation:

```cpp
Status process(const Event& e, Context& ctx) {
    return std::visit(bt::overloaded{
        [&](const TickEvent& t) {
            // Handle tick
            return Status::Running;
        },
        [&](const SensorEvent& s) {
            // Handle sensor
            return Status::Success;
        },
        [](const auto&) {
            // Ignore other events
            return Status::Running;
        }
    }, e);
}
```

## Testing

```bash
# ROS2
colcon test --packages-select behavior_tree_lite
colcon test-result --verbose

# Standalone
cmake -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## License

MIT License. See [LICENSE](LICENSE) for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure CI passes
5. Submit a pull request
