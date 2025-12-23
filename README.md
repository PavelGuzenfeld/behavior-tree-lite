# behavior_tree_lite

A lightweight, header-only, compile-time behavior tree library for C++23/26.

## Features

- **Header-only**: Zero compile-time dependencies (other than STL).
- **Compile-time Composition**: Tree structure is flattened at compile time.
- **Expressive DSL**: Use `&&`, `||`, and `!` to compose trees naturally.
- **Stack-based**: Events use `std::variant` (no virtual dispatch for events).
- **Debug Tools**: Built-in tree visualizer.
- **ROS2 Ready**: Includes `package.xml` and `ament_cmake` integration.

## Installation

### ROS2 Workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourname/behavior_tree_lite.git
colcon build --packages-select behavior_tree_lite
```

### Standalone CMake
```bash
mkdir build && cd build
cmake ..
cmake --build .
sudo cmake --install .
```

## Quick Start

### 1. Define Your Domain

Define the **Events** your robot reacts to and the **Context** (blackboard) it modifies.
```cpp
#include <behavior_tree_lite/behavior_tree.hpp>
#include <behavior_tree_lite/dsl.hpp>   // Required for &&, ||, ! operators
#include <behavior_tree_lite/debug.hpp> // Required for print_tree
#include <iostream>
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

Nodes are simple structs with a `process` method.
```cpp
struct CheckBattery {
    Status process(const Event&, Context& ctx) {
        return ctx.battery > 20 ? Status::Success : Status::Failure;
    }
    void reset() {}
};

struct Attack {
    Status process(const Event&, Context&) {
        std::cout << "Attack!\n";
        return Status::Success;
    }
    void reset() {}
};

struct RunAway {
    Status process(const Event&, Context&) {
        std::cout << "Fleeing!\n";
        return Status::Success;
    }
    void reset() {}
};
```

### 3. Compose the Tree (DSL)

Use the logical operators to build the tree.

* `&&` = **Sequence** (Run left, if Success, run right)
* `||` = **Selector** (Run left, if Failure, run right)
* `!`  = **Inverter** (Flip Success/Failure)
```cpp
int main() {
    Context ctx;

    // Logic: (CheckBattery AND Attack) OR RunAway
    auto tree = 
        (CheckBattery{} && Attack{}) || RunAway{};

    // Visualizing the tree
    std::cout << "Tree Structure:\n";
    bt::print_tree(tree);

    // Execution
    tree.process(Tick{}, ctx);
}
```

## DSL Reference

| Operator | Node Type | Logic |
|----------|-----------|-------|
| `A && B` | `Sequence<...>` | Runs `A`. If Success, runs `B`. Fails if any fail. |
| `A \|\| B` | `Selector<...>` | Runs `A`. If Failure, runs `B`. Succeeds if any succeed. |
| `!A` | `Inverter<...>` | Returns Failure if `A` succeeds, and vice versa. |

**Note on Precedence:**
`&&` binds tighter than `||`.
`A && B || C` is interpreted as `(A && B) || C`.

## Advanced Usage

### Decorators

Decorators like `Retry`, `Repeat`, and `Timeout` can be wrapped around nodes manually or via helper functions.
```cpp
// Retrying a scan operation 3 times before failing
auto complex_tree = 
    bt::make_retry<Event, Context>(3, Scan{}) && Attack{};
```

### Custom Complex Nodes

If your node uses templates (like `process(this auto&&...)`), automatic type deduction might fail. You must explicitly define `EventType` and `ContextType` in your struct:
```cpp
struct MyComplexNode : bt::NodeBase {
    using EventType = MyEvent;   // <--- Required for DSL
    using ContextType = MyCtx;   // <--- Required for DSL

    template <typename Self>
    Status process(this Self&& self, const EventType& e, ContextType& ctx) {
        return Status::Success;
    }
    void reset() {}
};
```

## License

MIT License.