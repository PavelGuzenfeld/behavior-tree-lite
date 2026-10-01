# Events and debugging

## Events as `std::variant`

The tree is ticked with an event, not only a clock. Make the event type a
`std::variant` and let each node pick out what it cares about with
`bt::overloaded`:

```cpp
struct Tick {};
struct BatteryUpdate { int level; };
struct ObjectSpotted { float distance; };
using Event = std::variant<Tick, BatteryUpdate, ObjectSpotted>;

struct CheckBattery : NodeBase {
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event& e, Context& ctx) {
        std::visit(overloaded{
            [&](const BatteryUpdate& b) { ctx.battery = b.level; },
            [](const auto&) {}
        }, e);
        return ctx.battery > 20 ? Status::Success : Status::Failure;
    }
};
```

Sensor callbacks push their message as an event, and a timer pushes `Tick`.
The [ROS 2 example](ros2.md) works this way.

## Printing a tree

`<behavior_tree_lite/debug.hpp>` prints the tree's shape from its type:

```cpp
#include <behavior_tree_lite/debug.hpp>

auto tree = (A{} && B{}) || C{};
bt::print_tree(tree);
```

```text
Selector
  Sequence
    A
    B
  C
```

Decorators print their parameter, such as `Retry (3x)` or
`Timeout (10 ticks)`. Leaf names come from the compiler's type name, so
they read best for non-template structs. `print_tree` takes an optional
`std::ostream&`, which defaults to `std::cout`.
