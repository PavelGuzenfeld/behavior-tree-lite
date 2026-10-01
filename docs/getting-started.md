# Getting started

## Requirements

- C++23, with deducing `this`: GCC 14+ or Clang 18+
- CMake 3.22+
- doctest, only for the tests (`apt install doctest-dev`)
- ROS 2 Jazzy, only for the ROS examples

## Install

The library is headers only. Any of these works.

=== "CMake FetchContent"

    ```cmake
    include(FetchContent)
    FetchContent_Declare(behavior_tree_lite
      GIT_REPOSITORY https://github.com/PavelGuzenfeld/behavior-tree-lite.git
      GIT_TAG v0.3.1)
    set(BUILD_EXAMPLES OFF)
    FetchContent_MakeAvailable(behavior_tree_lite)
    target_link_libraries(my_app PRIVATE behavior_tree_lite::behavior_tree_lite)
    ```

=== "Installed package"

    ```bash
    cmake -B build -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    sudo cmake --install build
    ```

    ```cmake
    find_package(behavior_tree_lite REQUIRED)
    target_link_libraries(my_app PRIVATE behavior_tree_lite::behavior_tree_lite)
    ```

=== "ROS 2 workspace"

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/PavelGuzenfeld/behavior-tree-lite.git
    cd .. && colcon build --packages-select behavior_tree_lite
    ```

To build and run the tests:

```bash
cmake -B build -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## A first tree

### 1. Events and context

Events are what the tree reacts to. The context is the blackboard the nodes
read and write. Both are your own types.

```cpp
#include <behavior_tree_lite/behavior_tree.hpp>
#include <behavior_tree_lite/dsl.hpp>
#include <iostream>
#include <variant>

using namespace bt;

struct Tick {};
struct Low {};
using Event = std::variant<Tick, Low>;

struct Context {
    int battery = 100;
};
```

### 2. Leaf nodes

A node is a struct with `process()`. The two typedefs let the
operators find your event and context types.

```cpp
struct CheckBattery : NodeBase {
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event&, Context& ctx) {
        return ctx.battery > 20 ? Status::Success : Status::Failure;
    }
};

struct Inspect : NodeBase {
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event&, Context&) {
        std::cout << "Inspecting\n";
        return Status::Success;
    }
};

struct ReturnHome : NodeBase {
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event&, Context&) {
        std::cout << "Heading to the dock\n";
        return Status::Success;
    }
};
```

### 3. Compose and tick

```cpp
int main() {
    Context ctx;
    auto tree = (CheckBattery{} && Inspect{}) || ReturnHome{};

    tree.process(Tick{}, ctx);   // Inspecting
    ctx.battery = 10;
    tree.process(Tick{}, ctx);   // Heading to the dock
}
```

`&&` is a sequence, `||` a selector, `!` an inverter. The [DSL page](dsl.md)
has the details.

## Threads

Nodes keep mutable state: the running child's index, tick counters. None of
it is synchronized. Tick a tree from one thread, the way a game loop or a
ROS 2 timer does. If several threads must reach it, put a mutex around
`process()`.
