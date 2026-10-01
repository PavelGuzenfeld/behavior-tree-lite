# behavior_tree_lite

A header-only behavior tree library for C++23. You compose the tree with
`&&`, `||` and `!`, and the compiler turns it into one nested type. Nothing
is dispatched through a virtual call and nothing goes on the heap, except
the one node built for that (`DynamicAction`).

```cpp
auto tree = (CheckBattery{} && Inspect{}) || ReturnHome{};
tree.process(Tick{}, ctx);
```

## Why it exists

Most behavior tree libraries build the tree at runtime: nodes on the heap,
virtual `tick()`, often an XML loader in front. That buys runtime editing
and visual tools. It costs an allocation per node and an indirect call per
tick, and a typo in the tree shows up when the robot runs, not when it
compiles.

If the tree is known when you build, none of that is needed. Here the tree
is a type. A node with the wrong event type or a missing `process()` is a
compile error, and the optimizer sees through the whole tree.

| | behavior_tree_lite | [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) |
|---|---|---|
| Dispatch | Templates, inlined | Virtual calls |
| Tree definition | C++ operators: `&&` `\|\|` `!` | XML or a runtime builder |
| Tree shape | Fixed at compile time | Loaded and changed at runtime |
| Allocations | None outside `DynamicAction` | Heap-allocated nodes |
| Header-only | Yes | No (shared library) |
| C++ standard | C++23 | C++17 |
| Good fit | Hot loops, embedded, game AI | Large trees, visual editors, logging |

Pick BehaviorTree.CPP if you need to change the tree without rebuilding, or
want Groot. Pick this one if the tree ships with the binary.

## Where to go next

- [Getting started](getting-started.md): install, then a three-node tree.
- [DSL](dsl.md): the operators, flattening, and how event and context
  types are found.
- [Nodes](nodes.md): every composite, decorator and leaf, and exactly what
  each returns.
- [Events and debugging](events.md): `std::variant` events and
  `print_tree`.
- [ROS 2 example](ros2.md) and [PX4 SITL example](px4.md).
- [Performance](performance.md): what the benchmark measures, and what it
  doesn't.
