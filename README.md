# behavior_tree_lite

![C++](https://img.shields.io/badge/C++-23-00599C?style=flat&logo=cplusplus&logoColor=white)
![Header Only](https://img.shields.io/badge/Header--Only-yes-brightgreen)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

Header-only behavior trees for C++23. Compose the tree with `&&`, `||` and
`!`. It becomes one nested type: no virtual calls, no heap allocations
outside `DynamicAction`, and mistakes show up as compile errors.

```cpp
auto tree = (CheckBattery{} && Inspect{}) || ReturnHome{};
tree.process(Tick{}, ctx);
```

**Docs: [pavelguzenfeld.com/behavior-tree-lite](https://pavelguzenfeld.com/behavior-tree-lite/)**

```bash
cmake -B build -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build build && ctest --test-dir build --output-on-failure
```

Needs GCC 14+ or Clang 18+. ROS 2 Jazzy is optional, for the examples.
Planned work is in [issues](https://github.com/PavelGuzenfeld/behavior-tree-lite/issues).

MIT licensed. Design notes on [pavelguzenfeld.com](https://pavelguzenfeld.com/projects/behavior-tree-lite/).
