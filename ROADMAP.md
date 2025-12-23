# Roadmap & To-Do

This document tracks the planned features, improvements, and known limitations of `behavior_tree_lite`.

**Current Version:** 0.0.1

## 🚧 Upcoming Features (Targeting v0.0.2)

### Core Library Refactoring
- [ ] **Functor-based Nodes**: Replace `process()` with `operator()` to allow nodes to function as standard C++ callables.
- [ ] **Optional Reset**: Make `reset()` optional using Concepts (if not implemented, it does nothing).
- [ ] **Lambda Support**: Add first-class support for creating leaf nodes directly from lambdas without boilerplate structs.

### Testing Framework
- [ ] **Switch to Doctest**: Replace GTest with `doctest`. Use `#include <doctest/doctest.h>` instead of `gtest.h` to maintain the header-only philosophy and improve compile times.

### Project Structure & ROS2
- [ ] **Sub-package Split**: Move ROS2-specific assets (`urdf/`, `rviz/`, `launch/`) and nodes into a dedicated `examples/bt_ros2_examples` sub-package.
- [ ] **Isolated Build**: Create a separate `CMakeLists.txt` for the ROS2 examples to decouple them from the core header-only library.
- [ ] **Docker Environment**: Create a `Dockerfile` bundling all dependencies (ROS2, GTest/Doctest, build tools) for consistent development and testing.

### DevOps & Quality
- [ ] **Pre-commit Hooks**: Setup `pre-commit` for local linting.
- [ ] **Linting**: Enforce `clang-format` and `clang-tidy` on all commits.
- [ ] **Enhanced CI**: Update GitHub Actions to run full tests on Pull Requests and commits to `master`.

## 🔮 Future Backlog

### Core Features
- [ ] **Blackboard Typing**: Add type-safe accessors (e.g., `ctx.get<int>("key")`).

### Tooling
- [ ] **Graphviz Export**: Add `to_dot()` to visualize the tree structure.
- [ ] **Benchmarks**: Compare compile-time vs run-time composition performance.

## ✅ Completed (v0.0.1)
- [x] Initial header-only library structure.
- [x] C++23 "Deducing This" support.
- [x] Operator Overloading DSL (`&&`, `||`, `!`).
- [x] Basic ROS2 Humble/Jazzy CI integration.
- [x] PX4 Drone and Patrol Robot examples.