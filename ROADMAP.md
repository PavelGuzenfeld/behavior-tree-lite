# Roadmap & To-Do

This document tracks the planned features, improvements, and known limitations of `behavior_tree_lite`.

**Current Version:** 0.3.0

## 🚧 Upcoming Features (Targeting v0.4.0)

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

## 🔮 Future Backlog

### Core Features
- [ ] **Blackboard Typing**: Add type-safe accessors (e.g., `ctx.get<int>("key")`).

### Tooling
- [ ] **Graphviz Export**: Add `to_dot()` to visualize the tree structure.

## ✅ Completed (v0.3.0)
- [x] Performance benchmarks (`BUILD_BENCHMARKS=ON`) with `do_not_optimize` barrier.
- [x] ASan + UBSan CI job (sanitizers).
- [x] Benchmark CI job (runs on every push).
- [x] GitHub Release workflow (`release.yml`): triggers on `v*` tags, validates version against CMakeLists.txt, creates release with header tarball.

## ✅ Completed (v0.2.0)
- [x] `make_guard` factory helper and full Guard test coverage.
- [x] DynamicAction null-callback assertion.
- [x] Parallel fail-fast behavior documented.
- [x] `clang-format` enforcement in CI with `.clang-format` config.
- [x] `clang-tidy` made blocking in CI (removed `|| true`).
- [x] Edge case tests: single-child composites, deeply nested trees (82 tests total).
- [x] Thread-safety constraints documented in README.
- [x] Fixed duplicate CI matrix entry, stale C++26 references.

## ✅ Completed (v0.0.1)
- [x] Initial header-only library structure.
- [x] C++23 "Deducing This" support.
- [x] Operator Overloading DSL (`&&`, `||`, `!`).
- [x] Basic ROS2 Humble/Jazzy CI integration.
- [x] PX4 Drone and Patrol Robot examples.
