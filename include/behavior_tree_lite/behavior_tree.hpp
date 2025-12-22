#pragma once

/// @file behavior_tree.hpp
/// @brief Header-only Behavior Tree library for C++26
///
/// A lightweight, compile-time behavior tree implementation using
/// modern C++ features like concepts, variadic templates, and
/// fold expressions.

#include "types.hpp"
#include "nodes/composite.hpp"
#include "nodes/decorator.hpp"
#include "nodes/leaf.hpp"

namespace bt {

/// @brief Library version
inline constexpr struct {
    int major = 1;
    int minor = 0;
    int patch = 0;
} version;

// ==========================================
// FACTORY HELPERS
// ==========================================

/// Create a Sequence node
template <typename Event, typename Context, IsNode<Event, Context>... Children>
constexpr auto make_sequence(Children&&... children) {
    return Sequence<Event, Context, std::decay_t<Children>...>(
        std::forward<Children>(children)...);
}

/// Create a Selector node
template <typename Event, typename Context, IsNode<Event, Context>... Children>
constexpr auto make_selector(Children&&... children) {
    return Selector<Event, Context, std::decay_t<Children>...>(
        std::forward<Children>(children)...);
}

/// Create a Parallel node
template <typename Event, typename Context, IsNode<Event, Context>... Children>
constexpr auto make_parallel(Children&&... children) {
    return Parallel<Event, Context, std::decay_t<Children>...>(
        std::forward<Children>(children)...);
}

/// Create an Inverter node
template <typename Event, typename Context, IsNode<Event, Context> Child>
constexpr auto make_inverter(Child&& child) {
    return Inverter<Event, Context, std::decay_t<Child>>(
        std::forward<Child>(child));
}

/// Create a Retry node
template <typename Event, typename Context, IsNode<Event, Context> Child>
constexpr auto make_retry(int attempts, Child&& child) {
    return Retry<Event, Context, std::decay_t<Child>>(
        attempts, std::forward<Child>(child));
}

/// Create a Repeat node
template <typename Event, typename Context, IsNode<Event, Context> Child>
constexpr auto make_repeat(int iterations, Child&& child) {
    return Repeat<Event, Context, std::decay_t<Child>>(
        iterations, std::forward<Child>(child));
}

}  // namespace bt
