#pragma once

#include "blackboard.hpp"
#include "nodes/composite.hpp"
#include "nodes/decorator.hpp"
#include "nodes/leaf.hpp"
#include "types.hpp"

namespace bt
{

    inline constexpr struct
    {
        int major = 0;
        int minor = 4;
        int patch = 0;
    } version;

    template <typename Event, typename Context, typename F>
        requires std::invocable<F, const Event &, Context &>
    constexpr auto make_action(F &&f)
    {
        return Action<Event, Context, std::decay_t<F>>(std::forward<F>(f));
    }

    template <typename Event, typename Context, typename Pred, IsNode<Event, Context> Child>
        requires std::predicate<Pred, const Context &>
    constexpr auto make_guard(Pred &&pred, Child &&child)
    {
        return Guard<Event, Context, std::decay_t<Pred>, std::decay_t<Child>>(std::forward<Pred>(pred),
                                                                              std::forward<Child>(child));
    }

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    constexpr auto make_sequence(Children &&...children)
    {
        return Sequence<Event, Context, std::decay_t<Children>...>(std::forward<Children>(children)...);
    }

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    constexpr auto make_selector(Children &&...children)
    {
        return Selector<Event, Context, std::decay_t<Children>...>(std::forward<Children>(children)...);
    }

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    constexpr auto make_parallel(Children &&...children)
    {
        return Parallel<Event, Context, std::decay_t<Children>...>(std::forward<Children>(children)...);
    }

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    constexpr auto make_inverter(Child &&child)
    {
        return Inverter<Event, Context, std::decay_t<Child>>(std::forward<Child>(child));
    }

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    constexpr auto make_retry(int attempts, Child &&child)
    {
        return Retry<Event, Context, std::decay_t<Child>>(attempts, std::forward<Child>(child));
    }

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    constexpr auto make_repeat(int iterations, Child &&child)
    {
        return Repeat<Event, Context, std::decay_t<Child>>(iterations, std::forward<Child>(child));
    }

} // namespace bt
