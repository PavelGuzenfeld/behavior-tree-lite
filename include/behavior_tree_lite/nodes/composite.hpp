#pragma once

#include "behavior_tree_lite/types.hpp"

#include <array>
#include <tuple>
#include <utility>

namespace bt
{

    // ==========================================
    // SEQUENCE (AND)
    // ==========================================
    // Runs children in order. Fails if one fails. Succeeds if ALL succeed.

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    struct Sequence : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        std::tuple<Children...> children;
        std::size_t current_index = 0;

        constexpr explicit Sequence(Children... c) : children(std::move(c)...) {}

        constexpr Status process(this auto &&self, const Event &evt, Context &ctx)
        {
            return [&]<std::size_t... Is>(std::index_sequence<Is...>)
            {
                Status result = Status::Success;

                ((Is >= self.current_index && result == Status::Success ? [&]
                      {
                auto& child = std::get<Is>(self.children);
                Status s = child.process(evt, ctx);

                if (s == Status::Running) {
                    self.current_index = Is;
                    result = Status::Running;
                } else if (s == Status::Failure) {
                    self.current_index = 0;
                    child.reset();
                    result = Status::Failure;
                } else {
                    child.reset();
                } }()                                             : void()),
                 ...);

                if (result == Status::Success)
                    self.current_index = 0;
                return result;
            }(std::make_index_sequence<sizeof...(Children)>{});
        }

        constexpr void reset()
        {
            current_index = 0;
            std::apply([](auto &...c)
                       { (c.reset(), ...); }, children);
        }
    };

    // Deduction guide
    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    Sequence(Children...) -> Sequence<Event, Context, Children...>;

    // ==========================================
    // SELECTOR (OR / Fallback)
    // ==========================================
    // Runs children in order. Succeeds if one succeeds. Fails if ALL fail.

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    struct Selector : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        std::tuple<Children...> children;
        std::size_t current_index = 0;

        constexpr explicit Selector(Children... c) : children(std::move(c)...) {}

        constexpr Status process(this auto &&self, const Event &evt, Context &ctx)
        {
            return [&]<std::size_t... Is>(std::index_sequence<Is...>)
            {
                Status result = Status::Failure;

                ((Is >= self.current_index && result == Status::Failure ? [&]
                      {
                auto& child = std::get<Is>(self.children);
                Status s = child.process(evt, ctx);

                if (s == Status::Running) {
                    self.current_index = Is;
                    result = Status::Running;
                } else if (s == Status::Success) {
                    self.current_index = 0;
                    child.reset();
                    result = Status::Success;
                } else {
                    child.reset();
                } }()                                             : void()),
                 ...);

                if (result == Status::Failure)
                    self.current_index = 0;
                return result;
            }(std::make_index_sequence<sizeof...(Children)>{});
        }

        constexpr void reset()
        {
            current_index = 0;
            std::apply([](auto &...c)
                       { (c.reset(), ...); }, children);
        }
    };

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    Selector(Children...) -> Selector<Event, Context, Children...>;

    // ==========================================
    // PARALLEL (Concurrent)
    // ==========================================
    // Runs ALL children every tick. Succeeds if ALL succeed. Fails if ANY fail.

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    struct Parallel : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        std::tuple<Children...> children;
        std::array<bool, sizeof...(Children)> finished{};

        constexpr explicit Parallel(Children... c) : children(std::move(c)...) {}

        constexpr Status process(this auto &&self, const Event &evt, Context &ctx)
        {
            return [&]<std::size_t... Is>(std::index_sequence<Is...>)
            {
                std::size_t successes = 0;
                bool failed = false;

                (([&]
                  {
                if (self.finished[Is]) { ++successes; return; }

                auto& child = std::get<Is>(self.children);
                switch (child.process(evt, ctx)) {
                    case Status::Success:
                        self.finished[Is] = true;
                        child.reset();
                        ++successes;
                        break;
                    case Status::Failure:
                        failed = true;
                        child.reset();
                        break;
                    case Status::Running:
                        break;
                } }()),
                 ...);

                if (failed)
                {
                    self.reset();
                    return Status::Failure;
                }
                if (successes == sizeof...(Children))
                {
                    self.reset();
                    return Status::Success;
                }
                return Status::Running;
            }(std::make_index_sequence<sizeof...(Children)>{});
        }

        constexpr void reset()
        {
            finished.fill(false);
            std::apply([](auto &...c)
                       { (c.reset(), ...); }, children);
        }
    };

    template <typename Event, typename Context, IsNode<Event, Context>... Children>
    Parallel(Children...) -> Parallel<Event, Context, Children...>;

} // namespace bt