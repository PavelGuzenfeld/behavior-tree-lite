#pragma once

#include "../types.hpp"
#include <concepts>
#include <functional>
#include <type_traits>

namespace bt
{

    // ==========================================
    // ACTION (Template-based, zero overhead)
    // ==========================================
    // Wraps a callable as a leaf node. Supports:
    // - Status(Event, Context) -> direct return
    // - bool(Event, Context)   -> true=Success, false=Failure
    // - void(Event, Context)   -> always Success

    template <typename Event, typename Context, typename F>
        requires std::invocable<F, const Event &, Context &>
    struct Action : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        [[no_unique_address]] F func;

        constexpr explicit Action(F f) : func(std::move(f)) {}

        constexpr Status process(const Event &e, Context &ctx)
        {
            using R = std::invoke_result_t<F, const Event &, Context &>;
            if constexpr (std::same_as<R, Status>)
            {
                return func(e, ctx);
            }
            else if constexpr (std::same_as<R, bool>)
            {
                return func(e, ctx) ? Status::Success : Status::Failure;
            }
            else
            {
                func(e, ctx);
                return Status::Success;
            }
        }

        constexpr void reset() {}
    };

    template <typename E, typename C, typename F>
    Action(F) -> Action<E, C, F>;

    // ==========================================
    // STATEFUL ACTION
    // ==========================================
    // Action with internal state that resets.

    template <typename Event, typename Context, typename State, typename F>
    struct StatefulAction : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        State state;
        State initial;
        [[no_unique_address]] F func;

        constexpr StatefulAction(State init, F f)
            : state(init), initial(init), func(std::move(f)) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            return self.func(self.state, e, ctx);
        }

        constexpr void reset()
        {
            if constexpr (std::is_copy_assignable_v<State>)
            {
                state = initial;
            }
        }
    };

    template <typename E, typename C, typename S, typename F>
    StatefulAction(S, F) -> StatefulAction<E, C, S, F>;

    // ==========================================
    // CONDITION
    // ==========================================
    // Returns Success if predicate true, Failure otherwise. Never Running.

    template <typename Event, typename Context, typename Pred>
        requires std::predicate<Pred, const Event &, const Context &>
    struct Condition : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        [[no_unique_address]] Pred predicate;

        constexpr explicit Condition(Pred pred) : predicate(std::move(pred)) {}

        constexpr Status process(const Event &e, Context &ctx)
        {
            return predicate(e, ctx) ? Status::Success : Status::Failure;
        }

        constexpr void reset() {}
    };

    template <typename E, typename C, typename P>
    Condition(P) -> Condition<E, C, P>;

    // ==========================================
    // ALWAYS SUCCESS
    // ==========================================

    template <typename Event, typename Context>
    struct AlwaysSuccess : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        constexpr Status process([[maybe_unused]] const Event &, [[maybe_unused]] Context &)
        {
            return Status::Success;
        }
        constexpr void reset() {}
    };

    // ==========================================
    // ALWAYS FAILURE
    // ==========================================

    template <typename Event, typename Context>
    struct AlwaysFailure : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        constexpr Status process([[maybe_unused]] const Event &, [[maybe_unused]] Context &)
        {
            return Status::Failure;
        }
        constexpr void reset() {}
    };

    // ==========================================
    // ALWAYS RUNNING
    // ==========================================

    template <typename Event, typename Context>
    struct AlwaysRunning : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        constexpr Status process([[maybe_unused]] const Event &, [[maybe_unused]] Context &)
        {
            return Status::Running;
        }
        constexpr void reset() {}
    };

    // ==========================================
    // TYPE-ERASED ACTION (std::function fallback)
    // ==========================================
    // For runtime polymorphism when needed.

    template <typename Event, typename Context>
    struct DynamicAction : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;

        using ProcessFn = std::function<Status(const Event &, Context &)>;
        using ResetFn = std::function<void()>;

        ProcessFn on_process;
        ResetFn on_reset;

        explicit DynamicAction(ProcessFn process_fn, ResetFn reset_fn = [] {})
            : on_process(std::move(process_fn)), on_reset(std::move(reset_fn)) {}

        Status process(const Event &e, Context &ctx) { return on_process(e, ctx); }
        void reset() { on_reset(); }
    };

} // namespace bt