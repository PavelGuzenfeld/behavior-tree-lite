#pragma once

#include "../types.hpp"
#include <functional>
#include <optional>

namespace bt
{

    // ==========================================
    // INVERTER (NOT)
    // ==========================================

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    struct Inverter : NodeBase
    {
        Child child;

        constexpr explicit Inverter(Child c) : child(std::move(c)) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            switch (self.child.process(e, ctx))
            {
            case Status::Success:
                return Status::Failure;
            case Status::Failure:
                return Status::Success;
            default:
                return Status::Running;
            }
        }

        constexpr void reset() { child.reset(); }
    };

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    Inverter(Child) -> Inverter<Event, Context, Child>;

    // ==========================================
    // RETRY
    // ==========================================
    // Retries child N times on failure.

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    struct Retry : NodeBase
    {
        Child child;
        int max_attempts;
        int attempts = 0;

        constexpr Retry(int n, Child c)
            : child(std::move(c)), max_attempts(n) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            Status s = self.child.process(e, ctx);
            if (s == Status::Failure && ++self.attempts < self.max_attempts)
            {
                self.child.reset();
                return Status::Running;
            }
            if (s == Status::Success)
                self.attempts = 0;
            return s;
        }

        constexpr void reset()
        {
            attempts = 0;
            child.reset();
        }
    };

    // ==========================================
    // REPEAT
    // ==========================================
    // Repeats child N times (or forever if N < 0).

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    struct Repeat : NodeBase
    {
        Child child;
        int max_iterations;
        int completed = 0;

        constexpr Repeat(int iterations, Child c)
            : child(std::move(c)), max_iterations(iterations) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            Status s = self.child.process(e, ctx);

            if (s == Status::Running)
                return Status::Running;

            if (s == Status::Failure)
            {
                self.reset();
                return Status::Failure;
            }

            // Success
            self.child.reset();
            if (self.max_iterations < 0)
                return Status::Running; // Infinite

            if (++self.completed >= self.max_iterations)
            {
                self.completed = 0;
                return Status::Success;
            }
            return Status::Running;
        }

        constexpr void reset()
        {
            completed = 0;
            child.reset();
        }
    };

    // ==========================================
    // SUCCEEDER
    // ==========================================
    // Always returns Success (wraps child result).

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    struct Succeeder : NodeBase
    {
        Child child;

        constexpr explicit Succeeder(Child c) : child(std::move(c)) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            Status s = self.child.process(e, ctx);
            return s == Status::Running ? Status::Running : Status::Success;
        }

        constexpr void reset() { child.reset(); }
    };

    // ==========================================
    // FAILER
    // ==========================================
    // Always returns Failure (wraps child result).

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    struct Failer : NodeBase
    {
        Child child;

        constexpr explicit Failer(Child c) : child(std::move(c)) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            Status s = self.child.process(e, ctx);
            return s == Status::Running ? Status::Running : Status::Failure;
        }

        constexpr void reset() { child.reset(); }
    };

    // ==========================================
    // TIMEOUT
    // ==========================================
    // Fails if child doesn't complete within N ticks.

    template <typename Event, typename Context, IsNode<Event, Context> Child>
    struct Timeout : NodeBase
    {
        Child child;
        int max_ticks;
        int ticks = 0;

        constexpr Timeout(int n, Child c)
            : child(std::move(c)), max_ticks(n) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            if (++self.ticks > self.max_ticks)
            {
                self.reset();
                return Status::Failure;
            }

            Status s = self.child.process(e, ctx);
            if (s != Status::Running)
                self.ticks = 0;
            return s;
        }

        constexpr void reset()
        {
            ticks = 0;
            child.reset();
        }
    };

    // ==========================================
    // GUARD (Conditional Decorator)
    // ==========================================
    // Only processes child if predicate returns true.

    template <typename Event, typename Context, typename Pred, IsNode<Event, Context> Child>
        requires std::predicate<Pred, const Context &>
    struct Guard : NodeBase
    {
        [[no_unique_address]] Pred pred;
        Child child;

        constexpr Guard(Pred p, Child c) : pred(std::move(p)), child(std::move(c)) {}

        constexpr Status process(this auto &&self, const Event &e, Context &ctx)
        {
            if (!self.pred(ctx))
                return Status::Failure;
            return self.child.process(e, ctx);
        }

        constexpr void reset() { child.reset(); }
    };

    template <typename Event, typename Context, typename P, IsNode<Event, Context> C>
    Guard(P, C) -> Guard<Event, Context, P, C>;

} // namespace bt
