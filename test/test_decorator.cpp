#include "behavior_tree_lite/behavior_tree.hpp"
#include <gtest/gtest.h>

using namespace bt;

namespace
{

    struct TestEvent
    {
    };
    struct TestContext
    {
    };

    struct SuccessNode : NodeBase
    {
        Status process(const TestEvent &, TestContext &) { return Status::Success; }
        void reset() {}
    };

    struct FailureNode : NodeBase
    {
        Status process(const TestEvent &, TestContext &) { return Status::Failure; }
        void reset() {}
    };

    struct RunningNode : NodeBase
    {
        Status process(const TestEvent &, TestContext &) { return Status::Running; }
        void reset() {}
    };

    struct CountingNode : NodeBase
    {
        int *counter;
        int ticks_to_success;
        int current_tick = 0;

        CountingNode(int *c, int ticks) : counter(c), ticks_to_success(ticks) {}

        Status process(const TestEvent &, TestContext &)
        {
            (*counter)++;
            current_tick++;
            if (current_tick >= ticks_to_success)
            {
                return Status::Success;
            }
            return Status::Running;
        }

        void reset() { current_tick = 0; }
    };

    struct FailAfterNode : NodeBase
    {
        int fail_after;
        int current = 0;

        explicit FailAfterNode(int n) : fail_after(n) {}

        Status process(const TestEvent &, TestContext &)
        {
            current++;
            if (current >= fail_after)
            {
                return Status::Failure;
            }
            return Status::Running;
        }

        void reset() { current = 0; }
    };

    TEST(InverterTest, InvertsSuccess)
    {
        TestContext ctx;
        TestEvent evt;

        Inverter<TestEvent, TestContext, SuccessNode> inv(SuccessNode{});

        auto result = inv.process(evt, ctx);
        EXPECT_EQ(result, Status::Failure);
    }

    TEST(InverterTest, InvertsFailure)
    {
        TestContext ctx;
        TestEvent evt;

        Inverter<TestEvent, TestContext, FailureNode> inv(FailureNode{});

        auto result = inv.process(evt, ctx);
        EXPECT_EQ(result, Status::Success);
    }

    TEST(InverterTest, PassesThroughRunning)
    {
        TestContext ctx;
        TestEvent evt;

        Inverter<TestEvent, TestContext, RunningNode> inv(RunningNode{});

        auto result = inv.process(evt, ctx);
        EXPECT_EQ(result, Status::Running);
    }

    TEST(RetryTest, SucceedsImmediately)
    {
        TestContext ctx;
        TestEvent evt;

        Retry<TestEvent, TestContext, SuccessNode> retry(3, SuccessNode{});

        auto result = retry.process(evt, ctx);
        EXPECT_EQ(result, Status::Success);
        EXPECT_EQ(retry.attempts, 0);
    }

    TEST(RetryTest, RetriesOnFailure)
    {
        TestContext ctx;
        TestEvent evt;

        Retry<TestEvent, TestContext, FailureNode> retry(3, FailureNode{});

        auto r1 = retry.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);
        EXPECT_EQ(retry.attempts, 1);

        auto r2 = retry.process(evt, ctx);
        EXPECT_EQ(r2, Status::Running);
        EXPECT_EQ(retry.attempts, 2);

        auto r3 = retry.process(evt, ctx);
        EXPECT_EQ(r3, Status::Failure);
        EXPECT_EQ(retry.attempts, 3);
    }

    TEST(RetryTest, ResetClearsAttempts)
    {
        TestContext ctx;
        TestEvent evt;

        Retry<TestEvent, TestContext, FailureNode> retry(3, FailureNode{});

        retry.process(evt, ctx);
        EXPECT_EQ(retry.attempts, 1);

        retry.reset();
        EXPECT_EQ(retry.attempts, 0);
    }

    TEST(RepeatTest, RepeatsNTimes)
    {
        TestContext ctx;
        TestEvent evt;
        int counter = 0;

        Repeat<TestEvent, TestContext, CountingNode> repeat(3, CountingNode(&counter, 1));

        auto r1 = repeat.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);
        EXPECT_EQ(counter, 1);

        auto r2 = repeat.process(evt, ctx);
        EXPECT_EQ(r2, Status::Running);
        EXPECT_EQ(counter, 2);

        auto r3 = repeat.process(evt, ctx);
        EXPECT_EQ(r3, Status::Success);
        EXPECT_EQ(counter, 3);
    }

    TEST(RepeatTest, FailsIfChildFails)
    {
        TestContext ctx;
        TestEvent evt;

        Repeat<TestEvent, TestContext, FailureNode> repeat(5, FailureNode{});

        auto result = repeat.process(evt, ctx);
        EXPECT_EQ(result, Status::Failure);
    }

    TEST(RepeatTest, InfiniteRepeat)
    {
        TestContext ctx;
        TestEvent evt;
        int counter = 0;

        Repeat<TestEvent, TestContext, CountingNode> repeat(-1, CountingNode(&counter, 1));

        for (int i = 0; i < 100; ++i)
        {
            auto result = repeat.process(evt, ctx);
            EXPECT_EQ(result, Status::Running);
        }
        EXPECT_EQ(counter, 100);
    }

    TEST(SucceederTest, ConvertsFailureToSuccess)
    {
        TestContext ctx;
        TestEvent evt;

        Succeeder<TestEvent, TestContext, FailureNode> succ(FailureNode{});

        auto result = succ.process(evt, ctx);
        EXPECT_EQ(result, Status::Success);
    }

    TEST(SucceederTest, PassesThroughRunning)
    {
        TestContext ctx;
        TestEvent evt;

        Succeeder<TestEvent, TestContext, RunningNode> succ(RunningNode{});

        auto result = succ.process(evt, ctx);
        EXPECT_EQ(result, Status::Running);
    }

    TEST(FailerTest, ConvertsSuccessToFailure)
    {
        TestContext ctx;
        TestEvent evt;

        Failer<TestEvent, TestContext, SuccessNode> failer(SuccessNode{});

        auto result = failer.process(evt, ctx);
        EXPECT_EQ(result, Status::Failure);
    }

    TEST(FailerTest, PassesThroughRunning)
    {
        TestContext ctx;
        TestEvent evt;

        Failer<TestEvent, TestContext, RunningNode> failer(RunningNode{});

        auto result = failer.process(evt, ctx);
        EXPECT_EQ(result, Status::Running);
    }

    TEST(TimeoutTest, SucceedsWithinTimeout)
    {
        TestContext ctx;
        TestEvent evt;
        int counter = 0;

        Timeout<TestEvent, TestContext, CountingNode> timeout(5, CountingNode(&counter, 2));

        auto r1 = timeout.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);

        auto r2 = timeout.process(evt, ctx);
        EXPECT_EQ(r2, Status::Success);
    }

    TEST(TimeoutTest, FailsOnTimeout)
    {
        TestContext ctx;
        TestEvent evt;

        Timeout<TestEvent, TestContext, RunningNode> timeout(3, RunningNode{});

        timeout.process(evt, ctx);
        timeout.process(evt, ctx);
        timeout.process(evt, ctx);

        auto result = timeout.process(evt, ctx);
        EXPECT_EQ(result, Status::Failure);
    }

    TEST(TimeoutTest, ResetClearsTicks)
    {
        TestContext ctx;
        TestEvent evt;

        Timeout<TestEvent, TestContext, RunningNode> timeout(5, RunningNode{});

        timeout.process(evt, ctx);
        timeout.process(evt, ctx);
        EXPECT_EQ(timeout.ticks, 2);

        timeout.reset();
        EXPECT_EQ(timeout.ticks, 0);
    }

    TEST(GuardTest, ProcessesChildWhenPredicateTrue)
    {
        TestContext ctx;
        TestEvent evt;

        auto always_true = [](const TestContext &)
        {
            return true;
        };
        Guard<TestEvent, TestContext, decltype(always_true), SuccessNode> guard(always_true, SuccessNode{});

        auto result = guard.process(evt, ctx);
        EXPECT_EQ(result, Status::Success);
    }

    TEST(GuardTest, ReturnsFailureWhenPredicateFalse)
    {
        TestContext ctx;
        TestEvent evt;

        auto always_false = [](const TestContext &)
        {
            return false;
        };
        Guard<TestEvent, TestContext, decltype(always_false), SuccessNode> guard(always_false, SuccessNode{});

        auto result = guard.process(evt, ctx);
        EXPECT_EQ(result, Status::Failure);
    }

    TEST(GuardTest, ChildNotProcessedWhenPredicateFalse)
    {
        TestContext ctx;
        TestEvent evt;
        int counter = 0;

        auto pred = [](const TestContext &)
        {
            return false;
        };
        Guard<TestEvent, TestContext, decltype(pred), CountingNode> guard(pred, CountingNode(&counter, 1));

        guard.process(evt, ctx);
        EXPECT_EQ(counter, 0);
    }

    TEST(GuardTest, PredicateReadsContext)
    {
        struct ContextWithFlag
        {
        };
        bool flag = true;

        TestContext ctx;
        TestEvent evt;

        auto pred = [&flag](const TestContext &)
        {
            return flag;
        };
        Guard<TestEvent, TestContext, decltype(pred), SuccessNode> guard(pred, SuccessNode{});

        EXPECT_EQ(guard.process(evt, ctx), Status::Success);

        flag = false;
        EXPECT_EQ(guard.process(evt, ctx), Status::Failure);
    }

    TEST(GuardTest, PassesThroughChildRunning)
    {
        TestContext ctx;
        TestEvent evt;

        auto always_true = [](const TestContext &)
        {
            return true;
        };
        Guard<TestEvent, TestContext, decltype(always_true), RunningNode> guard(always_true, RunningNode{});

        auto result = guard.process(evt, ctx);
        EXPECT_EQ(result, Status::Running);
    }

    TEST(GuardTest, ResetResetsChild)
    {
        TestContext ctx;
        TestEvent evt;
        int counter = 0;

        auto pred = [](const TestContext &)
        {
            return true;
        };
        Guard<TestEvent, TestContext, decltype(pred), CountingNode> guard(pred, CountingNode(&counter, 3));

        guard.process(evt, ctx);
        guard.process(evt, ctx);
        EXPECT_EQ(counter, 2);

        guard.reset();

        auto result = guard.process(evt, ctx);
        EXPECT_EQ(result, Status::Running);
    }

    TEST(GuardTest, MakeGuardFactory)
    {
        TestContext ctx;
        TestEvent evt;

        auto guard = make_guard<TestEvent, TestContext>([](const TestContext &) { return true; }, SuccessNode{});

        EXPECT_EQ(guard.process(evt, ctx), Status::Success);
    }

} // namespace