#include "behavior_tree_lite/behavior_tree.hpp"
#include <gtest/gtest.h>

using namespace bt;

namespace
{ // Anonymous namespace to avoid ODR violations

    // ==========================================
    // TEST FIXTURES
    // ==========================================

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

    // ==========================================
    // INVERTER TESTS
    // ==========================================

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

    // ==========================================
    // RETRY TESTS
    // ==========================================

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

        // First attempt fails, returns Running
        auto r1 = retry.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);
        EXPECT_EQ(retry.attempts, 1);

        // Second attempt fails, returns Running
        auto r2 = retry.process(evt, ctx);
        EXPECT_EQ(r2, Status::Running);
        EXPECT_EQ(retry.attempts, 2);

        // Third attempt fails, gives up
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

    // ==========================================
    // REPEAT TESTS
    // ==========================================

    TEST(RepeatTest, RepeatsNTimes)
    {
        TestContext ctx;
        TestEvent evt;
        int counter = 0;

        Repeat<TestEvent, TestContext, CountingNode> repeat(3, CountingNode(&counter, 1));

        // First iteration
        auto r1 = repeat.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);
        EXPECT_EQ(counter, 1);

        // Second iteration
        auto r2 = repeat.process(evt, ctx);
        EXPECT_EQ(r2, Status::Running);
        EXPECT_EQ(counter, 2);

        // Third iteration - completes
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

    // ==========================================
    // SUCCEEDER TESTS
    // ==========================================

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

    // ==========================================
    // FAILER TESTS
    // ==========================================

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

    // ==========================================
    // TIMEOUT TESTS
    // ==========================================

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

        timeout.process(evt, ctx); // tick 1
        timeout.process(evt, ctx); // tick 2
        timeout.process(evt, ctx); // tick 3

        auto result = timeout.process(evt, ctx); // tick 4 - timeout!
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

} // anonymous namespace