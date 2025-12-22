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
        int value = 0;
    };

    struct TestContext
    {
        int counter = 0;
        std::vector<std::string> log;
    };

    struct SuccessNode : NodeBase
    {
        std::string name;
        explicit SuccessNode(std::string n = "success") : name(std::move(n)) {}

        Status process(const TestEvent &, TestContext &ctx)
        {
            ctx.log.push_back(name);
            return Status::Success;
        }
        void reset() {}
    };

    struct FailureNode : NodeBase
    {
        std::string name;
        explicit FailureNode(std::string n = "failure") : name(std::move(n)) {}

        Status process(const TestEvent &, TestContext &ctx)
        {
            ctx.log.push_back(name);
            return Status::Failure;
        }
        void reset() {}
    };

    struct RunningNode : NodeBase
    {
        std::string name;
        int ticks_to_complete;
        int current_tick = 0;

        explicit RunningNode(int ticks = 2, std::string n = "running")
            : name(std::move(n)), ticks_to_complete(ticks) {}

        Status process(const TestEvent &, TestContext &ctx)
        {
            ctx.log.push_back(name);
            current_tick++;
            if (current_tick >= ticks_to_complete)
            {
                current_tick = 0;
                return Status::Success;
            }
            return Status::Running;
        }

        void reset() { current_tick = 0; }
    };

    struct CounterNode : NodeBase
    {
        int *counter;
        explicit CounterNode(int *c) : counter(c) {}

        Status process(const TestEvent &, TestContext &)
        {
            (*counter)++;
            return Status::Success;
        }
        void reset() {}
    };

    // ==========================================
    // SEQUENCE TESTS
    // ==========================================

    TEST(SequenceTest, AllChildrenSucceed)
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, SuccessNode, SuccessNode, SuccessNode> seq(
            SuccessNode("a"), SuccessNode("b"), SuccessNode("c"));

        auto result = seq.process(evt, ctx);

        EXPECT_EQ(result, Status::Success);
        ASSERT_EQ(ctx.log.size(), 3u);
        EXPECT_EQ(ctx.log[0], "a");
        EXPECT_EQ(ctx.log[1], "b");
        EXPECT_EQ(ctx.log[2], "c");
    }

    TEST(SequenceTest, FirstChildFails)
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, FailureNode, SuccessNode> seq(
            FailureNode("fail"), SuccessNode("skip"));

        auto result = seq.process(evt, ctx);

        EXPECT_EQ(result, Status::Failure);
        ASSERT_EQ(ctx.log.size(), 1u);
        EXPECT_EQ(ctx.log[0], "fail");
    }

    TEST(SequenceTest, MiddleChildFails)
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, SuccessNode, FailureNode, SuccessNode> seq(
            SuccessNode("a"), FailureNode("fail"), SuccessNode("skip"));

        auto result = seq.process(evt, ctx);

        EXPECT_EQ(result, Status::Failure);
        ASSERT_EQ(ctx.log.size(), 2u);
        EXPECT_EQ(ctx.log[0], "a");
        EXPECT_EQ(ctx.log[1], "fail");
    }

    TEST(SequenceTest, RunningChildPausesExecution)
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, SuccessNode, RunningNode, SuccessNode> seq(
            SuccessNode("a"), RunningNode(3, "running"), SuccessNode("b"));

        // First tick: a succeeds, running returns Running
        auto r1 = seq.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);

        // Second tick: running still Running
        auto r2 = seq.process(evt, ctx);
        EXPECT_EQ(r2, Status::Running);

        // Third tick: running succeeds, b succeeds
        ctx.log.clear();
        auto r3 = seq.process(evt, ctx);
        EXPECT_EQ(r3, Status::Success);
        EXPECT_EQ(ctx.log.back(), "b");
    }

    // ==========================================
    // SELECTOR TESTS
    // ==========================================

    TEST(SelectorTest, FirstChildSucceeds)
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, SuccessNode, FailureNode> sel(
            SuccessNode("first"), FailureNode("skip"));

        auto result = sel.process(evt, ctx);

        EXPECT_EQ(result, Status::Success);
        ASSERT_EQ(ctx.log.size(), 1u);
        EXPECT_EQ(ctx.log[0], "first");
    }

    TEST(SelectorTest, FirstFailsSecondSucceeds)
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, FailureNode, SuccessNode> sel(
            FailureNode("fail"), SuccessNode("success"));

        auto result = sel.process(evt, ctx);

        EXPECT_EQ(result, Status::Success);
        ASSERT_EQ(ctx.log.size(), 2u);
        EXPECT_EQ(ctx.log[0], "fail");
        EXPECT_EQ(ctx.log[1], "success");
    }

    TEST(SelectorTest, AllChildrenFail)
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, FailureNode, FailureNode, FailureNode> sel(
            FailureNode("a"), FailureNode("b"), FailureNode("c"));

        auto result = sel.process(evt, ctx);

        EXPECT_EQ(result, Status::Failure);
        EXPECT_EQ(ctx.log.size(), 3u);
    }

    TEST(SelectorTest, RunningChildPausesExecution)
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, FailureNode, RunningNode, SuccessNode> sel(
            FailureNode("fail"), RunningNode(2, "running"), SuccessNode("skip"));

        // First tick
        auto r1 = sel.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);

        // Second tick: running completes with success
        ctx.log.clear();
        auto r2 = sel.process(evt, ctx);
        EXPECT_EQ(r2, Status::Success);
    }

    // ==========================================
    // PARALLEL TESTS
    // ==========================================

    TEST(ParallelTest, AllChildrenSucceed)
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, SuccessNode, SuccessNode> par(
            SuccessNode("a"), SuccessNode("b"));

        auto result = par.process(evt, ctx);

        EXPECT_EQ(result, Status::Success);
        EXPECT_EQ(ctx.log.size(), 2u);
    }

    TEST(ParallelTest, OneChildFails)
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, SuccessNode, FailureNode> par(
            SuccessNode("a"), FailureNode("fail"));

        auto result = par.process(evt, ctx);

        EXPECT_EQ(result, Status::Failure);
    }

    TEST(ParallelTest, MixedRunningAndSuccess)
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, RunningNode, SuccessNode> par(
            RunningNode(2, "running"), SuccessNode("instant"));

        // First tick: instant succeeds, running still going
        auto r1 = par.process(evt, ctx);
        EXPECT_EQ(r1, Status::Running);

        // Second tick: running completes
        auto r2 = par.process(evt, ctx);
        EXPECT_EQ(r2, Status::Success);
    }

    TEST(ParallelTest, AllChildrenRunTogether)
    {
        int counter1 = 0, counter2 = 0;
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, CounterNode, CounterNode> par{
            CounterNode{&counter1}, CounterNode{&counter2}};

        par.process(evt, ctx);

        EXPECT_EQ(counter1, 1);
        EXPECT_EQ(counter2, 1);
    }

    // ==========================================
    // RESET TESTS
    // ==========================================

    TEST(CompositeTest, SequenceReset)
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, RunningNode, SuccessNode> seq(
            RunningNode(3, "running"), SuccessNode("b"));

        seq.process(evt, ctx);
        EXPECT_EQ(seq.current_index, 0u);

        seq.reset();
        EXPECT_EQ(seq.current_index, 0u);
    }

    TEST(CompositeTest, ParallelReset)
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, RunningNode, SuccessNode> par(
            RunningNode(3), SuccessNode());

        par.process(evt, ctx);

        par.reset();
        for (bool f : par.finished)
        {
            EXPECT_FALSE(f);
        }
    }

} // anonymous namespace
