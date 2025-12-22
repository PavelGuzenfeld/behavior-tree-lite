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
        int state = 0;
        bool flag = false;
    };

    // ==========================================
    // ACTION TESTS
    // ==========================================

    TEST(ActionTest, CallsProcessCallback)
    {
        TestContext ctx;
        TestEvent evt{42};

        bool called = false;
        int received_value = 0;

        Action<TestEvent, TestContext> action(
            [&](const TestEvent &e, TestContext &)
            {
                called = true;
                received_value = e.value;
                return Status::Success;
            });

        auto result = action.process(evt, ctx);

        EXPECT_TRUE(called);
        EXPECT_EQ(received_value, 42);
        EXPECT_EQ(result, Status::Success);
    }

    TEST(ActionTest, CallsResetCallback)
    {
        TestContext ctx;
        TestEvent evt;

        bool reset_called = false;

        Action<TestEvent, TestContext> action(
            [](const TestEvent &, TestContext &)
            { return Status::Success; },
            [&]()
            { reset_called = true; });

        action.reset();

        EXPECT_TRUE(reset_called);
    }

    TEST(ActionTest, ModifiesContext)
    {
        TestContext ctx;
        ctx.state = 0;
        TestEvent evt;

        Action<TestEvent, TestContext> action(
            [](const TestEvent &, TestContext &c)
            {
                c.state = 100;
                return Status::Success;
            });

        action.process(evt, ctx);

        EXPECT_EQ(ctx.state, 100);
    }

    TEST(ActionTest, ReturnsRunning)
    {
        TestContext ctx;
        TestEvent evt;
        int tick_count = 0;

        Action<TestEvent, TestContext> action(
            [&](const TestEvent &, TestContext &)
            {
                tick_count++;
                if (tick_count >= 3)
                    return Status::Success;
                return Status::Running;
            });

        EXPECT_EQ(action.process(evt, ctx), Status::Running);
        EXPECT_EQ(action.process(evt, ctx), Status::Running);
        EXPECT_EQ(action.process(evt, ctx), Status::Success);
    }

    // ==========================================
    // CONDITION TESTS
    // ==========================================

    TEST(ConditionTest, ReturnsTrueAsSuccess)
    {
        TestContext ctx;
        TestEvent evt;

        Condition<TestEvent, TestContext> cond(
            [](const TestEvent &, const TestContext &)
            { return true; });

        auto result = cond.process(evt, ctx);
        EXPECT_EQ(result, Status::Success);
    }

    TEST(ConditionTest, ReturnsFalseAsFailure)
    {
        TestContext ctx;
        TestEvent evt;

        Condition<TestEvent, TestContext> cond(
            [](const TestEvent &, const TestContext &)
            { return false; });

        auto result = cond.process(evt, ctx);
        EXPECT_EQ(result, Status::Failure);
    }

    TEST(ConditionTest, ReadsContext)
    {
        TestContext ctx;
        ctx.flag = true;
        TestEvent evt;

        Condition<TestEvent, TestContext> cond(
            [](const TestEvent &, const TestContext &c)
            { return c.flag; });

        EXPECT_EQ(cond.process(evt, ctx), Status::Success);

        ctx.flag = false;
        EXPECT_EQ(cond.process(evt, ctx), Status::Failure);
    }

    TEST(ConditionTest, ReadsEvent)
    {
        TestContext ctx;
        TestEvent evt{50};

        Condition<TestEvent, TestContext> cond(
            [](const TestEvent &e, const TestContext &)
            { return e.value > 25; });

        EXPECT_EQ(cond.process(evt, ctx), Status::Success);

        evt.value = 10;
        EXPECT_EQ(cond.process(evt, ctx), Status::Failure);
    }

    TEST(ConditionTest, NeverReturnsRunning)
    {
        TestContext ctx;
        TestEvent evt;

        // Condition should never return Running
        Condition<TestEvent, TestContext> cond_true(
            [](const TestEvent &, const TestContext &)
            { return true; });

        Condition<TestEvent, TestContext> cond_false(
            [](const TestEvent &, const TestContext &)
            { return false; });

        EXPECT_NE(cond_true.process(evt, ctx), Status::Running);
        EXPECT_NE(cond_false.process(evt, ctx), Status::Running);
    }

    // ==========================================
    // ALWAYS SUCCESS/FAILURE/RUNNING TESTS
    // ==========================================

    TEST(AlwaysSuccessTest, AlwaysReturnsSuccess)
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysSuccess<TestEvent, TestContext> node;

        for (int i = 0; i < 10; ++i)
        {
            EXPECT_EQ(node.process(evt, ctx), Status::Success);
        }
    }

    TEST(AlwaysFailureTest, AlwaysReturnsFailure)
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysFailure<TestEvent, TestContext> node;

        for (int i = 0; i < 10; ++i)
        {
            EXPECT_EQ(node.process(evt, ctx), Status::Failure);
        }
    }

    TEST(AlwaysRunningTest, AlwaysReturnsRunning)
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysRunning<TestEvent, TestContext> node;

        for (int i = 0; i < 10; ++i)
        {
            EXPECT_EQ(node.process(evt, ctx), Status::Running);
        }
    }

    // ==========================================
    // RESET BEHAVIOR
    // ==========================================

    TEST(LeafTest, ResetIsIdempotent)
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysSuccess<TestEvent, TestContext> success;
        AlwaysFailure<TestEvent, TestContext> failure;
        AlwaysRunning<TestEvent, TestContext> running;

        // Multiple resets should be safe
        for (int i = 0; i < 5; ++i)
        {
            success.reset();
            failure.reset();
            running.reset();
        }

        // Behavior should be unchanged
        EXPECT_EQ(success.process(evt, ctx), Status::Success);
        EXPECT_EQ(failure.process(evt, ctx), Status::Failure);
        EXPECT_EQ(running.process(evt, ctx), Status::Running);
    }

} // anonymous namespace
