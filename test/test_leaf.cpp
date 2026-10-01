#include "behavior_tree_lite/behavior_tree.hpp"
#include <doctest/doctest.h>
#include <functional>

using namespace bt;

namespace
{

    struct TestEvent
    {
        int value = 0;
    };

    struct TestContext
    {
        int state = 0;
        bool flag = false;
    };

    template <typename E, typename C> struct DynamicCondition : NodeBase
    {
        std::function<bool(const E &, const C &)> predicate;

        explicit DynamicCondition(std::function<bool(const E &, const C &)> pred) : predicate(std::move(pred)) {}

        Status process(const E &e, C &ctx) { return predicate(e, ctx) ? Status::Success : Status::Failure; }
        void reset() {}
    };

    TEST_CASE("ActionTest.CallsProcessCallback")
    {
        TestContext ctx;
        TestEvent evt{42};

        bool called = false;
        int received_value = 0;

        DynamicAction<TestEvent, TestContext> action(
            [&](const TestEvent &e, TestContext &)
            {
                called = true;
                received_value = e.value;
                return Status::Success;
            });

        auto result = action.process(evt, ctx);

        CHECK(called);
        CHECK_EQ(received_value, 42);
        CHECK_EQ(result, Status::Success);
    }

    TEST_CASE("ActionTest.CallsResetCallback")
    {
        TestContext ctx;
        TestEvent evt;

        bool reset_called = false;

        DynamicAction<TestEvent, TestContext> action([](const TestEvent &, TestContext &) { return Status::Success; },
                                                     [&]() { reset_called = true; });

        action.reset();

        CHECK(reset_called);
    }

    TEST_CASE("ActionTest.ModifiesContext")
    {
        TestContext ctx;
        ctx.state = 0;
        TestEvent evt;

        DynamicAction<TestEvent, TestContext> action(
            [](const TestEvent &, TestContext &c)
            {
                c.state = 100;
                return Status::Success;
            });

        action.process(evt, ctx);

        CHECK_EQ(ctx.state, 100);
    }

    TEST_CASE("ActionTest.ReturnsRunning")
    {
        TestContext ctx;
        TestEvent evt;
        int tick_count = 0;

        DynamicAction<TestEvent, TestContext> action(
            [&](const TestEvent &, TestContext &)
            {
                tick_count++;
                if (tick_count >= 3)
                    return Status::Success;
                return Status::Running;
            });

        CHECK_EQ(action.process(evt, ctx), Status::Running);
        CHECK_EQ(action.process(evt, ctx), Status::Running);
        CHECK_EQ(action.process(evt, ctx), Status::Success);
    }

    TEST_CASE("ConditionTest.ReturnsTrueAsSuccess")
    {
        TestContext ctx;
        TestEvent evt;

        DynamicCondition<TestEvent, TestContext> cond([](const TestEvent &, const TestContext &) { return true; });

        auto result = cond.process(evt, ctx);
        CHECK_EQ(result, Status::Success);
    }

    TEST_CASE("ConditionTest.ReturnsFalseAsFailure")
    {
        TestContext ctx;
        TestEvent evt;

        DynamicCondition<TestEvent, TestContext> cond([](const TestEvent &, const TestContext &) { return false; });

        auto result = cond.process(evt, ctx);
        CHECK_EQ(result, Status::Failure);
    }

    TEST_CASE("ConditionTest.ReadsContext")
    {
        TestContext ctx;
        ctx.flag = true;
        TestEvent evt;

        DynamicCondition<TestEvent, TestContext> cond([](const TestEvent &, const TestContext &c) { return c.flag; });

        CHECK_EQ(cond.process(evt, ctx), Status::Success);

        ctx.flag = false;
        CHECK_EQ(cond.process(evt, ctx), Status::Failure);
    }

    TEST_CASE("ConditionTest.ReadsEvent")
    {
        TestContext ctx;
        TestEvent evt{50};

        DynamicCondition<TestEvent, TestContext> cond([](const TestEvent &e, const TestContext &)
                                                      { return e.value > 25; });

        CHECK_EQ(cond.process(evt, ctx), Status::Success);

        evt.value = 10;
        CHECK_EQ(cond.process(evt, ctx), Status::Failure);
    }

    TEST_CASE("ConditionTest.NeverReturnsRunning")
    {
        TestContext ctx;
        TestEvent evt;

        DynamicCondition<TestEvent, TestContext> cond_true([](const TestEvent &, const TestContext &) { return true; });

        DynamicCondition<TestEvent, TestContext> cond_false([](const TestEvent &, const TestContext &)
                                                            { return false; });

        CHECK_NE(cond_true.process(evt, ctx), Status::Running);
        CHECK_NE(cond_false.process(evt, ctx), Status::Running);
    }

    TEST_CASE("AlwaysSuccessTest.AlwaysReturnsSuccess")
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysSuccess<TestEvent, TestContext> node;

        for (int i = 0; i < 10; ++i)
        {
            CHECK_EQ(node.process(evt, ctx), Status::Success);
        }
    }

    TEST_CASE("AlwaysFailureTest.AlwaysReturnsFailure")
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysFailure<TestEvent, TestContext> node;

        for (int i = 0; i < 10; ++i)
        {
            CHECK_EQ(node.process(evt, ctx), Status::Failure);
        }
    }

    TEST_CASE("AlwaysRunningTest.AlwaysReturnsRunning")
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysRunning<TestEvent, TestContext> node;

        for (int i = 0; i < 10; ++i)
        {
            CHECK_EQ(node.process(evt, ctx), Status::Running);
        }
    }

    TEST_CASE("LeafTest.ResetIsIdempotent")
    {
        TestContext ctx;
        TestEvent evt;

        AlwaysSuccess<TestEvent, TestContext> success;
        AlwaysFailure<TestEvent, TestContext> failure;
        AlwaysRunning<TestEvent, TestContext> running;

        for (int i = 0; i < 5; ++i)
        {
            success.reset();
            failure.reset();
            running.reset();
        }

        CHECK_EQ(success.process(evt, ctx), Status::Success);
        CHECK_EQ(failure.process(evt, ctx), Status::Failure);
        CHECK_EQ(running.process(evt, ctx), Status::Running);
    }

} // namespace