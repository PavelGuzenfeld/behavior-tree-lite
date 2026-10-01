#include "behavior_tree_lite/behavior_tree.hpp"
#include <doctest/doctest.h>
#include <string>
#include <vector>

using namespace bt;

namespace
{

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

        explicit RunningNode(int ticks = 2, std::string n = "running") : name(std::move(n)), ticks_to_complete(ticks) {}

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

    TEST_CASE("SequenceTest.AllChildrenSucceed")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, SuccessNode, SuccessNode, SuccessNode> seq(SuccessNode("a"), SuccessNode("b"),
                                                                                    SuccessNode("c"));

        auto result = seq.process(evt, ctx);

        CHECK_EQ(result, Status::Success);
        REQUIRE_EQ(ctx.log.size(), 3u);
        CHECK_EQ(ctx.log[0], "a");
        CHECK_EQ(ctx.log[1], "b");
        CHECK_EQ(ctx.log[2], "c");
    }

    TEST_CASE("SequenceTest.FirstChildFails")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, FailureNode, SuccessNode> seq(FailureNode("fail"), SuccessNode("skip"));

        auto result = seq.process(evt, ctx);

        CHECK_EQ(result, Status::Failure);
        REQUIRE_EQ(ctx.log.size(), 1u);
        CHECK_EQ(ctx.log[0], "fail");
    }

    TEST_CASE("SequenceTest.MiddleChildFails")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, SuccessNode, FailureNode, SuccessNode> seq(
            SuccessNode("a"), FailureNode("fail"), SuccessNode("skip"));

        auto result = seq.process(evt, ctx);

        CHECK_EQ(result, Status::Failure);
        REQUIRE_EQ(ctx.log.size(), 2u);
        CHECK_EQ(ctx.log[0], "a");
        CHECK_EQ(ctx.log[1], "fail");
    }

    TEST_CASE("SequenceTest.RunningChildPausesExecution")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, SuccessNode, RunningNode, SuccessNode> seq(
            SuccessNode("a"), RunningNode(3, "running"), SuccessNode("b"));

        auto r1 = seq.process(evt, ctx);
        CHECK_EQ(r1, Status::Running);

        auto r2 = seq.process(evt, ctx);
        CHECK_EQ(r2, Status::Running);

        ctx.log.clear();
        auto r3 = seq.process(evt, ctx);
        CHECK_EQ(r3, Status::Success);
        CHECK_EQ(ctx.log.back(), "b");
    }

    TEST_CASE("SelectorTest.FirstChildSucceeds")
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, SuccessNode, FailureNode> sel(SuccessNode("first"), FailureNode("skip"));

        auto result = sel.process(evt, ctx);

        CHECK_EQ(result, Status::Success);
        REQUIRE_EQ(ctx.log.size(), 1u);
        CHECK_EQ(ctx.log[0], "first");
    }

    TEST_CASE("SelectorTest.FirstFailsSecondSucceeds")
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, FailureNode, SuccessNode> sel(FailureNode("fail"), SuccessNode("success"));

        auto result = sel.process(evt, ctx);

        CHECK_EQ(result, Status::Success);
        REQUIRE_EQ(ctx.log.size(), 2u);
        CHECK_EQ(ctx.log[0], "fail");
        CHECK_EQ(ctx.log[1], "success");
    }

    TEST_CASE("SelectorTest.AllChildrenFail")
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, FailureNode, FailureNode, FailureNode> sel(FailureNode("a"), FailureNode("b"),
                                                                                    FailureNode("c"));

        auto result = sel.process(evt, ctx);

        CHECK_EQ(result, Status::Failure);
        CHECK_EQ(ctx.log.size(), 3u);
    }

    TEST_CASE("SelectorTest.RunningChildPausesExecution")
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, FailureNode, RunningNode, SuccessNode> sel(
            FailureNode("fail"), RunningNode(2, "running"), SuccessNode("skip"));

        auto r1 = sel.process(evt, ctx);
        CHECK_EQ(r1, Status::Running);

        ctx.log.clear();
        auto r2 = sel.process(evt, ctx);
        CHECK_EQ(r2, Status::Success);
    }

    TEST_CASE("ParallelTest.AllChildrenSucceed")
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, SuccessNode, SuccessNode> par(SuccessNode("a"), SuccessNode("b"));

        auto result = par.process(evt, ctx);

        CHECK_EQ(result, Status::Success);
        CHECK_EQ(ctx.log.size(), 2u);
    }

    TEST_CASE("ParallelTest.OneChildFails")
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, SuccessNode, FailureNode> par(SuccessNode("a"), FailureNode("fail"));

        auto result = par.process(evt, ctx);

        CHECK_EQ(result, Status::Failure);
    }

    TEST_CASE("ParallelTest.MixedRunningAndSuccess")
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, RunningNode, SuccessNode> par(RunningNode(2, "running"),
                                                                       SuccessNode("instant"));

        auto r1 = par.process(evt, ctx);
        CHECK_EQ(r1, Status::Running);

        auto r2 = par.process(evt, ctx);
        CHECK_EQ(r2, Status::Success);
    }

    TEST_CASE("ParallelTest.AllChildrenRunTogether")
    {
        int counter1 = 0, counter2 = 0;
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, CounterNode, CounterNode> par{CounterNode{&counter1}, CounterNode{&counter2}};

        par.process(evt, ctx);

        CHECK_EQ(counter1, 1);
        CHECK_EQ(counter2, 1);
    }

    TEST_CASE("SequenceTest.SingleChildSuccess")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, SuccessNode> seq(SuccessNode("only"));

        auto result = seq.process(evt, ctx);
        CHECK_EQ(result, Status::Success);
        CHECK_EQ(ctx.log.size(), 1u);
    }

    TEST_CASE("SequenceTest.SingleChildFailure")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, FailureNode> seq(FailureNode("only"));

        auto result = seq.process(evt, ctx);
        CHECK_EQ(result, Status::Failure);
    }

    TEST_CASE("SelectorTest.SingleChildSuccess")
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, SuccessNode> sel(SuccessNode("only"));

        auto result = sel.process(evt, ctx);
        CHECK_EQ(result, Status::Success);
    }

    TEST_CASE("SelectorTest.SingleChildFailure")
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, FailureNode> sel(FailureNode("only"));

        auto result = sel.process(evt, ctx);
        CHECK_EQ(result, Status::Failure);
    }

    TEST_CASE("ParallelTest.SingleChildSuccess")
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, SuccessNode> par(SuccessNode("only"));

        auto result = par.process(evt, ctx);
        CHECK_EQ(result, Status::Success);
    }

    TEST_CASE("ParallelTest.SingleChildFailure")
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, FailureNode> par(FailureNode("only"));

        auto result = par.process(evt, ctx);
        CHECK_EQ(result, Status::Failure);
    }

    TEST_CASE("CompositeTest.DeeplyNestedSequenceInSelector")
    {
        TestContext ctx;
        TestEvent evt;

        Selector<TestEvent, TestContext, Sequence<TestEvent, TestContext, FailureNode, SuccessNode>,
                 Sequence<TestEvent, TestContext, SuccessNode, SuccessNode>>
            root(Sequence<TestEvent, TestContext, FailureNode, SuccessNode>(FailureNode("f"), SuccessNode("skip")),
                 Sequence<TestEvent, TestContext, SuccessNode, SuccessNode>(SuccessNode("a"), SuccessNode("b")));

        auto result = root.process(evt, ctx);
        CHECK_EQ(result, Status::Success);
        REQUIRE_EQ(ctx.log.size(), 3u);
        CHECK_EQ(ctx.log[0], "f");
        CHECK_EQ(ctx.log[1], "a");
        CHECK_EQ(ctx.log[2], "b");
    }

    TEST_CASE("CompositeTest.ThreeLevelNesting")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, Selector<TestEvent, TestContext, FailureNode, SuccessNode>, SuccessNode> root(
            Selector<TestEvent, TestContext, FailureNode, SuccessNode>(FailureNode("f"), SuccessNode("inner")),
            SuccessNode("outer"));

        auto result = root.process(evt, ctx);
        CHECK_EQ(result, Status::Success);
        REQUIRE_EQ(ctx.log.size(), 3u);
        CHECK_EQ(ctx.log[0], "f");
        CHECK_EQ(ctx.log[1], "inner");
        CHECK_EQ(ctx.log[2], "outer");
    }

    TEST_CASE("CompositeTest.SequenceReset")
    {
        TestContext ctx;
        TestEvent evt;

        Sequence<TestEvent, TestContext, RunningNode, SuccessNode> seq(RunningNode(3, "running"), SuccessNode("b"));

        seq.process(evt, ctx);
        CHECK_EQ(seq.current_index, 0u);

        seq.reset();
        CHECK_EQ(seq.current_index, 0u);
    }

    TEST_CASE("CompositeTest.ParallelReset")
    {
        TestContext ctx;
        TestEvent evt;

        Parallel<TestEvent, TestContext, RunningNode, SuccessNode> par(RunningNode(3), SuccessNode());

        par.process(evt, ctx);

        par.reset();
        for (bool f : par.finished)
        {
            CHECK_FALSE(f);
        }
    }

} // namespace
