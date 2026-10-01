#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/dsl.hpp"
#include <gtest/gtest.h>

using namespace bt;

namespace
{

    struct Event
    {
    };
    struct Context
    {
    };

    struct NodeA : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;
        Status process(const Event &, Context &) { return Status::Success; }
        void reset() {}
    };

    struct NodeB : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;
        Status process(const Event &, Context &) { return Status::Failure; }
        void reset() {}
    };

    struct NodeC : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;
        Status process(const Event &, Context &) { return Status::Running; }
        void reset() {}
    };

    TEST(DSLTest, SequenceOperator)
    {
        auto seq = NodeA{} && NodeB{};

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB>>);

        Context ctx;
        EXPECT_EQ(seq.process(Event{}, ctx), Status::Failure);
    }

    TEST(DSLTest, SelectorOperator)
    {
        auto sel = NodeB{} || NodeA{};

        static_assert(std::is_same_v<decltype(sel), Selector<Event, Context, NodeB, NodeA>>);

        Context ctx;
        EXPECT_EQ(sel.process(Event{}, ctx), Status::Success);
    }

    TEST(DSLTest, InverterOperator)
    {
        auto inv = !NodeA{};

        static_assert(std::is_same_v<decltype(inv), Inverter<Event, Context, NodeA>>);

        Context ctx;
        EXPECT_EQ(inv.process(Event{}, ctx), Status::Failure);
    }

    TEST(DSLTest, SequenceFlattening)
    {
        auto seq = (NodeA{} && NodeB{}) && NodeC{};

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB, NodeC>>);
    }

    TEST(DSLTest, SequenceFlatteningRight)
    {
        auto seq = NodeA{} && (NodeB{} && NodeC{});

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB, NodeC>>);
    }

    TEST(DSLTest, ComplexComposition)
    {
        auto tree = (NodeA{} && NodeB{}) || (!NodeC{});

        using ExpectedTree =
            Selector<Event, Context, Sequence<Event, Context, NodeA, NodeB>, Inverter<Event, Context, NodeC>>;

        static_assert(std::is_same_v<decltype(tree), ExpectedTree>);
    }

    struct SimpleNode
    {
        Status process(const Event &, Context &) { return Status::Success; }
        void reset() {}
    };

    TEST(DSLTest, DeduceFromProcess)
    {
        auto seq = SimpleNode{} && NodeA{};

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, SimpleNode, NodeA>>);
    }

    struct NoResetNode : NodeBase
    {
        using EventType = Event;
        using ContextType = Context;
        Status process(const Event &, Context &) { return Status::Failure; }
    };

    TEST(OptionalResetTest, NodeWithoutResetIsANode)
    {
        static_assert(IsNode<NoResetNode, Event, Context>);
    }

    TEST(OptionalResetTest, NodeWithoutResetComposesUnderEveryParent)
    {
        Context ctx;
        auto seq = NoResetNode{} && NodeA{};
        auto sel = NoResetNode{} || NodeA{};
        auto inv = !NoResetNode{};
        auto par = make_parallel<Event, Context>(NoResetNode{}, NodeA{});
        auto retry = make_retry<Event, Context>(2, NoResetNode{});

        EXPECT_EQ(seq.process(Event{}, ctx), Status::Failure);
        EXPECT_EQ(sel.process(Event{}, ctx), Status::Success);
        EXPECT_EQ(inv.process(Event{}, ctx), Status::Success);
        EXPECT_EQ(par.process(Event{}, ctx), Status::Failure);
        EXPECT_EQ(retry.process(Event{}, ctx), Status::Running);
        seq.reset();
        retry.reset();
    }

    struct CallableWithTypedefs
    {
        using EventType = Event;
        using ContextType = Context;
        Status operator()(const Event &, Context &) { return Status::Failure; }
    };

    struct CallableDeduced
    {
        Status operator()(const Event &, Context &) const { return Status::Success; }
    };

    TEST(CallableNodeTest, OperatorCallIsANode)
    {
        static_assert(IsNode<CallableWithTypedefs, Event, Context>);
        static_assert(IsNode<CallableDeduced, Event, Context>);
    }

    TEST(CallableNodeTest, CallableNodesComposeAndRunUnderEveryParent)
    {
        Context ctx;
        auto seq = CallableDeduced{} && CallableWithTypedefs{};
        auto sel = CallableWithTypedefs{} || CallableDeduced{};
        auto inv = !CallableWithTypedefs{};
        auto retry = make_retry<Event, Context>(2, CallableWithTypedefs{});

        EXPECT_EQ(seq.process(Event{}, ctx), Status::Failure);
        EXPECT_EQ(sel.process(Event{}, ctx), Status::Success);
        EXPECT_EQ(inv.process(Event{}, ctx), Status::Success);
        EXPECT_EQ(retry.process(Event{}, ctx), Status::Running);
    }

    TEST(CallableNodeTest, LibraryNodesAreCallableToo)
    {
        Context ctx;
        auto seq = NodeA{} && NodeB{};
        auto inv = !NodeA{};

        EXPECT_EQ(seq(Event{}, ctx), Status::Failure);
        EXPECT_EQ(inv(Event{}, ctx), Status::Failure);
        EXPECT_EQ(NodeA{}(Event{}, ctx), Status::Success);
    }

    TEST(CallableNodeTest, ProcessWinsWhenBothExist)
    {
        struct Both : NodeBase
        {
            using EventType = Event;
            using ContextType = Context;
            Status process(const Event &, Context &) { return Status::Success; }
            Status operator()(const Event &, Context &) { return Status::Failure; }
        };
        Context ctx;
        auto seq = Both{} && NodeA{};
        EXPECT_EQ(seq.process(Event{}, ctx), Status::Success);
    }

    TEST(LambdaLeafTest, LeafDeducesTypesAndMapsReturnValuesToStatus)
    {
        Context ctx;
        int calls = 0;

        auto from_void = leaf([&](const Event &, Context &) { ++calls; });
        auto from_true = leaf([](const Event &, Context &) { return true; });
        auto from_false = leaf([](const Event &, Context &) { return false; });
        auto from_status = leaf([](const Event &, Context &) { return Status::Running; });

        EXPECT_EQ(from_void.process(Event{}, ctx), Status::Success);
        EXPECT_EQ(calls, 1);
        EXPECT_EQ(from_true.process(Event{}, ctx), Status::Success);
        EXPECT_EQ(from_false.process(Event{}, ctx), Status::Failure);
        EXPECT_EQ(from_status.process(Event{}, ctx), Status::Running);
    }

    TEST(LambdaLeafTest, LambdaLeavesComposeWithTheOperators)
    {
        Context ctx;
        int visited = 0;

        auto tree = (leaf([&](const Event &, Context &) { ++visited; }) &&
                     leaf([](const Event &, Context &) { return false; })) ||
                    leaf([&](const Event &, Context &) { visited += 10; });

        static_assert(std::is_same_v<EventOf<decltype(tree)>, Event>);
        EXPECT_EQ(tree.process(Event{}, ctx), Status::Success);
        EXPECT_EQ(visited, 11);
    }

    TEST(LambdaLeafTest, MakeActionTakesExplicitTypes)
    {
        Context ctx;
        auto action = make_action<Event, Context>([](const auto &, auto &) { return false; });

        EXPECT_EQ(action.process(Event{}, ctx), Status::Failure);
    }

} // namespace