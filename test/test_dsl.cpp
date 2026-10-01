#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/debug.hpp"
#include "behavior_tree_lite/dsl.hpp"
#include <doctest/doctest.h>
#include <sstream>
#include <string>

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

    TEST_CASE("DSLTest.SequenceOperator")
    {
        auto seq = NodeA{} && NodeB{};

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB>>);

        Context ctx;
        CHECK_EQ(seq.process(Event{}, ctx), Status::Failure);
    }

    TEST_CASE("DSLTest.SelectorOperator")
    {
        auto sel = NodeB{} || NodeA{};

        static_assert(std::is_same_v<decltype(sel), Selector<Event, Context, NodeB, NodeA>>);

        Context ctx;
        CHECK_EQ(sel.process(Event{}, ctx), Status::Success);
    }

    TEST_CASE("DSLTest.InverterOperator")
    {
        auto inv = !NodeA{};

        static_assert(std::is_same_v<decltype(inv), Inverter<Event, Context, NodeA>>);

        Context ctx;
        CHECK_EQ(inv.process(Event{}, ctx), Status::Failure);
    }

    TEST_CASE("DSLTest.SequenceFlattening")
    {
        auto seq = (NodeA{} && NodeB{}) && NodeC{};

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB, NodeC>>);
    }

    TEST_CASE("DSLTest.SequenceFlatteningRight")
    {
        auto seq = NodeA{} && (NodeB{} && NodeC{});

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB, NodeC>>);
    }

    TEST_CASE("DSLTest.ComplexComposition")
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

    TEST_CASE("DSLTest.DeduceFromProcess")
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

    TEST_CASE("OptionalResetTest.NodeWithoutResetIsANode")
    {
        static_assert(IsNode<NoResetNode, Event, Context>);
    }

    TEST_CASE("OptionalResetTest.NodeWithoutResetComposesUnderEveryParent")
    {
        Context ctx;
        auto seq = NoResetNode{} && NodeA{};
        auto sel = NoResetNode{} || NodeA{};
        auto inv = !NoResetNode{};
        auto par = make_parallel<Event, Context>(NoResetNode{}, NodeA{});
        auto retry = make_retry<Event, Context>(2, NoResetNode{});

        CHECK_EQ(seq.process(Event{}, ctx), Status::Failure);
        CHECK_EQ(sel.process(Event{}, ctx), Status::Success);
        CHECK_EQ(inv.process(Event{}, ctx), Status::Success);
        CHECK_EQ(par.process(Event{}, ctx), Status::Failure);
        CHECK_EQ(retry.process(Event{}, ctx), Status::Running);
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

    TEST_CASE("CallableNodeTest.OperatorCallIsANode")
    {
        static_assert(IsNode<CallableWithTypedefs, Event, Context>);
        static_assert(IsNode<CallableDeduced, Event, Context>);
    }

    TEST_CASE("CallableNodeTest.CallableNodesComposeAndRunUnderEveryParent")
    {
        Context ctx;
        auto seq = CallableDeduced{} && CallableWithTypedefs{};
        auto sel = CallableWithTypedefs{} || CallableDeduced{};
        auto inv = !CallableWithTypedefs{};
        auto retry = make_retry<Event, Context>(2, CallableWithTypedefs{});

        CHECK_EQ(seq.process(Event{}, ctx), Status::Failure);
        CHECK_EQ(sel.process(Event{}, ctx), Status::Success);
        CHECK_EQ(inv.process(Event{}, ctx), Status::Success);
        CHECK_EQ(retry.process(Event{}, ctx), Status::Running);
    }

    TEST_CASE("CallableNodeTest.LibraryNodesAreCallableToo")
    {
        Context ctx;
        auto seq = NodeA{} && NodeB{};
        auto inv = !NodeA{};

        CHECK_EQ(seq(Event{}, ctx), Status::Failure);
        CHECK_EQ(inv(Event{}, ctx), Status::Failure);
        CHECK_EQ(NodeA{}(Event{}, ctx), Status::Success);
    }

    TEST_CASE("CallableNodeTest.ProcessWinsWhenBothExist")
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
        CHECK_EQ(seq.process(Event{}, ctx), Status::Success);
    }

    TEST_CASE("LambdaLeafTest.LeafDeducesTypesAndMapsReturnValuesToStatus")
    {
        Context ctx;
        int calls = 0;

        auto from_void = leaf([&](const Event &, Context &) { ++calls; });
        auto from_true = leaf([](const Event &, Context &) { return true; });
        auto from_false = leaf([](const Event &, Context &) { return false; });
        auto from_status = leaf([](const Event &, Context &) { return Status::Running; });

        CHECK_EQ(from_void.process(Event{}, ctx), Status::Success);
        CHECK_EQ(calls, 1);
        CHECK_EQ(from_true.process(Event{}, ctx), Status::Success);
        CHECK_EQ(from_false.process(Event{}, ctx), Status::Failure);
        CHECK_EQ(from_status.process(Event{}, ctx), Status::Running);
    }

    TEST_CASE("LambdaLeafTest.LambdaLeavesComposeWithTheOperators")
    {
        Context ctx;
        int visited = 0;

        auto tree = (leaf([&](const Event &, Context &) { ++visited; }) &&
                     leaf([](const Event &, Context &) { return false; })) ||
                    leaf([&](const Event &, Context &) { visited += 10; });

        static_assert(std::is_same_v<EventOf<decltype(tree)>, Event>);
        CHECK_EQ(tree.process(Event{}, ctx), Status::Success);
        CHECK_EQ(visited, 11);
    }

    TEST_CASE("LambdaLeafTest.MakeActionTakesExplicitTypes")
    {
        Context ctx;
        auto action = make_action<Event, Context>([](const auto &, auto &) { return false; });

        CHECK_EQ(action.process(Event{}, ctx), Status::Failure);
    }

} // namespace
namespace dot_test
{

    struct Event
    {
    };
    struct Context
    {
    };

    struct Leaf1 : bt::NodeBase
    {
        using EventType = Event;
        using ContextType = Context;
        bt::Status process(const Event &, Context &) { return bt::Status::Success; }
    };
    struct Leaf2 : Leaf1
    {
    };
    struct Leaf3 : Leaf1
    {
    };

    TEST_CASE("ToDotTest.EmitsOneNodePerTreeNodeAndAnEdgeFromEachParent")
    {
        auto tree = (Leaf1{} && Leaf2{}) || Leaf3{};
        std::ostringstream out;

        bt::to_dot(tree, out);

        CHECK_EQ(out.str(), "digraph behavior_tree {\n"
                            "  n0 [label=\"Selector\"];\n"
                            "  n1 [label=\"Sequence\"];\n"
                            "  n0 -> n1;\n"
                            "  n2 [label=\"dot_test::Leaf1\"];\n"
                            "  n1 -> n2;\n"
                            "  n3 [label=\"dot_test::Leaf2\"];\n"
                            "  n1 -> n3;\n"
                            "  n4 [label=\"dot_test::Leaf3\"];\n"
                            "  n0 -> n4;\n"
                            "}\n");
    }

    TEST_CASE("ToDotTest.DecoratorLabelsKeepTheirParameter")
    {
        auto tree = bt::make_retry<Event, Context>(3, Leaf1{});
        std::ostringstream out;

        bt::to_dot(tree, out);

        CHECK_NE(out.str().find("[label=\"Retry (3x)\"]"), std::string::npos);
        CHECK_NE(out.str().find("n0 -> n1;"), std::string::npos);
    }

} // namespace dot_test
