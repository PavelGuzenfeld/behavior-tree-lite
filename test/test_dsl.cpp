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

    // Test Sequence Operator (&&)
    TEST(DSLTest, SequenceOperator)
    {
        auto seq = NodeA{} && NodeB{};

        // Check type deduction
        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB>>);

        Context ctx;
        EXPECT_EQ(seq.process(Event{}, ctx), Status::Failure);
    }

    // Test Selector Operator (||)
    TEST(DSLTest, SelectorOperator)
    {
        auto sel = NodeB{} || NodeA{};

        // Check type deduction
        static_assert(std::is_same_v<decltype(sel), Selector<Event, Context, NodeB, NodeA>>);

        Context ctx;
        EXPECT_EQ(sel.process(Event{}, ctx), Status::Success);
    }

    // Test Inverter Operator (!)
    TEST(DSLTest, InverterOperator)
    {
        auto inv = !NodeA{}; // NodeA returns Success

        // Check type deduction
        static_assert(std::is_same_v<decltype(inv), Inverter<Event, Context, NodeA>>);

        Context ctx;
        EXPECT_EQ(inv.process(Event{}, ctx), Status::Failure);
    }

    // Test Flattening (Sequence && Node)
    TEST(DSLTest, SequenceFlattening)
    {
        // (A && B) && C should become Sequence<A, B, C>, not Sequence<Sequence<A, B>, C>
        auto seq = (NodeA{} && NodeB{}) && NodeC{};

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB, NodeC>>);
    }

    // Test Flattening (Node && Sequence)
    TEST(DSLTest, SequenceFlatteningRight)
    {
        auto seq = NodeA{} && (NodeB{} && NodeC{});

        static_assert(std::is_same_v<decltype(seq), Sequence<Event, Context, NodeA, NodeB, NodeC>>);
    }

    // Test Complex Composition
    TEST(DSLTest, ComplexComposition)
    {
        // Tree: (A >> B) | (!C)
        auto tree = (NodeA{} && NodeB{}) || (!NodeC{});

        using ExpectedTree = Selector<Event, Context,
                                      Sequence<Event, Context, NodeA, NodeB>,
                                      Inverter<Event, Context, NodeC>>;

        static_assert(std::is_same_v<decltype(tree), ExpectedTree>);
    }

    // Test Deduction from Process Method (User Node without typedefs)
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

} // namespace