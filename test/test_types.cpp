#include "behavior_tree_lite/types.hpp"
#include "gtest/gtest.h"

using namespace bt;

namespace
{ // Anonymous namespace to avoid ODR violations

    // ==========================================
    // STATUS TESTS
    // ==========================================

    TEST(TypesTest, StatusToString)
    {
        EXPECT_EQ(to_string(Status::Success), "Success");
        EXPECT_EQ(to_string(Status::Failure), "Failure");
        EXPECT_EQ(to_string(Status::Running), "Running");
    }

    TEST(TypesTest, StatusEnum)
    {
        Status s = Status::Success;
        EXPECT_NE(s, Status::Failure);
        EXPECT_NE(s, Status::Running);
    }

    // ==========================================
    // OVERLOADED VISITOR TESTS
    // ==========================================

    TEST(TypesTest, OverloadedVisitor)
    {
        std::variant<int, double, std::string> v = 42;

        auto result = std::visit(overloaded{[](int i)
                                            { return std::string("int: ") + std::to_string(i); },
                                            [](double d)
                                            { return std::string("double: ") + std::to_string(d); },
                                            [](const std::string &s)
                                            { return std::string("string: ") + s; }},
                                 v);

        EXPECT_EQ(result, "int: 42");
    }

    // ==========================================
    // CONCEPT TESTS
    // ==========================================

    struct MockEvent
    {
    };
    struct MockContext
    {
    };

    struct ValidNode : NodeBase
    {
        Status process(const MockEvent &, MockContext &) { return Status::Success; }
        void reset() {}
    };

    struct InvalidNodeNoProcess : NodeBase
    {
        void reset() {}
    };

    struct InvalidNodeNoReset : NodeBase
    {
        Status process(const MockEvent &, MockContext &) { return Status::Success; }
    };

    TEST(TypesTest, IsNodeConcept)
    {
        static_assert(IsNode<ValidNode, MockEvent, MockContext>);
        static_assert(!IsNode<InvalidNodeNoProcess, MockEvent, MockContext>);
        static_assert(!IsNode<InvalidNodeNoReset, MockEvent, MockContext>);
    }

    // ==========================================
    // NODEBASE TESTS
    // ==========================================

    TEST(TypesTest, NodeBaseMovable)
    {
        static_assert(std::is_move_constructible_v<NodeBase>);
        static_assert(std::is_move_assignable_v<NodeBase>);
    }

    TEST(TypesTest, NodeBaseNotCopyable)
    {
        static_assert(!std::is_copy_constructible_v<NodeBase>);
        static_assert(!std::is_copy_assignable_v<NodeBase>);
    }

} // anonymous namespace
