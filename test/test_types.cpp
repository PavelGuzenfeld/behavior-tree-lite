#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "behavior_tree_lite/types.hpp"
#include <doctest/doctest.h>
#include <string>

using namespace bt;

namespace
{

    TEST_CASE("TypesTest.StatusToString")
    {
        CHECK_EQ(to_string(Status::Success), "Success");
        CHECK_EQ(to_string(Status::Failure), "Failure");
        CHECK_EQ(to_string(Status::Running), "Running");
    }

    TEST_CASE("TypesTest.StatusEnum")
    {
        Status s = Status::Success;
        CHECK_NE(s, Status::Failure);
        CHECK_NE(s, Status::Running);
    }

    TEST_CASE("TypesTest.OverloadedVisitor")
    {
        std::variant<int, double, std::string> v = 42;

        auto result = std::visit(overloaded{[](int i) { return std::string("int: ") + std::to_string(i); },
                                            [](double d) { return std::string("double: ") + std::to_string(d); },
                                            [](const std::string &s)
                                            {
                                                return std::string("string: ") + s;
                                            }},
                                 v);

        CHECK_EQ(result, "int: 42");
    }

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

    struct NodeWithoutReset : NodeBase
    {
        Status process(const MockEvent &, MockContext &) { return Status::Success; }
    };

    TEST_CASE("TypesTest.IsNodeConcept")
    {
        static_assert(IsNode<ValidNode, MockEvent, MockContext>);
        static_assert(!IsNode<InvalidNodeNoProcess, MockEvent, MockContext>);
        static_assert(IsNode<NodeWithoutReset, MockEvent, MockContext>);
        static_assert(!HasReset<NodeWithoutReset>);
        static_assert(HasReset<ValidNode>);
    }

    TEST_CASE("TypesTest.NodeBaseMovable")
    {
        static_assert(std::is_move_constructible_v<NodeBase>);
        static_assert(std::is_move_assignable_v<NodeBase>);
    }

    TEST_CASE("TypesTest.NodeBaseNotCopyable")
    {
        static_assert(!std::is_copy_constructible_v<NodeBase>);
        static_assert(!std::is_copy_assignable_v<NodeBase>);
    }

} // namespace
