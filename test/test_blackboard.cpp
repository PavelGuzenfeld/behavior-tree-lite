#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/blackboard.hpp"
#include "behavior_tree_lite/dsl.hpp"
#include <gtest/gtest.h>
#include <string>

namespace blackboard_test
{

    struct Battery : bt::Key<int>
    {
    };
    struct Altitude : bt::Key<int>
    {
    };
    struct Callsign : bt::Key<std::string>
    {
    };
    struct Unregistered : bt::Key<int>
    {
    };

    using Board = bt::Blackboard<Battery, Altitude, Callsign>;

    template <typename B, typename K>
    concept CanGet = requires(B b) { b.template get<K>(); };

    template <typename B, typename K>
    concept CanSet = requires(B b) { b.template set<K>(1); };

    TEST(BlackboardTest, ValuesStartValueInitialised)
    {
        Board board;

        EXPECT_EQ(board.get<Battery>(), 0);
        EXPECT_EQ(board.get<Callsign>(), "");
    }

    TEST(BlackboardTest, SetThenGetReturnsTheStoredValue)
    {
        Board board;

        board.set<Battery>(80);
        board.set<Callsign>("falcon");

        EXPECT_EQ(board.get<Battery>(), 80);
        EXPECT_EQ(board.get<Callsign>(), "falcon");
    }

    TEST(BlackboardTest, KeysOfTheSameValueTypeDoNotAlias)
    {
        Board board;

        board.set<Battery>(80);
        board.set<Altitude>(120);

        EXPECT_EQ(board.get<Battery>(), 80);
        EXPECT_EQ(board.get<Altitude>(), 120);
    }

    TEST(BlackboardTest, GetReturnsAReferenceThatWritesThrough)
    {
        Board board;

        board.get<Battery>() += 5;
        board.get<Callsign>().append("x");

        EXPECT_EQ(board.get<Battery>(), 5);
        EXPECT_EQ(board.get<Callsign>(), "x");
    }

    TEST(BlackboardTest, ConstBoardIsReadable)
    {
        Board board;
        board.set<Altitude>(7);
        const Board &view = board;

        EXPECT_EQ(view.get<Altitude>(), 7);
    }

    TEST(BlackboardTest, KeyNotOnTheBoardIsRejectedAtCompileTime)
    {
        static_assert(CanGet<Board, Battery>);
        static_assert(CanSet<Board, Battery>);
        static_assert(!CanGet<Board, Unregistered>);
        static_assert(!CanSet<Board, Unregistered>);
    }

    struct Tick
    {
    };

    struct DrainBattery : bt::NodeBase
    {
        using EventType = Tick;
        using ContextType = Board;
        bt::Status process(const Tick &, Board &board)
        {
            board.get<Battery>() -= 30;
            return bt::Status::Success;
        }
    };

    struct BatteryOk : bt::NodeBase
    {
        using EventType = Tick;
        using ContextType = Board;
        bt::Status process(const Tick &, Board &board)
        {
            return board.get<Battery>() > 20 ? bt::Status::Success : bt::Status::Failure;
        }
    };

    TEST(BlackboardTest, ServesAsTheTreeContext)
    {
        Board board;
        board.set<Battery>(100);
        auto tree = BatteryOk{} && DrainBattery{};

        EXPECT_EQ(tree.process(Tick{}, board), bt::Status::Success);
        EXPECT_EQ(tree.process(Tick{}, board), bt::Status::Success);
        EXPECT_EQ(board.get<Battery>(), 40);
        EXPECT_EQ(tree.process(Tick{}, board), bt::Status::Success);
        EXPECT_EQ(tree.process(Tick{}, board), bt::Status::Failure);
        EXPECT_EQ(board.get<Battery>(), 10);
    }

} // namespace blackboard_test
