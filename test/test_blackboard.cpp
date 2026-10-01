#include "behavior_tree_lite/behavior_tree.hpp"
#include "behavior_tree_lite/blackboard.hpp"
#include "behavior_tree_lite/dsl.hpp"
#include <doctest/doctest.h>
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

    TEST_CASE("BlackboardTest.ValuesStartValueInitialised")
    {
        Board board;

        CHECK_EQ(board.get<Battery>(), 0);
        CHECK_EQ(board.get<Callsign>(), "");
    }

    TEST_CASE("BlackboardTest.SetThenGetReturnsTheStoredValue")
    {
        Board board;

        board.set<Battery>(80);
        board.set<Callsign>("falcon");

        CHECK_EQ(board.get<Battery>(), 80);
        CHECK_EQ(board.get<Callsign>(), "falcon");
    }

    TEST_CASE("BlackboardTest.KeysOfTheSameValueTypeDoNotAlias")
    {
        Board board;

        board.set<Battery>(80);
        board.set<Altitude>(120);

        CHECK_EQ(board.get<Battery>(), 80);
        CHECK_EQ(board.get<Altitude>(), 120);
    }

    TEST_CASE("BlackboardTest.GetReturnsAReferenceThatWritesThrough")
    {
        Board board;

        board.get<Battery>() += 5;
        board.get<Callsign>().append("x");

        CHECK_EQ(board.get<Battery>(), 5);
        CHECK_EQ(board.get<Callsign>(), "x");
    }

    TEST_CASE("BlackboardTest.ConstBoardIsReadable")
    {
        Board board;
        board.set<Altitude>(7);
        const Board &view = board;

        CHECK_EQ(view.get<Altitude>(), 7);
    }

    TEST_CASE("BlackboardTest.KeyNotOnTheBoardIsRejectedAtCompileTime")
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

    TEST_CASE("BlackboardTest.ServesAsTheTreeContext")
    {
        Board board;
        board.set<Battery>(100);
        auto tree = BatteryOk{} && DrainBattery{};

        CHECK_EQ(tree.process(Tick{}, board), bt::Status::Success);
        CHECK_EQ(tree.process(Tick{}, board), bt::Status::Success);
        CHECK_EQ(board.get<Battery>(), 40);
        CHECK_EQ(tree.process(Tick{}, board), bt::Status::Success);
        CHECK_EQ(tree.process(Tick{}, board), bt::Status::Failure);
        CHECK_EQ(board.get<Battery>(), 10);
    }

} // namespace blackboard_test
