# Blackboard

A context can be any struct. When you want shared values without writing one,
`bt::Blackboard` is a typed bag with one tag per value. A key's type is the
only name it has, so a wrong key or a wrong value type is a compile error,
and nothing is looked up at runtime or allocated.

```cpp
#include <behavior_tree_lite/blackboard.hpp>

struct Battery : bt::Key<int> {};
struct Callsign : bt::Key<std::string> {};

using Board = bt::Blackboard<Battery, Callsign>;

Board board;
board.set<Battery>(80);
board.get<Callsign>() = "falcon";
int level = board.get<Battery>();
```

Values start value-initialised. `get` returns a reference, so it writes
through. A key that is not in the board's list does not compile.

Use the board as the tree's context:

```cpp
struct BatteryOk : bt::NodeBase {
    using EventType = Tick;
    using ContextType = Board;

    bt::Status process(const Tick&, Board& board) {
        return board.get<Battery>() > 20 ? bt::Status::Success : bt::Status::Failure;
    }
};
```

Two keys may hold the same value type; they do not share storage. Each key
may appear once in a board.

## Why tags and not string keys

`ctx.get<int>("battery")` needs a runtime map from strings to values, which
means allocation and a lookup per access, and a typo fails when the robot
runs. Tags move all of that to compile time. The price is that the set of
keys is fixed when you write the board type.
