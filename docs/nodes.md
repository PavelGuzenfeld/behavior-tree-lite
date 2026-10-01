# Nodes

A node is anything the tree can call with an event and a context and get a
`Status` back: `Success`, `Failure` or `Running`. Write it as
`process(const Event&, Context&)` or as `operator()(const Event&, Context&)`.
That is all the `IsNode<T, Event, Context>` concept asks for. If a node has
both, `process` is used.

Library nodes derive from `NodeBase`, which forwards `operator()` to
`process`, so `tree(event, ctx)` works as well as `tree.process(event, ctx)`.

A node may also have `void reset()`, which puts it back to its first-tick
state. Parents call it after a child finishes or when they are reset
themselves. A node without `reset()` is skipped, so stateless leaves need
none.

## Composites

| Node | Each tick | Returns |
|---|---|---|
| `Sequence` | Resumes at the child that was Running, else starts at the first. Stops at the first child that does not succeed. | Failure on the first failure, Running if a child is running, Success when all succeed. |
| `Selector` | Same resume rule. Stops at the first child that does not fail. | Success on the first success, Running if a child is running, Failure when all fail. |
| `Parallel` | Ticks every child that has not succeeded yet. | Failure as soon as any child fails, and resets every child. Success when all have succeeded. Otherwise Running. |

`Sequence` and `Selector` remember which child was Running. On the next
tick they continue there and skip the children before it.

## Decorators

| Node | Behavior |
|---|---|
| `Inverter` | Swaps Success and Failure. Running passes through. |
| `Retry(n, child)` | Runs the child up to `n` times in total. A failure before the last attempt resets the child and returns Running. Success resets the count. |
| `Repeat(n, child)` | Runs the child to success `n` times, returning Running in between, then Success. A failure resets and returns Failure. With `n < 0` it repeats forever and never returns Success. |
| `Timeout(n, child)` | Counts ticks while the child runs. On tick `n + 1` it resets the child and returns Failure without ticking it. |
| `Succeeder` | Returns Success whatever the child returns, except Running. |
| `Failer` | Returns Failure whatever the child returns, except Running. |
| `Guard(pred, child)` | Calls `pred(ctx)` first. If it is false, returns Failure without ticking the child. |

```cpp
auto retry   = make_retry<Event, Context>(3, Scan{});
auto repeat  = make_repeat<Event, Context>(5, Patrol{});
auto timeout = Timeout<Event, Context, Scan>(10, Scan{});
auto safe    = Succeeder<Event, Context, Risky>(Risky{});
auto fail    = Failer<Event, Context, Check>(Check{});
auto guard   = make_guard<Event, Context>(
    [](const Context& ctx) { return ctx.battery > 50; }, Inspect{});
```

## Leaves

### Your own structs

Most leaves are your own structs with `process()`, as in
[Getting started](getting-started.md).

### `Action`

`Action` wraps a callable taking `(const Event&, Context&)`. What it returns
decides the status:

- `Status`: returned as is
- `bool`: `true` is Success, `false` is Failure
- `void`: always Success

```cpp
auto charge = leaf([](const Event&, Context& c) { c.battery = 100; });
auto at_home = leaf([](const Event&, Context& c) { return c.at_dock; });

auto go_home = leaf([](const Event&, Context& c) { c.at_dock = true; });

auto tree = (std::move(at_home) && std::move(charge)) || std::move(go_home);
```

Nodes are move-only, so a named leaf is moved into the tree. `leaf` reads the event and context types from the lambda's parameters, so the
parameters must be spelled out: `leaf` does not take generic lambdas. For
those, or when you want the types stated, use
`make_action<Event, Context>(f)`. Both return an `Action`.

### `Condition`

`Condition` wraps a predicate `(const Event&, const Context&) -> bool`. It
never returns Running.

### `StatefulAction`

`StatefulAction` carries a value that `reset()` restores. The callable gets
it as its first argument: `(State&, const Event&, Context&) -> Status`.

### `AlwaysSuccess`, `AlwaysFailure`, `AlwaysRunning`

Stateless fixed answers. Useful as placeholders and in tests.

### `DynamicAction`

`DynamicAction` holds two `std::function`s, one for process and one for reset.
Use it when the leaf is only known at runtime. It is the only node that can
allocate, and the only one with a measurable per-tick cost (see
[Performance](performance.md)).

```cpp
DynamicAction<Event, Context> action(
    [](const Event&, Context&) { return Status::Success; },
    [] {});
```
