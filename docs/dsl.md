# DSL

Include `<behavior_tree_lite/dsl.hpp>` to get the operators.

| Operator | Builds | Runs |
|---|---|---|
| `A && B` | `Sequence` | `A`; if it succeeds, `B`. Fails on the first failure. |
| `A \|\| B` | `Selector` | `A`; if it fails, `B`. Succeeds on the first success. |
| `!A` | `Inverter` | `A`, with Success and Failure swapped. |

C++ precedence applies: `&&` binds tighter than `||`, so `A && B || C` is
`(A && B) || C`. `!` binds tightest.

## Flattening

Chaining the same operator does not nest. `A && B && C` is one
`Sequence<A, B, C>`, not `Sequence<Sequence<A, B>, C>`.

```cpp
auto t1 = (A{} && B{}) && C{};   // Sequence<A, B, C>
auto t2 = A{} && (B{} && C{});   // Sequence<A, B, C>
auto t3 = A{} || B{} || C{};     // Selector<A, B, C>
```

Sequences flatten from either side. Selectors flatten only from the left, so
`A || (B || C)` stays nested. It behaves the same, with one extra level in
`print_tree`.

## How the event and context types are found

Every node in a tree must agree on one event type and one context type. The
operators read them from the left operand in one of three ways.

### Typedefs (recommended)

```cpp
struct MyNode : NodeBase {
    using EventType = Event;
    using ContextType = Context;

    Status process(const Event&, Context&) { return Status::Success; }
    void reset() {}
};
```

### Deduced from `process()`

A plain node with a non-template `process(const E&, C&)` works without the
typedefs:

```cpp
struct SimpleNode {
    Status process(const Event&, Context&) { return Status::Success; }
    void reset() {}
};

auto tree = SimpleNode{} && OtherNode{};
```

This does not work when `process` is a template or uses deducing `this`
(`this auto&&`). Use the typedefs there.

### Library nodes

Composites and decorators carry the types as template parameters, so they
can sit on the left. Nodes are move-only, so move a named node into the
expression:

```cpp
auto retry_scan = make_retry<Event, Context>(3, Scan{});
auto tree = std::move(retry_scan) && Inspect{};
```

## Without the operators

Every composite has a factory that takes the types explicitly. The operators
are a thin layer over these.

```cpp
make_sequence<E, C>(children...)
make_selector<E, C>(children...)
make_parallel<E, C>(children...)
make_inverter<E, C>(child)
make_retry<E, C>(attempts, child)
make_repeat<E, C>(count, child)
make_guard<E, C>(predicate, child)
```

`Parallel`, `Timeout`, `Succeeder` and `Failer` have no operator. Build them
with a factory or a constructor and mix them into an operator expression.
