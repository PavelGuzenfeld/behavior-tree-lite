# Performance

The benchmark is `test/bench_tree.cpp`. Build it with
`-DBUILD_BENCHMARKS=ON`. CI runs it on every push.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_BENCHMARKS=ON
cmake --build build
./build/behavior_tree_lite_bench
```

## What it shows

Template nodes inline away. Their numbers below are under one clock cycle,
so the benchmark can't tell them apart from an empty loop. The compiler sees
the whole tree and keeps little more than the loop. Read those rows as "no
dispatch cost left to measure", not as a dispatch time.

`DynamicAction` is the exception. Its `std::function` call is a real
indirect call, and it shows up.

| Benchmark | ns/op |
|---|---:|
| Single Action node | 0.2 |
| DynamicAction (std::function) | 1.6 |
| AlwaysSuccess (stateless) | 0.2 |
| Sequence (3 children) | 0.1 |
| Selector (2 children) | 0.4 |
| Parallel (3 children) | 0.1 |
| Inverter(Action) | 0.1 |
| Guard(predicate, Action) | 0.4 |
| (Check && Action) \|\| Fallback | 1.0 |
| 5-level nested tree | 0.9 |
| Sequence with Running child (resume) | 0.5 |
| std::visit variant dispatch | 0.1 |

Measured 2026-10-01: GCC 14.2, `-O3`, Intel i7-12700H, 1M iterations.

## What it doesn't show

In your program the leaves do real work and the tree is ticked from a timer
or an event queue. The tree's own overhead will be lost in that. To compare
with another library, benchmark your own tree in your own loop.
