#include "behavior_tree_lite/behavior_tree.hpp"
#include <gtest/gtest.h>
#include <variant>

using namespace bt;

namespace
{ // Anonymous namespace to avoid ODR violations

    // ==========================================
    // VARIANT EVENT SYSTEM (like original example)
    // ==========================================

    struct TickEvent
    {
        double dt = 0.1;
    };
    struct BatteryEvent
    {
        int voltage;
    };
    struct EnemyEvent
    {
        int dist;
        int id;
    };

    using Event = std::variant<TickEvent, BatteryEvent, EnemyEvent>;

    struct RobotContext
    {
        int battery = 100;
        bool alarm_active = false;
        int enemies_detected = 0;
        int shots_fired = 0;
    };

    // ==========================================
    // DOMAIN-SPECIFIC NODES
    // ==========================================

    struct CheckBattery : NodeBase
    {
        int threshold;
        explicit CheckBattery(int t = 20) : threshold(t) {}

        Status process(const Event &e, RobotContext &ctx)
        {
            std::visit(overloaded{[&](const BatteryEvent &b)
                                  { ctx.battery = b.voltage; },
                                  [](const auto &) {}},
                       e);
            return (ctx.battery >= threshold) ? Status::Success : Status::Failure;
        }
        void reset() {}
    };

    struct ScanForEnemy : NodeBase
    {
        Status process(const Event &e, RobotContext &ctx)
        {
            return std::visit(overloaded{[&](const EnemyEvent &)
                                         {
                                             ctx.enemies_detected++;
                                             return Status::Success;
                                         },
                                         [](const TickEvent &)
                                         { return Status::Running; },
                                         [](const auto &)
                                         { return Status::Running; }},
                              e);
        }
        void reset() {}
    };

    struct FireWeapon : NodeBase
    {
        int shots_needed;
        int shots = 0;

        explicit FireWeapon(int s = 3) : shots_needed(s) {}

        Status process(const Event &e, RobotContext &ctx)
        {
            if (std::holds_alternative<TickEvent>(e))
            {
                shots++;
                ctx.shots_fired++;
                if (shots >= shots_needed)
                    return Status::Success;
            }
            return Status::Running;
        }
        void reset() { shots = 0; }
    };

    struct ActivateAlarm : NodeBase
    {
        Status process(const Event &, RobotContext &ctx)
        {
            ctx.alarm_active = true;
            return Status::Success;
        }
        void reset() {}
    };

    // ==========================================
    // INTEGRATION TESTS
    // ==========================================

    TEST(IntegrationTest, VariantEventDispatch)
    {
        RobotContext ctx;
        ctx.battery = 50;

        CheckBattery check(20);

        EXPECT_EQ(check.process(TickEvent{}, ctx), Status::Success);
        EXPECT_EQ(check.process(BatteryEvent{10}, ctx), Status::Failure);
        EXPECT_EQ(ctx.battery, 10);
        EXPECT_EQ(check.process(BatteryEvent{100}, ctx), Status::Success);
        EXPECT_EQ(ctx.battery, 100);
    }

    TEST(IntegrationTest, ScanWaitsForEnemy)
    {
        RobotContext ctx;
        ScanForEnemy scan;

        EXPECT_EQ(scan.process(TickEvent{}, ctx), Status::Running);
        EXPECT_EQ(scan.process(TickEvent{}, ctx), Status::Running);
        EXPECT_EQ(ctx.enemies_detected, 0);

        EXPECT_EQ(scan.process(EnemyEvent{100, 1}, ctx), Status::Success);
        EXPECT_EQ(ctx.enemies_detected, 1);
    }

    TEST(IntegrationTest, CombatSequence)
    {
        RobotContext ctx;
        ctx.battery = 100;

        Sequence<Event, RobotContext, CheckBattery, ScanForEnemy, FireWeapon> combat(
            CheckBattery(20),
            ScanForEnemy{},
            FireWeapon(3));

        // Battery OK, scanning...
        EXPECT_EQ(combat.process(TickEvent{}, ctx), Status::Running);

        // Enemy detected! FireWeapon starts but only fires on Tick
        EXPECT_EQ(combat.process(EnemyEvent{50, 1}, ctx), Status::Running);
        EXPECT_EQ(ctx.enemies_detected, 1);
        EXPECT_EQ(ctx.shots_fired, 0); // EnemyEvent doesn't trigger fire

        // First tick - first shot
        EXPECT_EQ(combat.process(TickEvent{}, ctx), Status::Running);
        EXPECT_EQ(ctx.shots_fired, 1);

        // Second tick - second shot
        EXPECT_EQ(combat.process(TickEvent{}, ctx), Status::Running);
        EXPECT_EQ(ctx.shots_fired, 2);

        // Third tick - final shot, completes
        EXPECT_EQ(combat.process(TickEvent{}, ctx), Status::Success);
        EXPECT_EQ(ctx.shots_fired, 3);
    }

    TEST(IntegrationTest, FallbackOnLowBattery)
    {
        RobotContext ctx;
        ctx.battery = 10; // Low battery!

        Selector<Event, RobotContext,
                 Sequence<Event, RobotContext, CheckBattery, ScanForEnemy>,
                 ActivateAlarm>
            root(
                Sequence<Event, RobotContext, CheckBattery, ScanForEnemy>(
                    CheckBattery(20),
                    ScanForEnemy{}),
                ActivateAlarm{});

        // Battery fails, alarm activates
        auto result = root.process(TickEvent{}, ctx);
        EXPECT_EQ(result, Status::Success);
        EXPECT_TRUE(ctx.alarm_active);
    }

    TEST(IntegrationTest, RetryOnScanFailure)
    {
        // Simulate a flaky scanner that fails twice then succeeds
        struct FlakyScan : NodeBase
        {
            int attempts = 0;
            int fail_count;

            explicit FlakyScan(int fails = 2) : fail_count(fails) {}

            Status process(const Event &, RobotContext &)
            {
                attempts++;
                if (attempts <= fail_count)
                    return Status::Failure;
                return Status::Success;
            }
            void reset() { /* Don't reset attempts - we want cumulative */ }
        };

        RobotContext ctx;
        Retry<Event, RobotContext, FlakyScan> retry_scan(5, FlakyScan(2));

        // First attempt fails
        EXPECT_EQ(retry_scan.process(TickEvent{}, ctx), Status::Running);

        // Second attempt fails
        EXPECT_EQ(retry_scan.process(TickEvent{}, ctx), Status::Running);

        // Third attempt succeeds
        EXPECT_EQ(retry_scan.process(TickEvent{}, ctx), Status::Success);
    }

    TEST(IntegrationTest, ParallelCombatManeuver)
    {
        struct MoveToCover : NodeBase
        {
            bool *moved;
            explicit MoveToCover(bool *m) : moved(m) {}
            Status process(const Event &, RobotContext &)
            {
                *moved = true;
                return Status::Success;
            }
            void reset() {}
        };

        RobotContext ctx;
        bool moved = false;

        Parallel<Event, RobotContext, MoveToCover, FireWeapon> maneuver(
            MoveToCover(&moved),
            FireWeapon(2));

        // First tick: Move completes, Fire starts
        EXPECT_EQ(maneuver.process(TickEvent{}, ctx), Status::Running);
        EXPECT_TRUE(moved);
        EXPECT_EQ(ctx.shots_fired, 1);

        // Second tick: Fire completes
        EXPECT_EQ(maneuver.process(TickEvent{}, ctx), Status::Success);
        EXPECT_EQ(ctx.shots_fired, 2);
    }

    TEST(IntegrationTest, ComplexNestedTree)
    {
        RobotContext ctx;
        ctx.battery = 100;

        // Tree structure:
        // Selector
        //   ├─ Sequence (Combat)
        //   │    ├─ CheckBattery
        //   │    └─ Parallel
        //   │         ├─ ScanForEnemy
        //   │         └─ Inverter(AlwaysFailure)  // becomes Success
        //   └─ ActivateAlarm (Fallback)

        using AlwaysFail = AlwaysFailure<Event, RobotContext>;

        Selector<Event, RobotContext,
                 Sequence<Event, RobotContext,
                          CheckBattery,
                          Parallel<Event, RobotContext,
                                   ScanForEnemy,
                                   Inverter<Event, RobotContext, AlwaysFail>>>,
                 ActivateAlarm>
            root(
                Sequence<Event, RobotContext,
                         CheckBattery,
                         Parallel<Event, RobotContext,
                                  ScanForEnemy,
                                  Inverter<Event, RobotContext, AlwaysFail>>>(
                    CheckBattery(20),
                    Parallel<Event, RobotContext, ScanForEnemy, Inverter<Event, RobotContext, AlwaysFail>>(
                        ScanForEnemy{},
                        Inverter<Event, RobotContext, AlwaysFail>(AlwaysFail{}))),
                ActivateAlarm{});

        // Tick: Battery OK, parallel running (scan waiting, inverter succeeds)
        EXPECT_EQ(root.process(TickEvent{}, ctx), Status::Running);
        EXPECT_FALSE(ctx.alarm_active);

        // Enemy event: Scan succeeds, parallel completes
        EXPECT_EQ(root.process(EnemyEvent{100, 1}, ctx), Status::Success);
        EXPECT_FALSE(ctx.alarm_active);
    }

    TEST(IntegrationTest, TreeReset)
    {
        RobotContext ctx;

        FireWeapon fire(3);
        fire.process(TickEvent{}, ctx);
        fire.process(TickEvent{}, ctx);
        EXPECT_EQ(fire.shots, 2);

        fire.reset();
        EXPECT_EQ(fire.shots, 0);
    }

    // ==========================================
    // FACTORY HELPER TESTS
    // ==========================================

    TEST(FactoryTest, MakeSequence)
    {
        RobotContext ctx;
        ctx.battery = 100;

        auto seq = make_sequence<Event, RobotContext>(
            CheckBattery(20),
            ActivateAlarm{});

        auto result = seq.process(TickEvent{}, ctx);
        EXPECT_EQ(result, Status::Success);
        EXPECT_TRUE(ctx.alarm_active);
    }

    TEST(FactoryTest, MakeSelector)
    {
        RobotContext ctx;
        ctx.battery = 10;

        auto sel = make_selector<Event, RobotContext>(
            CheckBattery(20),
            ActivateAlarm{});

        auto result = sel.process(TickEvent{}, ctx);
        EXPECT_EQ(result, Status::Success);
        EXPECT_TRUE(ctx.alarm_active);
    }

    TEST(FactoryTest, MakeInverter)
    {
        RobotContext ctx;

        auto inv = make_inverter<Event, RobotContext>(
            AlwaysFailure<Event, RobotContext>{});

        EXPECT_EQ(inv.process(TickEvent{}, ctx), Status::Success);
    }

} // anonymous namespace
