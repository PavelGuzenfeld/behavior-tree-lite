#include "behavior_tree_lite/behavior_tree.hpp"
#include <doctest/doctest.h>
#include <variant>

using namespace bt;

namespace
{

    struct TickEvent
    {
        double dt = 0.1;
    };
    struct BatteryEvent
    {
        int voltage;
    };
    struct ObjectEvent
    {
        int dist;
        int id;
    };

    using Event = std::variant<TickEvent, BatteryEvent, ObjectEvent>;

    struct RobotContext
    {
        int battery = 100;
        bool alarm_active = false;
        int objects_detected = 0;
        int samples_taken = 0;
    };

    struct CheckBattery : NodeBase
    {
        int threshold;
        explicit CheckBattery(int t = 20) : threshold(t) {}

        Status process(const Event &e, RobotContext &ctx)
        {
            std::visit(overloaded{[&](const BatteryEvent &b) { ctx.battery = b.voltage; },
                                  [](const auto &) {
                                  }},
                       e);
            return (ctx.battery >= threshold) ? Status::Success : Status::Failure;
        }
        void reset() {}
    };

    struct ScanForObject : NodeBase
    {
        Status process(const Event &e, RobotContext &ctx)
        {
            return std::visit(overloaded{[&](const ObjectEvent &)
                                         {
                                             ctx.objects_detected++;
                                             return Status::Success;
                                         },
                                         [](const TickEvent &) { return Status::Running; },
                                         [](const auto &)
                                         {
                                             return Status::Running;
                                         }},
                              e);
        }
        void reset() {}
    };

    struct TakeSample : NodeBase
    {
        int samples_needed;
        int samples = 0;

        explicit TakeSample(int s = 3) : samples_needed(s) {}

        Status process(const Event &e, RobotContext &ctx)
        {
            if (std::holds_alternative<TickEvent>(e))
            {
                samples++;
                ctx.samples_taken++;
                if (samples >= samples_needed)
                    return Status::Success;
            }
            return Status::Running;
        }
        void reset() { samples = 0; }
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

    TEST_CASE("IntegrationTest.VariantEventDispatch")
    {
        RobotContext ctx;
        ctx.battery = 50;

        CheckBattery check(20);

        CHECK_EQ(check.process(TickEvent{}, ctx), Status::Success);
        CHECK_EQ(check.process(BatteryEvent{10}, ctx), Status::Failure);
        CHECK_EQ(ctx.battery, 10);
        CHECK_EQ(check.process(BatteryEvent{100}, ctx), Status::Success);
        CHECK_EQ(ctx.battery, 100);
    }

    TEST_CASE("IntegrationTest.ScanWaitsForEnemy")
    {
        RobotContext ctx;
        ScanForObject scan;

        CHECK_EQ(scan.process(TickEvent{}, ctx), Status::Running);
        CHECK_EQ(scan.process(TickEvent{}, ctx), Status::Running);
        CHECK_EQ(ctx.objects_detected, 0);

        CHECK_EQ(scan.process(ObjectEvent{100, 1}, ctx), Status::Success);
        CHECK_EQ(ctx.objects_detected, 1);
    }

    TEST_CASE("IntegrationTest.InspectionSequence")
    {
        RobotContext ctx;
        ctx.battery = 100;

        Sequence<Event, RobotContext, CheckBattery, ScanForObject, TakeSample> inspection(
            CheckBattery(20), ScanForObject{}, TakeSample(3));

        CHECK_EQ(inspection.process(TickEvent{}, ctx), Status::Running);

        CHECK_EQ(inspection.process(ObjectEvent{50, 1}, ctx), Status::Running);
        CHECK_EQ(ctx.objects_detected, 1);
        CHECK_EQ(ctx.samples_taken, 0);

        CHECK_EQ(inspection.process(TickEvent{}, ctx), Status::Running);
        CHECK_EQ(ctx.samples_taken, 1);

        CHECK_EQ(inspection.process(TickEvent{}, ctx), Status::Running);
        CHECK_EQ(ctx.samples_taken, 2);

        CHECK_EQ(inspection.process(TickEvent{}, ctx), Status::Success);
        CHECK_EQ(ctx.samples_taken, 3);
    }

    TEST_CASE("IntegrationTest.FallbackOnLowBattery")
    {
        RobotContext ctx;
        ctx.battery = 10;

        Selector<Event, RobotContext, Sequence<Event, RobotContext, CheckBattery, ScanForObject>, ActivateAlarm> root(
            Sequence<Event, RobotContext, CheckBattery, ScanForObject>(CheckBattery(20), ScanForObject{}),
            ActivateAlarm{});

        auto result = root.process(TickEvent{}, ctx);
        CHECK_EQ(result, Status::Success);
        CHECK(ctx.alarm_active);
    }

    TEST_CASE("IntegrationTest.RetryOnScanFailure")
    {
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
            void reset() {}
        };

        RobotContext ctx;
        Retry<Event, RobotContext, FlakyScan> retry_scan(5, FlakyScan(2));

        CHECK_EQ(retry_scan.process(TickEvent{}, ctx), Status::Running);

        CHECK_EQ(retry_scan.process(TickEvent{}, ctx), Status::Running);

        CHECK_EQ(retry_scan.process(TickEvent{}, ctx), Status::Success);
    }

    TEST_CASE("IntegrationTest.ParallelInspection")
    {
        struct MoveToStation : NodeBase
        {
            bool *moved;
            explicit MoveToStation(bool *m) : moved(m) {}
            Status process(const Event &, RobotContext &)
            {
                *moved = true;
                return Status::Success;
            }
            void reset() {}
        };

        RobotContext ctx;
        bool moved = false;

        Parallel<Event, RobotContext, MoveToStation, TakeSample> maneuver(MoveToStation(&moved), TakeSample(2));

        CHECK_EQ(maneuver.process(TickEvent{}, ctx), Status::Running);
        CHECK(moved);
        CHECK_EQ(ctx.samples_taken, 1);

        CHECK_EQ(maneuver.process(TickEvent{}, ctx), Status::Success);
        CHECK_EQ(ctx.samples_taken, 2);
    }

    TEST_CASE("IntegrationTest.ComplexNestedTree")
    {
        RobotContext ctx;
        ctx.battery = 100;

        using AlwaysFail = AlwaysFailure<Event, RobotContext>;

        Selector<Event, RobotContext,
                 Sequence<Event, RobotContext, CheckBattery,
                          Parallel<Event, RobotContext, ScanForObject, Inverter<Event, RobotContext, AlwaysFail>>>,
                 ActivateAlarm>
            root(Sequence<Event, RobotContext, CheckBattery,
                          Parallel<Event, RobotContext, ScanForObject, Inverter<Event, RobotContext, AlwaysFail>>>(
                     CheckBattery(20),
                     Parallel<Event, RobotContext, ScanForObject, Inverter<Event, RobotContext, AlwaysFail>>(
                         ScanForObject{}, Inverter<Event, RobotContext, AlwaysFail>(AlwaysFail{}))),
                 ActivateAlarm{});

        CHECK_EQ(root.process(TickEvent{}, ctx), Status::Running);
        CHECK_FALSE(ctx.alarm_active);

        CHECK_EQ(root.process(ObjectEvent{100, 1}, ctx), Status::Success);
        CHECK_FALSE(ctx.alarm_active);
    }

    TEST_CASE("IntegrationTest.TreeReset")
    {
        RobotContext ctx;

        TakeSample sampler(3);
        sampler.process(TickEvent{}, ctx);
        sampler.process(TickEvent{}, ctx);
        CHECK_EQ(sampler.samples, 2);

        sampler.reset();
        CHECK_EQ(sampler.samples, 0);
    }

    TEST_CASE("FactoryTest.MakeSequence")
    {
        RobotContext ctx;
        ctx.battery = 100;

        auto seq = make_sequence<Event, RobotContext>(CheckBattery(20), ActivateAlarm{});

        auto result = seq.process(TickEvent{}, ctx);
        CHECK_EQ(result, Status::Success);
        CHECK(ctx.alarm_active);
    }

    TEST_CASE("FactoryTest.MakeSelector")
    {
        RobotContext ctx;
        ctx.battery = 10;

        auto sel = make_selector<Event, RobotContext>(CheckBattery(20), ActivateAlarm{});

        auto result = sel.process(TickEvent{}, ctx);
        CHECK_EQ(result, Status::Success);
        CHECK(ctx.alarm_active);
    }

    TEST_CASE("FactoryTest.MakeInverter")
    {
        RobotContext ctx;

        auto inv = make_inverter<Event, RobotContext>(AlwaysFailure<Event, RobotContext>{});

        CHECK_EQ(inv.process(TickEvent{}, ctx), Status::Success);
    }

} // namespace
