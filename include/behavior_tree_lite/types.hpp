#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <utility>
#include <variant>

namespace bt
{

    enum class Status : std::uint8_t
    {
        Success,
        Failure,
        Running
    };

    inline constexpr std::array<std::string_view, 3> status_names = {"Success", "Failure", "Running"};

    constexpr std::string_view to_string(Status s) noexcept
    {
        return status_names[std::to_underlying(s)];
    }

    template <typename T, typename Event, typename Context>
    concept HasProcess = requires(T t, const Event &e, Context &ctx) {
        { t.process(e, ctx) } -> std::same_as<Status>;
    };

    template <typename T, typename Event, typename Context>
    concept HasCallOperator = requires(T t, const Event &e, Context &ctx) {
        { t(e, ctx) } -> std::same_as<Status>;
    };

    template <typename T, typename Event, typename Context>
    concept IsNode = HasProcess<T, Event, Context> || HasCallOperator<T, Event, Context>;

    template <typename Event, typename Context, typename T>
        requires IsNode<T, Event, Context>
    constexpr Status call_node(T &node, const Event &e, Context &ctx)
    {
        if constexpr (HasProcess<T, Event, Context>)
            return node.process(e, ctx);
        else
            return node(e, ctx);
    }

    template <typename T>
    concept HasReset = requires(T t) {
        { t.reset() } -> std::same_as<void>;
    };

    template <typename T> constexpr void reset_node(T &node)
    {
        if constexpr (HasReset<T>)
            node.reset();
    }

    template <typename T, typename Event, typename Context>
    concept IsStatelessNode = IsNode<T, Event, Context> && std::is_empty_v<T>;

    struct NodeBase
    {
        NodeBase() = default;
        NodeBase(const NodeBase &) = delete;
        NodeBase &operator=(const NodeBase &) = delete;
        NodeBase(NodeBase &&) = default;
        NodeBase &operator=(NodeBase &&) = default;

        constexpr auto operator()(this auto &&self, const auto &e, auto &ctx) -> decltype(self.process(e, ctx))
        {
            return self.process(e, ctx);
        }
    };

    template <class... Ts> struct overloaded : Ts...
    {
        using Ts::operator()...;
    };

    template <class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

} // namespace bt
