#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <utility>
#include <variant>

namespace bt
{

    // ==========================================
    // STATUS
    // ==========================================

    enum class Status : std::uint8_t
    {
        Success,
        Failure,
        Running
    };

    // Compile-time status names lookup
    inline constexpr std::array<std::string_view, 3> status_names = {
        "Success", "Failure", "Running"};

    constexpr std::string_view to_string(Status s) noexcept
    {
        return status_names[std::to_underlying(s)];
    }

    // ==========================================
    // CONCEPTS
    // ==========================================

    template <typename T, typename Event, typename Context>
    concept IsNode = requires(T t, const Event &e, Context &ctx) {
        { t.process(e, ctx) } -> std::same_as<Status>;
        { t.reset() } -> std::same_as<void>;
    };

    template <typename T, typename Event, typename Context>
    concept IsStatelessNode = IsNode<T, Event, Context> && std::is_empty_v<T>;

    // ==========================================
    // BASE
    // ==========================================

    struct NodeBase
    {
        NodeBase() = default;
        NodeBase(const NodeBase &) = delete;
        NodeBase &operator=(const NodeBase &) = delete;
        NodeBase(NodeBase &&) = default;
        NodeBase &operator=(NodeBase &&) = default;
    };

    // ==========================================
    // HELPERS
    // ==========================================

    template <class... Ts>
    struct overloaded : Ts...
    {
        using Ts::operator()...;
    };

    template <class... Ts>
    overloaded(Ts...) -> overloaded<Ts...>;

} // namespace bt
