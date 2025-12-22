#pragma once

#include "behavior_tree.hpp"
#include <type_traits>

namespace bt
{

    // ==========================================
    // TYPE DEDUCTION TRAITS
    // ==========================================

    // 1. Try to get Event/Context from typedefs (Library nodes)
    template <typename T>
    concept HasNodeTypedefs = requires {
        typename T::EventType;
        typename T::ContextType;
    };

    // 2. Helper to deduce from process() method (User nodes)
    template <typename T>
    struct ProcessTraits;

    // Match process(const Event&, Context&)
    template <typename R, typename C, typename E, typename Ctx>
    struct ProcessTraits<R (C::*)(const E &, Ctx &)>
    {
        using Event = E;
        using Context = Ctx;
    };

    // Match process(const Event&, Context&) const
    template <typename R, typename C, typename E, typename Ctx>
    struct ProcessTraits<R (C::*)(const E &, Ctx &) const>
    {
        using Event = E;
        using Context = Ctx;
    };

    // Traits Wrapper
    template <typename T>
    struct NodeTraits
    {
        using Node = std::decay_t<T>;

        // Fallback logic
        using Event = typename std::conditional_t<
            HasNodeTypedefs<Node>,
            Node,
            ProcessTraits<decltype(&Node::process)>>::EventType; // Alias to match concept or struct member

        using Context = typename std::conditional_t<
            HasNodeTypedefs<Node>,
            Node,
            ProcessTraits<decltype(&Node::process)>>::ContextType;
    };

    // Specialization helpers for Traits that don't match the simple struct
    template <typename T>
        requires(!HasNodeTypedefs<std::decay_t<T>>)
    struct NodeTraits<T>
    {
        using PTraits = ProcessTraits<decltype(&std::decay_t<T>::process)>;
        using EventType = typename PTraits::Event;
        using ContextType = typename PTraits::Context;
    };

    template <typename T>
        requires(HasNodeTypedefs<std::decay_t<T>>)
    struct NodeTraits<T>
    {
        using EventType = typename std::decay_t<T>::EventType;
        using ContextType = typename std::decay_t<T>::ContextType;
    };

    // ==========================================
    // OPERATORS
    // ==========================================

    // --- SEQUENCE (>>) ---

    // Case 1: Node >> Node
    template <typename L, typename R>
    constexpr auto operator>>(L &&l, R &&r)
    {
        using E = typename NodeTraits<L>::EventType;
        using C = typename NodeTraits<L>::ContextType;
        return make_sequence<E, C>(std::forward<L>(l), std::forward<R>(r));
    }

    // Case 2: Sequence >> Node (Flattening)
    template <typename E, typename C, typename... Children, typename R>
    constexpr auto operator>>(Sequence<E, C, Children...> &&seq, R &&r)
    {
        return std::apply([&](auto &&...args)
                          { return make_sequence<E, C>(std::forward<decltype(args)>(args)..., std::forward<R>(r)); }, std::move(seq.children));
    }

    // Case 3: Node >> Sequence (Flattening)
    template <typename L, typename E, typename C, typename... Children>
    constexpr auto operator>>(L &&l, Sequence<E, C, Children...> &&seq)
    {
        return std::apply([&](auto &&...args)
                          { return make_sequence<E, C>(std::forward<L>(l), std::forward<decltype(args)>(args)...); }, std::move(seq.children));
    }

    // --- SELECTOR (|) ---

    // Case 1: Node | Node
    template <typename L, typename R>
    constexpr auto operator|(L &&l, R &&r)
    {
        using E = typename NodeTraits<L>::EventType;
        using C = typename NodeTraits<L>::ContextType;
        return make_selector<E, C>(std::forward<L>(l), std::forward<R>(r));
    }

    // Case 2: Selector | Node (Flattening)
    template <typename E, typename C, typename... Children, typename R>
    constexpr auto operator|(Selector<E, C, Children...> &&sel, R &&r)
    {
        return std::apply([&](auto &&...args)
                          { return make_selector<E, C>(std::forward<decltype(args)>(args)..., std::forward<R>(r)); }, std::move(sel.children));
    }

    // --- INVERTER (!) ---

    template <typename T>
    constexpr auto operator!(T &&node)
    {
        using E = typename NodeTraits<T>::EventType;
        using C = typename NodeTraits<T>::ContextType;
        return make_inverter<E, C>(std::forward<T>(node));
    }

} // namespace bt