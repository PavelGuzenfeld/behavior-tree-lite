#pragma once

#include "behavior_tree.hpp"
#include <type_traits>

namespace bt
{

    template <typename T>
    concept DerivedFromNodeBase = std::is_base_of_v<NodeBase, std::decay_t<T>>;

    template <typename T>
    concept HasNodeTypedefs = requires {
        typename std::decay_t<T>::EventType;
        typename std::decay_t<T>::ContextType;
    };

    template <typename T> struct is_bt_library_node : std::false_type
    {
    };

    template <typename E, typename C, typename... Ts> struct is_bt_library_node<Sequence<E, C, Ts...>> : std::true_type
    {
    };

    template <typename E, typename C, typename... Ts> struct is_bt_library_node<Selector<E, C, Ts...>> : std::true_type
    {
    };

    template <typename E, typename C, typename... Ts> struct is_bt_library_node<Parallel<E, C, Ts...>> : std::true_type
    {
    };

    template <typename E, typename C, typename T> struct is_bt_library_node<Inverter<E, C, T>> : std::true_type
    {
    };

    template <typename E, typename C, typename T> struct is_bt_library_node<Retry<E, C, T>> : std::true_type
    {
    };

    template <typename E, typename C, typename T> struct is_bt_library_node<Repeat<E, C, T>> : std::true_type
    {
    };

    template <typename E, typename C, typename T> struct is_bt_library_node<Succeeder<E, C, T>> : std::true_type
    {
    };

    template <typename E, typename C, typename T> struct is_bt_library_node<Failer<E, C, T>> : std::true_type
    {
    };

    template <typename E, typename C, typename T> struct is_bt_library_node<Timeout<E, C, T>> : std::true_type
    {
    };

    template <typename T>
    concept IsLibraryNode = is_bt_library_node<std::decay_t<T>>::value;

    template <typename T> struct ProcessTraits;

    template <typename R, typename C, typename E, typename Ctx> struct ProcessTraits<R (C::*)(E const &, Ctx &)>
    {
        using EventType = std::remove_cvref_t<E>;
        using ContextType = std::remove_cvref_t<Ctx>;
    };

    template <typename R, typename C, typename E, typename Ctx> struct ProcessTraits<R (C::*)(E const &, Ctx &) const>
    {
        using EventType = std::remove_cvref_t<E>;
        using ContextType = std::remove_cvref_t<Ctx>;
    };

    template <typename T>
    concept HasDeducibleProcess = requires {
        typename ProcessTraits<decltype(&std::decay_t<T>::process)>::EventType;
        typename ProcessTraits<decltype(&std::decay_t<T>::process)>::ContextType;
    };

    template <typename T>
    concept IsBTNode = HasNodeTypedefs<T> || IsLibraryNode<T> || HasDeducibleProcess<T>;

    template <typename T> struct LibraryNodeTraits;

    template <typename E, typename C, typename... Ts> struct LibraryNodeTraits<Sequence<E, C, Ts...>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename... Ts> struct LibraryNodeTraits<Selector<E, C, Ts...>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename... Ts> struct LibraryNodeTraits<Parallel<E, C, Ts...>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename T> struct LibraryNodeTraits<Inverter<E, C, T>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename T> struct LibraryNodeTraits<Retry<E, C, T>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename T> struct LibraryNodeTraits<Repeat<E, C, T>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename T> struct LibraryNodeTraits<Succeeder<E, C, T>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename T> struct LibraryNodeTraits<Failer<E, C, T>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename E, typename C, typename T> struct LibraryNodeTraits<Timeout<E, C, T>>
    {
        using EventType = E;
        using ContextType = C;
    };

    template <typename T, typename = void> struct EventOfImpl;

    template <typename T> struct EventOfImpl<T, std::enable_if_t<IsLibraryNode<std::decay_t<T>>>>
    {
        using type = typename LibraryNodeTraits<std::decay_t<T>>::EventType;
    };

    template <typename T>
    struct EventOfImpl<T, std::enable_if_t<!IsLibraryNode<std::decay_t<T>> && HasNodeTypedefs<std::decay_t<T>>>>
    {
        using type = typename std::decay_t<T>::EventType;
    };

    template <typename T>
    struct EventOfImpl<T, std::enable_if_t<!IsLibraryNode<std::decay_t<T>> && !HasNodeTypedefs<std::decay_t<T>> &&
                                           HasDeducibleProcess<std::decay_t<T>>>>
    {
        using type = typename ProcessTraits<decltype(&std::decay_t<T>::process)>::EventType;
    };

    template <typename T, typename = void> struct ContextOfImpl;

    template <typename T> struct ContextOfImpl<T, std::enable_if_t<IsLibraryNode<std::decay_t<T>>>>
    {
        using type = typename LibraryNodeTraits<std::decay_t<T>>::ContextType;
    };

    template <typename T>
    struct ContextOfImpl<T, std::enable_if_t<!IsLibraryNode<std::decay_t<T>> && HasNodeTypedefs<std::decay_t<T>>>>
    {
        using type = typename std::decay_t<T>::ContextType;
    };

    template <typename T>
    struct ContextOfImpl<T, std::enable_if_t<!IsLibraryNode<std::decay_t<T>> && !HasNodeTypedefs<std::decay_t<T>> &&
                                             HasDeducibleProcess<std::decay_t<T>>>>
    {
        using type = typename ProcessTraits<decltype(&std::decay_t<T>::process)>::ContextType;
    };

    template <typename T> using EventOf = typename EventOfImpl<T>::type;

    template <typename T> using ContextOf = typename ContextOfImpl<T>::type;

    template <typename L, typename R>
        requires IsBTNode<L> && IsBTNode<R>
    constexpr auto operator&&(L &&l, R &&r)
    {
        using E = EventOf<L>;
        using C = ContextOf<L>;
        return make_sequence<E, C>(std::forward<L>(l), std::forward<R>(r));
    }

    template <typename E, typename C, typename... Children, typename R>
        requires IsBTNode<R>
    constexpr auto operator&&(Sequence<E, C, Children...> &&seq, R &&r)
    {
        return std::apply([&](auto &&...args)
                          { return make_sequence<E, C>(std::forward<decltype(args)>(args)..., std::forward<R>(r)); },
                          std::move(seq.children));
    }

    template <typename L, typename E, typename C, typename... Children>
        requires IsBTNode<L>
    constexpr auto operator&&(L &&l, Sequence<E, C, Children...> &&seq)
    {
        return std::apply([&](auto &&...args)
                          { return make_sequence<E, C>(std::forward<L>(l), std::forward<decltype(args)>(args)...); },
                          std::move(seq.children));
    }

    template <typename L, typename R>
        requires IsBTNode<L> && IsBTNode<R>
    constexpr auto operator||(L &&l, R &&r)
    {
        using E = EventOf<L>;
        using C = ContextOf<L>;
        return make_selector<E, C>(std::forward<L>(l), std::forward<R>(r));
    }

    template <typename E, typename C, typename... Children, typename R>
        requires IsBTNode<R>
    constexpr auto operator||(Selector<E, C, Children...> &&sel, R &&r)
    {
        return std::apply([&](auto &&...args)
                          { return make_selector<E, C>(std::forward<decltype(args)>(args)..., std::forward<R>(r)); },
                          std::move(sel.children));
    }

    template <typename T>
        requires IsBTNode<T>
    constexpr auto operator!(T &&node)
    {
        using E = EventOf<T>;
        using C = ContextOf<T>;
        return make_inverter<E, C>(std::forward<T>(node));
    }

} // namespace bt