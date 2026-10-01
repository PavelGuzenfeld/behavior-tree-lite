#pragma once

#include <tuple>
#include <type_traits>
#include <utility>

namespace bt
{

    template <typename T> struct Key
    {
        using value_type = T;
    };

    template <typename... Keys> class Blackboard
    {
        template <typename K> struct Entry
        {
            typename K::value_type value{};
        };

        template <typename K> static constexpr bool has_key = (std::is_same_v<K, Keys> || ...);

        std::tuple<Entry<Keys>...> entries_;

      public:
        template <typename K>
            requires has_key<K>
        constexpr typename K::value_type &get()
        {
            return std::get<Entry<K>>(entries_).value;
        }

        template <typename K>
            requires has_key<K>
        constexpr const typename K::value_type &get() const
        {
            return std::get<Entry<K>>(entries_).value;
        }

        template <typename K>
            requires has_key<K>
        constexpr void set(typename K::value_type value)
        {
            std::get<Entry<K>>(entries_).value = std::move(value);
        }
    };

} // namespace bt
