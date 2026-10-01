#pragma once

#include "behavior_tree.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace bt
{

    namespace internal
    {

        template <typename T> constexpr std::string_view get_type_name()
        {
#if defined(__clang__)
            std::string_view name = __PRETTY_FUNCTION__;
            std::string_view prefix = "std::string_view bt::internal::get_type_name() [T = ";
            std::string_view suffix = "]";
#elif defined(__GNUC__)
            std::string_view name = __PRETTY_FUNCTION__;
            std::string_view prefix = "constexpr std::string_view bt::internal::get_type_name() [with T = ";
            std::string_view suffix = "]";
#elif defined(_MSC_VER)
            std::string_view name = __FUNCSIG__;
            std::string_view prefix = "class std::basic_string_view<char,struct std::char_traits<char> > __cdecl "
                                      "bt::internal::get_type_name<";
            std::string_view suffix = ">(void)";
#else
            return "UnknownType";
#endif

            auto start = name.find(prefix);
            if (start == std::string_view::npos)
                return "Unknown";
            start += prefix.size();

            auto end = name.find(suffix, start);
            if (end == std::string_view::npos)
                return name.substr(start);

            auto type_str = name.substr(start, end - start);

            if (type_str.starts_with("struct "))
                type_str.remove_prefix(7);
            if (type_str.starts_with("class "))
                type_str.remove_prefix(6);

            auto semi = type_str.find(';');
            if (semi != std::string_view::npos)
                type_str = type_str.substr(0, semi);

            return type_str;
        }

        template <typename T> struct NodePrinter;

        template <typename T> void print_tree_impl(const T &node, int indent, std::ostream &os);

        inline void print_indent(int indent, std::ostream &os)
        {
            for (int i = 0; i < indent; ++i)
                os << "  ";
        }

        template <typename E, typename C, typename... Children> struct NodePrinter<Sequence<E, C, Children...>>
        {
            static void print(const Sequence<E, C, Children...> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Sequence\n";
                std::apply([&](const auto &...children) { (print_tree_impl(children, indent + 1, os), ...); },
                           node.children);
            }
        };

        template <typename E, typename C, typename... Children> struct NodePrinter<Selector<E, C, Children...>>
        {
            static void print(const Selector<E, C, Children...> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Selector\n";
                std::apply([&](const auto &...children) { (print_tree_impl(children, indent + 1, os), ...); },
                           node.children);
            }
        };

        template <typename E, typename C, typename... Children> struct NodePrinter<Parallel<E, C, Children...>>
        {
            static void print(const Parallel<E, C, Children...> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Parallel\n";
                std::apply([&](const auto &...children) { (print_tree_impl(children, indent + 1, os), ...); },
                           node.children);
            }
        };

        template <typename E, typename C, typename Child> struct NodePrinter<Inverter<E, C, Child>>
        {
            static void print(const Inverter<E, C, Child> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Inverter\n";
                print_tree_impl(node.child, indent + 1, os);
            }
        };

        template <typename E, typename C, typename Child> struct NodePrinter<Retry<E, C, Child>>
        {
            static void print(const Retry<E, C, Child> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Retry (" << node.max_attempts << "x)\n";
                print_tree_impl(node.child, indent + 1, os);
            }
        };

        template <typename E, typename C, typename Child> struct NodePrinter<Repeat<E, C, Child>>
        {
            static void print(const Repeat<E, C, Child> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Repeat (" << node.max_iterations << "x)\n";
                print_tree_impl(node.child, indent + 1, os);
            }
        };

        template <typename E, typename C, typename Child> struct NodePrinter<Timeout<E, C, Child>>
        {
            static void print(const Timeout<E, C, Child> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Timeout (" << node.max_ticks << " ticks)\n";
                print_tree_impl(node.child, indent + 1, os);
            }
        };

        template <typename E, typename C, typename Child> struct NodePrinter<Succeeder<E, C, Child>>
        {
            static void print(const Succeeder<E, C, Child> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Succeeder\n";
                print_tree_impl(node.child, indent + 1, os);
            }
        };

        template <typename E, typename C, typename Child> struct NodePrinter<Failer<E, C, Child>>
        {
            static void print(const Failer<E, C, Child> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Failer\n";
                print_tree_impl(node.child, indent + 1, os);
            }
        };

        template <typename E, typename C, typename Pred, typename Child> struct NodePrinter<Guard<E, C, Pred, Child>>
        {
            static void print(const Guard<E, C, Pred, Child> &node, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << "Guard\n";
                print_tree_impl(node.child, indent + 1, os);
            }
        };

        template <typename T> struct NodePrinter
        {
            static void print(const T &, int indent, std::ostream &os)
            {
                print_indent(indent, os);
                os << get_type_name<T>() << "\n";
            }
        };

        template <typename T> void print_tree_impl(const T &node, int indent, std::ostream &os)
        {
            NodePrinter<std::decay_t<T>>::print(node, indent, os);
        }

    } // namespace internal

    template <typename T> void print_tree(const T &node, std::ostream &os = std::cout)
    {
        internal::print_tree_impl(node, 0, os);
    }

    namespace internal
    {

        inline std::string dot_escaped(std::string_view label)
        {
            std::string escaped;
            for (char c : label)
            {
                if (c == '"' || c == '\\')
                    escaped += '\\';
                escaped += c;
            }
            return escaped;
        }

    } // namespace internal

    template <typename T> void to_dot(const T &node, std::ostream &os = std::cout)
    {
        std::ostringstream indented;
        print_tree(node, indented);

        os << "digraph behavior_tree {\n";
        std::vector<std::size_t> last_at_depth;
        std::istringstream lines(indented.str());
        std::string line;
        for (std::size_t id = 0; std::getline(lines, line); ++id)
        {
            const auto label_start = line.find_first_not_of(' ');
            const auto depth = label_start / 2;
            os << "  n" << id << " [label=\"" << internal::dot_escaped(line.substr(label_start)) << "\"];\n";
            if (depth > 0)
                os << "  n" << last_at_depth[depth - 1] << " -> n" << id << ";\n";
            last_at_depth.resize(depth + 1);
            last_at_depth[depth] = id;
        }
        os << "}\n";
    }

} // namespace bt