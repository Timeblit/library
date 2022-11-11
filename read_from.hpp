#pragma once

#include "unspecified_tag.hpp"
#include "all_types.hpp"

#include <fmt/format.h>
#include <fmt/compile.h>
#include <fmt/color.h>

#include <span>
#include <concepts>
#include <cstddef>
#include <cstring>

// This file defines function movency::read_from
//
// It is used to read data as an object of type T
//
// T can be explicitly specified as a template parameter, or deduced from the type of the variable which you save the result of read_from into
//
// It takes any pointer or span, and does (runtime) bounds-checking in the span case
// 
// If an overread is detected, 0 is returned and an error is printed
//
// Examples for usage can be found in read_from_test.cpp:
//
// TODO?: put this somewhere other than types/ ?


namespace movency {

namespace detail {

// returned from unchecked versions of read_from
struct read_from_unchecked_temporary
{
  const std::byte* const ptr;

  template<class T>
  [[nodiscard, gnu::always_inline]] inline operator T() const
  {
    T out;

    std::memcpy(&out, ptr, sizeof(out));

    return out;
  }
};

// returned from checked versions of read_from
struct read_from_checked_temporary
{
  const std::byte* const ptr;
  std::size_t size;

  template<class T>
  [[nodiscard, gnu::always_inline]] inline operator T() const
  {
    if (sizeof(T) > size) [[unlikely]]
    {
      fmt::print(fmt::emphasis::bold | fg(fmt::color::red), "read_from with span is attempting to copy object of size {} from buffer of size {}\n", sizeof(T), size);

      return static_cast<T>(0);
    }

    T out;

    std::memcpy(&out, ptr, sizeof(out));

    return out;
  }
};

} // namespace detail

// safe, checked at runtime (may overread buffer)
template<class To = unspecified_tag, class B>
[[nodiscard, gnu::always_inline]] inline auto read_from(const std::span<B> buf)
{
  const auto alias = reinterpret_cast<const std::byte* const>(buf.data());

  if constexpr (std::same_as<To, unspecified_tag>)
    return                 detail::read_from_checked_temporary{alias, sizeof(B) * buf.size()};
  else
    return static_cast<To>(detail::read_from_checked_temporary{alias, sizeof(B) * buf.size()});
}

// unsafe, unchecked (cannot overread buffer)
template<class To = unspecified_tag>
[[nodiscard, gnu::always_inline]] inline auto read_from(const auto* const buf)
{
  auto alias = reinterpret_cast<const std::byte* const>(buf);

  if constexpr (std::same_as<To, unspecified_tag>)
    return                 detail::read_from_unchecked_temporary{alias};
  else
    return static_cast<To>(detail::read_from_unchecked_temporary{alias});
}


} // namespace movency

