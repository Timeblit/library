#pragma once

#include "unspecified_tag.hpp"

#include <cstdint>
#include <concepts>
#include <type_traits>


namespace movency {

template<class T> struct  is_floating_point : std::bool_constant<std::is_floating_point_v<T>> {};

template<class T> concept    floating       = is_floating_point<T>::value;


template<class T> struct  is_unsigned_integral : std::bool_constant<std::is_unsigned_v<T> && std::is_integral_v<T>> {};

template<class T> concept    unsigned_integral = is_unsigned_integral<T>::value;


template<class T> struct  is_signed_integral : std::bool_constant<std::is_signed_v<T> && std::is_integral_v<T>> {};

template<class T> concept    signed_integral = is_signed_integral<T>::value;


template<class T> concept integral   = movency::unsigned_integral<T> || movency::signed_integral<T>;


template<class T> concept arithmetic = movency::integral<T> || movency::floating<T>;


template<class T> struct  is_custom_type : std::false_type {};

template<class T> concept custom_type = is_custom_type<T>::value;


template<class T> concept custom_floating = floating<T> && custom_type<T>;


template<class C, class E = typename C::value_type>
concept container_of = requires(C c, E e)
{
  requires std::same_as<std::remove_cv_t<typename C::value_type>, std::remove_cv_t<E>>;
};

template<class C> concept container                   = container_of<C, typename C::value_type>;

template<class C> concept container_of_floating_point = container<C> && floating<typename C::value_type>;

template<class C> concept container_of_bool           = container<C> && std::same_as<typename C::value_type, bool>;



using uint8      = std::uint8_t;      static_assert(sizeof(uint8)   == 1);
using uint16     = std::uint16_t;     static_assert(sizeof(uint16)  == 2);
using uint32     = std::uint32_t;     static_assert(sizeof(uint32)  == 4);
using uint64     = std::uint64_t;     static_assert(sizeof(uint64)  == 8);

using int8       = std::int8_t;       static_assert(sizeof(int8)    == 1);
using int16      = std::int16_t;      static_assert(sizeof(uint16)  == 2);
using int32      = std::int32_t;      static_assert(sizeof(uint32)  == 4);
using int64      = std::int64_t;      static_assert(sizeof(uint64)  == 8);

using float32    = float;             static_assert(sizeof(float32)  == 4);
using float64    = double;            static_assert(sizeof(float64)  == 8);


static_assert(sizeof(bool) == 1);

static_assert(std::same_as<uint8, unsigned char>);
static_assert(std::same_as< int8,   signed char>); // NOTE: int8 cannot be used to alias any data, unlike uint8 and std::byte

}
