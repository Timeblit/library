#pragma once

#include <span>
#include <type_traits>

// For functions which always want a generic span, create a wrapper function with overloads:
// [[gnu::always_inline]] auto func(spanifiable auto&&)                   which explicitly converts its parameter to a span
// [[gnu::always_inline]] auto func(const specialization_of_span auto&&)  which just passes on its value
//
// Thus func can be called with a pre-existing container, a pre-existing span, and a temporary span (but unfortunately cannot deduce span from {it, (sz|it)} unless explicitly written as std::span{it, (sz|it)}
//
// Note that both functions take universal (forwarding) references
// Also note that to wrap functions wanting a generic span of const values, the value type will have to be explicitly specifiedin the conversion of the first wrapper function
// Also note that the first wrapper function could be func(const spanifiable auto&&) instead if desired (to prevent mutating the contents)


namespace movency {

template<class T>
concept specialization_of_span =    std::same_as<T, std::span<typename T::element_type, T::extent>>
                                 && !std::is_reference_v<T>;

template<class T>
concept spanifiable =    std::convertible_to<std::remove_reference_t<T>, std::span<const typename std::remove_reference_t<T>::value_type>>
                      && std::is_lvalue_reference_v<T>
                      && !specialization_of_span<T>;

} // namespace movency
