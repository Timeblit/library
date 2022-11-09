#pragma once

#include "span_interface.hpp"
#include "all_types.hpp"

#include <fmt/format.h>
#include <fmt/compile.h>
#include <fmt/color.h>

#include <cstdint>
#include <random>
#include <cstring>
#include <span>

#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
  #include <emscripten/html5.h>
  #include <emscripten/bind.h>
#else
  #include <sys/random.h>
#endif

// This file provides various utilities to aid (psuedo)random number generation
//
// The main use is easy generation of cryptographically-secure random numbers/data with functions random::crypto and random::crypto_fill, and of pseudorandom numbers/data with functions random::fast and random::fast_fill.
// Examples of their usage is given in random_test.cpp
//
// random::fast<T>() and random::crypto<T>() generate random values uniformly distributed over the range of T
// random::fast() and random::crypto() can also be used without specifying a type, and will automatically generate a value from the range of the type their result is converted to
// random::fast(distribution) and random::crypto(distribution) will use the provided distribution
// All of the above should also work on contiguous containers except for random::fast/crypto on types that are dynamically sized or non-owning
//
// random::fast_fill(auto) and random::crypto_fill(auto) can be used to fill an existing object with random data
// a distribution can also be passed into either as an optional second parameter
// if no distribution is given, they will use uniform distributions for recognized types ([u]ints, floats, and bool) and containers of recognized types, else they will randomize the bits of the memory of the object
//
// Uniform distributions of custom types can be registered (and will then be used by default for randomizing that type)
// To do so, specialize movency::random::uniform_distribution_implementation<Type> for the type, ensuring it satisfies the distrubution concept
// The distribution can then be used as movency::random::uniform_distribution<Type>
// Note that publicly inheriting from movency::random::bit_distribution will allow the type to be bit-randomized by default
//
// Other useful things defined in this file (in namespace ::movency::random):
//
// bit_distribution struct, a distribution which indicates the binary representation of an object should be randomized
//
// prng_ and rng_ objects, thread-safe (pseudo)random number generators (eg for use in std::shuffle)
//
// seed(auto) and randomize_seed() functions, to seed prng_
//
// uniform_full_real_distribution<floating T> structs, which produce uniformly distributed random numbers over the full range of T
//
// uniform_distribution<class T> structs (with a deduction guide), which act like std::(bernoulli|uniform(_int|_real))_distribution for arithmetic<T>, and like a user-defined distribution for custom types which have been registered
//
// uniform_full_distribution<arithmetic T> structs, which act like uniform_distributions which can only be default constructed and cover the full range of T
//
// accumulating_distribution, which takes a distribution, a value, and an optional operation. It accumulates random values from the distribution into the value at each generation using the operation provided (or std::plus otherwise)


namespace movency {

namespace random {

template<class T>
struct uniform_distribution_implementation {};


template<class D>
concept distribution = requires(std::remove_reference_t<D> d, typename std::remove_reference_t<D>::result_type a)
{
  { d(a)      } -> std::same_as<typename std::remove_reference_t<D>::result_type>;
  { d.min()   } -> std::same_as<typename std::remove_reference_t<D>::result_type>;
  { d.max()   } -> std::same_as<typename std::remove_reference_t<D>::result_type>;
  { d.reset() } -> std::same_as<void>;
  { d.param() }; // note no input param option and no restrictions on return of .param() TODO?: change, or remove???
};


template<class T>
  requires distribution<uniform_distribution_implementation<T>>
using uniform_distribution = uniform_distribution_implementation<T>;


template<class T>
concept trivially_randomizable = std::is_trivial_v<T> && std::is_standard_layout_v<T> && !container<T> && !std::is_pointer_v<T>;

template<class T>
concept base_randomizable = trivially_randomizable<T> || distribution<uniform_distribution<T>>;

template<class T>
concept randomizable = base_randomizable<T> || (container<T> && base_randomizable<typename T::value_type>);

template<class T>
concept specialization_of_vector = std::same_as<T, std::vector<typename T::value_type, typename T::allocator_type>>;

template<class T>
concept randomizable_fixed_size = randomizable<T> && !specialization_of_span<T> && !specialization_of_vector<T>; //TODO:check if statically size

template<class D>
concept maybe_distribution = distribution<D> || std::same_as<D, unspecified_tag>;

// TODO: generator concept?

static_assert(randomizable<int32>);
static_assert(randomizable<std::span<int32, 6>>);
static_assert(randomizable<std::array<int32, 6>>);
static_assert(randomizable<std::vector<int32>>);
static_assert(!randomizable<std::vector<std::vector<int32>>>);


// This distribution to be used to indicate that an objects data should be filled with random bits
struct bit_distribution
{
  using result_type = uint32;

  static constexpr result_type min()   noexcept { return 0; }

  static constexpr result_type max()   noexcept { return std::numeric_limits<result_type>::max(); }

  static constexpr void        reset() noexcept { }

  static constexpr void        param() noexcept { }

  inline /*TODO: static*/ result_type operator()(auto a) const noexcept 
  {
    static_assert(std::same_as<typename decltype(a)::result_type, result_type>);

    return a();
  }
};

static_assert(distribution<bit_distribution>);


[[gnu::always_inline]] inline void crypto_fill(spanifiable auto&& container) noexcept;
[[gnu::always_inline]] inline void crypto_fill(base_randomizable auto& val) noexcept;


// Create a seed sequence with enough seeds to fully initialize a std::mt19937
std::seed_seq generate_seeds() noexcept
{
  std::array<std::mt19937::result_type, std::mt19937::state_size> seeds;

  crypto_fill(seeds);

  return std::seed_seq(seeds.begin(), seeds.end());
}


thread_local std::mt19937 prng_ = []{ auto seeds = generate_seeds(); return std::mt19937{seeds}; }();

struct
{
  using result_type = uint32;

  static constexpr result_type min()             noexcept { return 0; }

  static constexpr result_type max()             noexcept { return std::numeric_limits<result_type>::max(); }

  static constexpr void        seed(auto a)      noexcept { }

         constexpr void        discard(uint64 u) noexcept { for (uint64 i = 0; i < u; ++i) (*this)(); }

  /*TODO: static*/ result_type operator() () const noexcept 
  {
    result_type out;

    crypto_fill(out);

    return out;
  }
}
rng_;


namespace detail {

enum class Source { Crypto = 0, Fast = 1 };


template<Source source>
inline auto& get_generator() noexcept
{
  if constexpr (source == Source::Crypto)
    return rng_;
  else
    return prng_;
};


// Forward declarations

template<Source source, distribution D, class T = unspecified_tag>
auto rand(D&& dist) noexcept;

template<Source source, randomizable_fixed_size T>
T rand() noexcept;

} // namespace detail


// distribution covering the full range of floating type
// necessary as attempting to create this using a single std::uinform_real_distribution just yields infs (eg for doubles)
template<floating T>
struct uniform_full_real_distribution
{
  using result_type = T;

  static constexpr result_type min()   noexcept { return std::numeric_limits<result_type>::lowest(); }

  static constexpr result_type max()   noexcept { return std::numeric_limits<result_type>::max(); }

         constexpr void        reset() noexcept { internal_low.reset(); internal_high.reset(); }

  static constexpr void        param() noexcept { }

  template<class G>
  result_type operator()(G&& gen) noexcept
  {
    constexpr detail::Source source = std::same_as<std::decay_t<G>, std::mt19937> ? detail::Source::Fast : detail::Source::Crypto;

    if (detail::rand<source, bool>())
      return internal_low(std::forward<G>(gen));
    else
      return internal_high(std::forward<G>(gen));
  }

private:

  std::uniform_real_distribution<T> internal_low  = std::uniform_real_distribution<T>(std::numeric_limits<T>::lowest(), 0);
  std::uniform_real_distribution<T> internal_high = std::uniform_real_distribution<T>(0,    std::numeric_limits<T>::max());
};

static_assert(distribution<uniform_full_real_distribution<float64>>);



template<>
struct uniform_distribution_implementation<std::byte> : public bit_distribution { };

template<>
struct uniform_distribution_implementation<bool> : public std::bernoulli_distribution
{
  using std::bernoulli_distribution::bernoulli_distribution;
};

template<integral T>
struct uniform_distribution_implementation<T> : public std::uniform_int_distribution<T>
{
  using std::uniform_int_distribution<T>::uniform_int_distribution;
};

template<floating T>
struct uniform_distribution_implementation<T> : public std::uniform_real_distribution<T>
{
  using std::uniform_real_distribution<T>::uniform_real_distribution;
};

template<integral T>
  requires (sizeof(T) == 1)
struct uniform_distribution_implementation<T>
{
public:

  using result_type = T;

  constexpr uniform_distribution_implementation() noexcept
    : uniform_distribution_implementation(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::max()) {}

  constexpr uniform_distribution_implementation(T a, T b) noexcept
    : internal_distribution(a, b) {}

  template<class G>
  constexpr result_type operator() (G& gen) noexcept
  {
    return static_cast<T>(internal_distribution(gen));
  }

  void reset()
  {
    internal_distribution.reset();
  }

  result_type min() const noexcept
  {
    return static_cast<T>(internal_distribution.min());
  }

  result_type max() const noexcept
  {
    return static_cast<T>(internal_distribution.max());
  }

  auto param() const noexcept
  {
    return internal_distribution.param();
  }

private:

  std::uniform_int_distribution<std::conditional_t<signed_integral<T>, int16, uint16>> internal_distribution;
};


// deduction guide
template<base_randomizable T>
uniform_distribution_implementation(T, T) -> uniform_distribution_implementation<T>;


static_assert(distribution<uniform_distribution<uint32>>);
static_assert(distribution<uniform_distribution<int32>>);
static_assert(distribution<uniform_distribution<float32>>);
static_assert(distribution<uniform_distribution<bool>>);
static_assert(distribution<uniform_distribution<std::byte>>);


namespace detail {

template<class T>
struct internal_uniform_full_distribution
{ using type = void; };

template<class T>
  requires (distribution<uniform_distribution<T>>)
struct internal_uniform_full_distribution<T>
{ using type = uniform_distribution<T>; };

template<floating T>
  requires (distribution<uniform_distribution<T>>)
struct internal_uniform_full_distribution<T>
{ using type = uniform_full_real_distribution<T>; };

template<class T>
using internal_uniform_full_distribution_t = typename internal_uniform_full_distribution<T>::type;

} // namespace detail


template<class T>
  requires(distribution<detail::internal_uniform_full_distribution_t<T>>)
struct uniform_full_distribution : public detail::internal_uniform_full_distribution_t<T>
{
  using result_type = typename detail::internal_uniform_full_distribution_t<T>::result_type;

  uniform_full_distribution() = default;
};

static_assert(distribution<uniform_full_distribution<uint32>>);
static_assert(distribution<uniform_full_distribution<int32>>);
static_assert(distribution<uniform_full_distribution<float32>>);
static_assert(distribution<uniform_full_distribution<bool>>);


constexpr void seed(auto seeds) noexcept
{
  prng_.seed(seeds);
}

void randomize_seed()
{
  auto seeds = generate_seeds();

  prng_.seed(seeds);
}


#ifdef __EMSCRIPTEN__

EM_JS(int, getrandom, (std::byte* p, int len, int flag),
{
  for (let sum = 0; sum < len; )
  {
    let todo = Math.min(len - sum, 65536);

    let arr = new Uint8Array(HEAP8.buffer, p + sum, todo);

    self.crypto.getRandomValues(arr);

    sum += todo;
  }

  return len;
});

#endif



namespace detail {

// Fill the span with random bits

template<Source source, std::size_t Extent>
inline void raw_fill(const std::span<std::byte, Extent> buf)
{
  if constexpr (source == Source::Crypto)
  {
    for (auto it = buf.begin(); it != buf.end(); )
    {
      auto result = getrandom(&(*it), buf.end() - it, 0);

      if (result == -1)
      {
        fmt::print(fmt::emphasis::bold | fg(fmt::color::red), "getrandom failed, returned {}", result);

        continue;
      }

      it += result;
    }
  }
  else
  {
    auto it = buf.begin();

    //while (it <= buf.end() - 4) // ERROR: presumably iterators use unsigned?
    while (std::distance(it, buf.end()) >= 4)
    {
      uint32 r = static_cast<uint32>(prng_());

      std::memcpy(&(*it), &r, 4);

      it += 4;
    }

    if (it != buf.end())
    {
      uint32 r = static_cast<uint32>(prng_());

      std::memcpy(&(*it), &r, std::distance(it, buf.end()));
    }
  }
}


template<Source source, base_randomizable T, std::size_t Extent>
inline void internal_fill(const std::span<T, Extent> buf)
{
  if constexpr (!requires{uniform_full_distribution<T>{};}) //TODO: is this UB?????
  {
    static_assert(requires{uniform_full_distribution<T>{};}, "Cannot default randomize type with no uniform_distribution. Either pass a distribution or specialize movency::random::uniform_distribution_implementation (and, to allow bit randomization, publicly inherit from movency::random::bit_distribution)");
  }
  else if constexpr ((integral<T> && !std::same_as<T, bool>) || std::derived_from<uniform_full_distribution<T>, bit_distribution>)
  {
    raw_fill<source>(std::as_writable_bytes(buf));
  }
  else
  {
    static uniform_full_distribution<T> dist{};

    for (auto& val : buf)
      val = dist(get_generator<source>());
  }
}

template<Source source, base_randomizable T, distribution D, std::size_t Extent>
inline void internal_fill(const std::span<T, Extent> buf, D&& dist)
{
  if constexpr (std::derived_from<std::remove_reference_t<D>, bit_distribution>)
  {
    raw_fill<source>(std::as_writable_bytes(buf));
  }
  else
  {
    for (auto& val : buf)
      val = dist(get_generator<source>());
  }
}


template<Source source, base_randomizable T, distribution D>
[[gnu::always_inline]] inline void fill(T& val, D&& dist) noexcept
{
  return internal_fill<source>(std::span<T, 1>{&val, 1}, std::forward<D>(dist));
}

template<Source source, base_randomizable T>
[[gnu::always_inline]] inline void fill(T& val) noexcept
{
  return internal_fill<source>(std::span<T, 1>{&val, 1});
}

template<Source source, distribution D>
[[gnu::always_inline]] inline void fill(spanifiable auto&& container, D&& dist) noexcept
{
  return internal_fill<source>(std::span{container}, std::forward<D>(dist));
}

template<Source source>
[[gnu::always_inline]] inline void fill(spanifiable auto&& container) noexcept
{
  return internal_fill<source>(std::span{container});
}

template<Source source, distribution D>
[[gnu::always_inline]] inline void fill(const specialization_of_span auto&& sp, D&& dist) noexcept
{
  return internal_fill<source>(sp, std::forward<D>(dist));
}

template<Source source>
[[gnu::always_inline]] inline void fill(const specialization_of_span auto&& sp) noexcept
{
  return internal_fill<source>(sp);
}

} // namespace detail


// crypto_fill and fast_fill public interface

template<distribution D>
[[gnu::always_inline]] inline void crypto_fill(base_randomizable auto& val, D&& dist) noexcept
{
  return detail::fill<detail::Source::Crypto>(val, std::forward<D>(dist));
}

[[gnu::always_inline]] inline void crypto_fill(base_randomizable auto& val) noexcept
{
  return detail::fill<detail::Source::Crypto>(val);
}

template<distribution D>
[[gnu::always_inline]] inline void crypto_fill(spanifiable auto&& container, D&& dist) noexcept
{
  return detail::fill<detail::Source::Crypto>(container, std::forward<D>(dist));
}

[[gnu::always_inline]] inline void crypto_fill(spanifiable auto&& container) noexcept
{
  return detail::fill<detail::Source::Crypto>(container);
}
template<distribution D>
[[gnu::always_inline]] inline void crypto_fill(const specialization_of_span auto&& sp, D&& dist) noexcept
{
  return detail::fill<detail::Source::Crypto>(sp, std::forward<D>(dist));
}

[[gnu::always_inline]] inline void crypto_fill(const specialization_of_span auto&& sp) noexcept
{
  return detail::fill<detail::Source::Crypto>(sp);
}


template<distribution D>
[[gnu::always_inline]] inline void fast_fill(base_randomizable auto& val, D&& dist) noexcept
{
  return detail::fill<detail::Source::Fast>(val, std::forward<D>(dist));
}

[[gnu::always_inline]] inline void fast_fill(base_randomizable auto& val) noexcept
{
  return detail::fill<detail::Source::Fast>(val);
}

template<distribution D>
[[gnu::always_inline]] inline void fast_fill(spanifiable auto&& container, D&& dist) noexcept
{
  return detail::fill<detail::Source::Fast>(container, std::forward<D>(dist));
}

[[gnu::always_inline]] inline void fast_fill(spanifiable auto&& container) noexcept
{
  return detail::fill<detail::Source::Fast>(container);
}

template<distribution D>
[[gnu::always_inline]] inline void fast_fill(const specialization_of_span auto&& sp, D&& dist) noexcept
{
  return detail::fill<detail::Source::Fast>(sp, std::forward<D>(dist));
}

[[gnu::always_inline]] inline void fast_fill(const specialization_of_span auto&& sp) noexcept
{
  return detail::fill<detail::Source::Fast>(sp);
}



namespace detail{

// forward declaration

template<Source source, maybe_distribution D = unspecified_tag>
struct generic_random_result;


// return a generic_random_result object which can be implicitly converted to any randomizable (fixed-size) type

template<Source source>
inline generic_random_result<source> rand() noexcept
{
  return generic_random_result<source>();
}


// return random value from a uniform distribution over the range of values representable by the requested type

template<Source source, randomizable_fixed_size T>
[[gnu::always_inline]] inline T rand() noexcept
{
  T out;

  detail::fill<source>(out);

  return out;
}


// return (an object convertible to) a random value or container of values from a given distribution

template<Source source, distribution D, class T>
inline auto rand(D&& dist) noexcept
{
  if constexpr (std::same_as<T, unspecified_tag>)
  {
    return generic_random_result<source, D>(std::forward<D>(dist));
  }
  else
  {
    T out;

    detail::fill<source>(out, std::forward<D>(dist));

    return out;
  }
}


// object implicitly convertible to any randomizable type

template<Source source>
struct generic_random_result<source, unspecified_tag>
{
  // Convert to a randomizable by calling rand, which generates a random value of the desired type
  template<randomizable_fixed_size T>
  inline operator T() const noexcept
  {
    return detail::rand<source, T>();
  }
};

template<Source source, maybe_distribution Din>
struct generic_random_result
{
  static_assert(distribution<Din>, "Argument passed to random number generator function must be a distribution object");

  using D = std::remove_reference_t<Din>;

  generic_random_result(D&& d) noexcept
    : dist(d)
  { }

  generic_random_result(D& d) noexcept
    : dist(d)
  { }

  // Convert to a randomizable type by calling rand, which generates a random value of the desired type
  // Conditional explicit ensures casts are not implicitly performed within the random generation 
  // TODO: ? Potentially this could be changed to prevent otherwise explicit casts from happening, while allowing otherwise implicit ones?
  template<randomizable_fixed_size T>
  explicit ( !(std::same_as<D, bit_distribution> || container_of<T, typename D::result_type>) )
  inline operator T() const noexcept
  {
    return detail::rand<source, D, T>(std::forward<D>(dist));
  }

  // Convert to the result_type of ths stored distribution
  inline operator typename D::result_type() const noexcept
  {
    return detail::rand<source, D, typename D::result_type>(std::forward<D>(dist));
  }

  D& dist;
};

} // namespace detail


[[gnu::always_inline]] inline auto crypto() noexcept
{
  return detail::rand<detail::Source::Crypto>();
}

template<randomizable_fixed_size T>
[[gnu::always_inline]] inline auto crypto() noexcept
{
  return detail::rand<detail::Source::Crypto, T>();
}

template<distribution D>
[[gnu::always_inline]] inline auto crypto(D&& dist) noexcept
{
  return detail::rand<detail::Source::Crypto>(std::forward<D>(dist));
}

template<randomizable_fixed_size T, distribution D>
[[gnu::always_inline]] inline auto crypto(D&& dist) noexcept
{
  return detail::rand<detail::Source::Crypto, D, T>(std::forward<D>(dist));
}


[[gnu::always_inline]] inline auto fast() noexcept
{
  return detail::rand<detail::Source::Fast>();
}

template<randomizable_fixed_size T>
[[gnu::always_inline]] inline auto fast() noexcept
{
  return detail::rand<detail::Source::Fast, T>();
}

template<distribution D>
[[gnu::always_inline]] inline auto fast(D&& dist) noexcept
{
  return detail::rand<detail::Source::Fast>(std::forward<D>(dist));
}

template<randomizable_fixed_size T, distribution D>
[[gnu::always_inline]] inline auto fast(D&& dist) noexcept
{
  return detail::rand<detail::Source::Fast, D, T>(std::forward<D>(dist));
}


template<distribution D, class T, class Op = std::plus<>>
class accumulating_distribution
{
public:

  using result_type = T;

  constexpr accumulating_distribution(D din, T init, Op operation = Op{}) noexcept
    : initial(init), val(init), dist(din), op(operation) { }

  constexpr void reset()
  {
    dist.reset();

    val = initial;
  }

  template<class G>
  constexpr result_type operator() (G& gen) noexcept
  {
    return val = static_cast<T>(op(val, dist(gen)));
  }

  constexpr result_type min() const noexcept
  {
    return std::numeric_limits<T>::lowest();
  }

  constexpr result_type max() const noexcept
  {
    return std::numeric_limits<T>::max();
  }

  constexpr auto param() const noexcept
  {
    return std::tuple{initial, val, dist.param()};
  }

private:

  const T initial;

  T val;

  D dist;

  Op op;
};


} // namespace random

} // namespace movency
