#pragma once

#include <tuple>
#include <iterator>
#include <type_traits>

namespace tobas_std
{
struct get_iterator_or_pointer_impl
{
  template <typename T>
  static decltype(std::begin(T())) get(T*);

  template <typename T, std::enable_if_t<std::is_pointer<T>::value, std::nullptr_t> = nullptr>
  static T get(T);

  template <typename T, std::enable_if_t<std::is_array<T>::value, std::nullptr_t> = nullptr>
  static typename std::add_pointer<typename std::remove_extent<T>::type>::type get(T);
};

template <typename T>
struct get_iterator_or_pointer
{
public:
  using type = decltype(get_iterator_or_pointer_impl::template get<T>(nullptr));
};

struct get_value_type_impl
{
  template <typename T>
  static typename T::value_type get(typename T::value_type*);

  template <typename T, std::enable_if_t<std::is_pointer<T>::value, std::nullptr_t> = nullptr>
  static typename std::remove_pointer<T>::type get(T);

  template <typename T, std::enable_if_t<std::is_array<T>::value, std::nullptr_t> = nullptr>
  static typename std::remove_extent<T>::type get(T);
};

template <typename T>
struct get_value_type
{
public:
  using type = decltype(get_value_type_impl::template get<T>(nullptr));
};

template <typename... Iterators>
class zip_iterator : std::iterator<std::forward_iterator_tag, std::tuple<typename get_value_type<Iterators>::type...>>
{
  // Index sequence for accessing elements in tuples.
  using indices = std::make_index_sequence<sizeof...(Iterators)>;

  // Tuple of types in iterators.
  using value_type = std::tuple<typename get_value_type<Iterators>::type...>;
  using iterators_tuple = std::tuple<Iterators...>;  // Tuple of iterators.
  using this_type = zip_iterator<Iterators...>;      // Self type.

public:
  zip_iterator(Iterators... iters) : p_({ iters... })
  {
  }

  // Increment and decrement for iterator implement.
  zip_iterator& operator++()
  {
    apply([](auto& it) { ++it; });
    return *this;
  }

  zip_iterator operator++(int)
  {
    auto ret = *this;
    apply([](auto& it) { ++it; });
    return *ret;
  }

  zip_iterator& operator--()
  {
    apply([](auto& it) { --it; });
    return *this;
  }

  // For dereferencing iterator.
  constexpr auto operator*() const
  {
    return dereference_elements();
  }

  // For range base for.
  bool operator!=(const this_type& rhs) const
  {
    return all_cmp([](auto&& l, auto&& r) { return l != r; }, rhs);
  }

  bool operator==(const this_type& rhs) const
  {
    return all_cmp([](auto&& l, auto&& r) { return l == r; }, rhs);
  }

private:
  // Invoke function on elements of iterators.
  template <typename Fnc>
  constexpr void apply(Fnc fnc)
  {
    apply_impl(fnc, indices());
  }

  template <typename Fnc, std::size_t... I>
  constexpr void apply_impl(Fnc fnc, std::index_sequence<I...>)
  {
    using swallow = std::initializer_list<int>;  // Swallow idiom...
    (void)swallow{ (fnc(std::get<I>(p_)), 0)... };
  };

  // The all_cmp returns true when all of the comparison functions return true. Using for "=="
  template <typename Fnc>
  bool all_cmp(Fnc fnc, const this_type& rhs) const
  {
    return this_type::all_cmp_impl(fnc, *this, rhs);
  }

  template <
    std::size_t n = 0,
    typename Fnc,
    std::enable_if_t<n<(sizeof...(Iterators)), std::nullptr_t> = nullptr> static bool
      all_cmp_impl(Fnc fnc, const this_type& lhs, const this_type& rhs)
  {
    return fnc(std::get<n>(lhs.dereference()), std::get<n>(rhs.dereference()))
           && this_type::all_cmp_impl<n + 1>(fnc, lhs, rhs);
  };

  template <std::size_t n, typename Fnc, std::enable_if_t<n == (sizeof...(Iterators)), std::nullptr_t> = nullptr>
  static bool all_cmp_impl(Fnc, const this_type&, const this_type&)
  {
    return true;
  };

  // The all_cmp returns true when one of the comparison functions return true.
  template <typename Fnc>
  bool any_cmp(Fnc fnc, const this_type& rhs) const
  {
    return this_type::any_cmp_impl(fnc, *this, rhs);
  }

  template <
    std::size_t n = 0,
    typename Fnc,
    std::enable_if_t<n<(sizeof...(Iterators)), std::nullptr_t> = nullptr> static bool
      any_cmp_impl(Fnc fnc, const this_type& lhs, const this_type& rhs)
  {
    return fnc(std::get<n>(lhs.dereference()), std::get<n>(rhs.dereference()))
           || this_type::all_cmp_impl<n + 1>(fnc, lhs, rhs);
  };

  template <std::size_t n, typename Fnc, std::enable_if_t<n == (sizeof...(Iterators)), std::nullptr_t> = nullptr>
  static bool any_cmp_impl(Fnc, const this_type&, const this_type&)
  {
    return false;
  };

  // Dereference tuple.
  constexpr auto dereference() const
  {
    return dereference_impl(indices());
  }

  template <typename std::size_t... I>
  constexpr auto dereference_impl(std::index_sequence<I...>) const
  {
    return std::tie(std::get<I>(p_)...);
  }

  // Dereference iterators in the tuple.
  constexpr auto dereference_elements() const
  {
    return dereference_elements_impl(indices());
  }

  template <typename std::size_t... I>
  constexpr auto dereference_elements_impl(std::index_sequence<I...>) const
  {
    return std::tie(*std::get<I>(p_)...);
  }

  iterators_tuple p_;
};

template <typename... Args>
class zip
{
public:
  using iterator = zip_iterator<typename get_iterator_or_pointer<Args>::type...>;
  zip(Args&... args) : begin_({ std::begin(args)... }), end_({ std::end(args)... }){};
  zip_iterator<typename get_iterator_or_pointer<Args>::type...> begin_;
  zip_iterator<typename get_iterator_or_pointer<Args>::type...> end_;

  auto& begin()
  {
    return begin_;
  }

  auto& end()
  {
    return end_;
  }
};
}  // namespace tobas_std
