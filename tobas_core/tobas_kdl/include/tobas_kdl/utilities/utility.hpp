#pragma once

#include <cmath>

namespace kdl
{
/**
 * Auxiliary class for argument types (Trait-template class )
 *
 * Is used to pass doubles by value, and arbitrary objects by const reference.
 * This is TWICE as fast (2 x less memory access) and avoids bugs in VC6++ concerning
 * the assignment of the result of intrinsic functions to const double&-typed variables,
 * and optimization on.
 */
template <class T>
class TI
{
public:
  typedef const T& Arg;  // Arg is used for passing the element to a function.
};

template <>
class TI<double>
{
public:
  typedef double Arg;
};

template <>
class TI<int>
{
public:
  typedef int Arg;
};

inline constexpr double sqr(double arg)
{
  return arg * arg;
}
}  // namespace kdl
