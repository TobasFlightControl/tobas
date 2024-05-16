#pragma once

#include <cmath>

namespace KDL
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

inline double LinComb(double alfa, double a, double beta, double b)
{
  return alfa * a + beta * b;
}

inline void LinCombR(double alfa, double a, double beta, double b, double& result)
{
  result = alfa * a + beta * b;
}

// to uniformly set double, RNDouble,Vector,... objects to zero in template-classes
inline void setToZero(double& arg)
{
  arg = 0;
}

// to uniformly set double, RNDouble,Vector,... objects to the identity element in template-classes
inline void setToIdentity(double& arg)
{
  arg = 1;
}

inline double sign(double arg)
{
  return (arg < 0) ? (-1) : (1);
}

inline constexpr double sqr(double arg)
{
  return arg * arg;
}

inline double norm(double arg)
{
  return fabs((double)arg);
}

inline double diff(double a, double b, double dt)
{
  return (b - a) / dt;
}

inline double addDelta(double a, double da, double dt)
{
  return a + da * dt;
}
}  // namespace KDL
