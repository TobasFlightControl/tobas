// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

#include "./utility.hpp"

namespace tobas
{
namespace kdl
{
/**
 * Rall1d contains a value, and its gradient, and defines an algebraic structure on this pair.
 *  This template class has 3 template parameters :
 *  -   T contains the type of the value.
 *  -   V contains the type of the gradient (can be a vector-like type).
 *  -   S defines a scalar type that can operate on Rall1d. This is the type that
 *      is used to give back values of norm() etc.
 *
 * S is useful when you recurse a Rall1d object into itself to create a 2nd, 3rd, 4th,..
 * derivatives. (e.g. Rall1d< Rall1d<double>, Rall1d<double>, double> ).
 *
 * S is always passed by value.
 */
template <typename T, typename V = T, typename S = T>
class Rall1d
{
public:
  T t;     ///< value
  V grad;  ///< gradient

  inline explicit Rall1d() : t(), grad()
  {
  }

  inline explicit Rall1d(typename TI<T>::Arg _c)
  {
    t = T(_c);
    setToZero(grad);
  }

  inline explicit Rall1d(typename TI<T>::Arg _tn, typename TI<V>::Arg _afg) : t(_tn), grad(_afg)
  {
  }

  inline Rall1d(const Rall1d<T, V, S>& _r) : t(_r.t), grad(_r.grad)
  {
  }

  static inline Rall1d<T, V, S> Zero()
  {
    Rall1d<T, V, S> tmp;
    setToZero(tmp);
    return tmp;
  }

  static inline Rall1d<T, V, S> Identity()
  {
    Rall1d<T, V, S> tmp;
    setToIdentity(tmp);
    return tmp;
  }

  inline Rall1d<T, V, S>& operator=(const Rall1d<T, V, S>& _r)
  {
    t = _r.t;
    grad = _r.grad;
    return *this;
  }

  inline Rall1d<T, V, S>& operator=(S _c)
  {
    t = _c;
    setToZero(grad);
    return *this;
  }

  inline Rall1d<T, V, S>& operator/=(const Rall1d<T, V, S>& _rhs)
  {
    grad = LinComb(_rhs.t, grad, -t, _rhs.grad) / (_rhs.t * _rhs.t);
    t /= _rhs.t;
    return *this;
  }

  inline Rall1d<T, V, S>& operator*=(const Rall1d<T, V, S>& _rhs)
  {
    LinCombR(_rhs.t, grad, t, _rhs.grad, grad);
    t *= _rhs.t;
    return *this;
  }

  inline Rall1d<T, V, S>& operator+=(const Rall1d<T, V, S>& _rhs)
  {
    grad += _rhs.grad;
    t += _rhs.t;
    return *this;
  }

  inline Rall1d<T, V, S>& operator-=(const Rall1d<T, V, S>& _rhs)
  {
    grad -= _rhs.grad;
    t -= _rhs.t;
    return *this;
  }

  inline Rall1d<T, V, S>& operator/=(S _rhs)
  {
    grad /= _rhs;
    t /= _rhs;
    return *this;
  }

  inline Rall1d<T, V, S>& operator*=(S _rhs)
  {
    grad *= _rhs;
    t *= _rhs;
    return *this;
  }

  inline Rall1d<T, V, S>& operator+=(S _rhs)
  {
    t += _rhs;
    return *this;
  }

  inline Rall1d<T, V, S>& operator-=(S _rhs)
  {
    t -= _rhs;
    return *this;
  }
};

using doubleVel = Rall1d<double>;

template <class T, class V, class S>
inline Rall1d<T, V, S> operator/(const Rall1d<T, V, S>& _lhs, const Rall1d<T, V, S>& _rhs)
{
  return Rall1d<T, V, S>(_lhs.t / _rhs.t, (_lhs.grad * _rhs.t - _lhs.t * _rhs.grad) / (_rhs.t * _rhs.t));
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator*(const Rall1d<T, V, S>& _lhs, const Rall1d<T, V, S>& _rhs)
{
  return Rall1d<T, V, S>(_lhs.t * _rhs.t, _rhs.t * _lhs.grad + _lhs.t * _rhs.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator+(const Rall1d<T, V, S>& _lhs, const Rall1d<T, V, S>& _rhs)
{
  return Rall1d<T, V, S>(_lhs.t + _rhs.t, _lhs.grad + _rhs.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator-(const Rall1d<T, V, S>& _lhs, const Rall1d<T, V, S>& _rhs)
{
  return Rall1d<T, V, S>(_lhs.t - _rhs.t, _lhs.grad - _rhs.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator-(const Rall1d<T, V, S>& _arg)
{
  return Rall1d<T, V, S>(-_arg.t, -_arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator*(S _s, const Rall1d<T, V, S>& _v)
{
  return Rall1d<T, V, S>(_s * _v.t, _s * _v.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator*(const Rall1d<T, V, S>& _v, S _s)
{
  return Rall1d<T, V, S>(_v.t * _s, _v.grad * _s);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator+(S _s, const Rall1d<T, V, S>& _v)
{
  return Rall1d<T, V, S>(_s + _v.t, _v.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator+(const Rall1d<T, V, S>& _v, S _s)
{
  return Rall1d<T, V, S>(_v.t + _s, _v.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator-(S _s, const Rall1d<T, V, S>& _v)
{
  return Rall1d<T, V, S>(_s - _v.t, -_v.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator-(const Rall1d<T, V, S>& _v, S _s)
{
  return Rall1d<T, V, S>(_v.t - _s, _v.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator/(S _s, const Rall1d<T, V, S>& _v)
{
  return Rall1d<T, V, S>(_s / _v.t, (-_s * _v.grad) / (_v.t * _v.t));
}

template <class T, class V, class S>
inline Rall1d<T, V, S> operator/(const Rall1d<T, V, S>& _v, S _s)
{
  return Rall1d<T, V, S>(_v.t / _s, _v.grad / _s);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> exp(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (exp(_arg.t));
  return Rall1d<T, V, S>(v, v * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> log(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (log(_arg.t));
  return Rall1d<T, V, S>(v, _arg.grad / _arg.t);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> sin(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (sin(_arg.t));
  return Rall1d<T, V, S>(v, cos(_arg.t) * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> cos(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (cos(_arg.t));
  return Rall1d<T, V, S>(v, -sin(_arg.t) * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> tan(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (tan(_arg.t));
  return Rall1d<T, V, S>(v, _arg.grad / sqr(cos(_arg.t)));
}

template <class T, class V, class S>
inline Rall1d<T, V, S> sinh(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (sinh(_arg.t));
  return Rall1d<T, V, S>(v, cosh(_arg.t) * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> cosh(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (cosh(_arg.t));
  return Rall1d<T, V, S>(v, sinh(_arg.t) * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> sqr(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = (_arg.t * _arg.t);
  return Rall1d<T, V, S>(v, (2.0 * _arg.t) * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> pow(const Rall1d<T, V, S>& _arg, double _m)
{
  T v;
  v = (pow(_arg.t, _m));
  return Rall1d<T, V, S>(v, (_m * v / _arg.t) * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> sqrt(const Rall1d<T, V, S>& _arg)
{
  T v;
  v = sqrt(_arg.t);
  return Rall1d<T, V, S>(v, (0.5 / v) * _arg.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> atan(const Rall1d<T, V, S>& _x)
{
  T v;
  v = (atan(_x.t));
  return Rall1d<T, V, S>(v, _x.grad / (1.0 + sqr(_x.t)));
}

template <class T, class V, class S>
inline Rall1d<T, V, S> hypot(const Rall1d<T, V, S>& _y, const Rall1d<T, V, S>& _x)
{
  T v;
  v = (hypot(_y.t, _x.t));
  return Rall1d<T, V, S>(v, (_x.t / v) * _x.grad + (_y.t / v) * _y.grad);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> asin(const Rall1d<T, V, S>& _x)
{
  T v;
  v = (asin(_x.t));
  return Rall1d<T, V, S>(v, _x.grad / std::sqrt(1.0 - sqr(_x.t)));
}

template <class T, class V, class S>
inline Rall1d<T, V, S> acos(const Rall1d<T, V, S>& _x)
{
  T v;
  v = (acos(_x.t));
  return Rall1d<T, V, S>(v, -_x.grad / std::sqrt(1.0 - sqr(_x.t)));
}

template <class T, class V, class S>
inline Rall1d<T, V, S> abs(const Rall1d<T, V, S>& _x)
{
  T v;
  v = (Sign(_x));
  return Rall1d<T, V, S>(v * _x, v * _x.grad);
}

template <class T, class V, class S>
inline S norm(const Rall1d<T, V, S>& _value)
{
  return norm(_value.t);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> tanh(const Rall1d<T, V, S>& _arg)
{
  T v(tanh(_arg.t));
  return Rall1d<T, V, S>(v, _arg.grad / sqr(cosh(_arg.t)));
}

template <class T, class V, class S>
inline Rall1d<T, V, S> atan2(const Rall1d<T, V, S>& _y, const Rall1d<T, V, S>& _x)
{
  T v(_x.t * _x.t + _y.t * _y.t);
  return Rall1d<T, V, S>(atan2(_y.t, _x.t), (_x.t * _y.grad - _y.t * _x.grad) / v);
}

template <class T, class V, class S>
inline Rall1d<T, V, S> LinComb(S _alfa, const Rall1d<T, V, S>& _a, const T& _beta, const Rall1d<T, V, S>& _b)
{
  return Rall1d<T, V, S>(LinComb(_alfa, _a.t, _beta, _b.t), LinComb(_alfa, _a.grad, _beta, _b.grad));
}

template <class T, class V, class S>
inline void
LinCombR(S _alfa, const Rall1d<T, V, S>& _a, const T& _beta, const Rall1d<T, V, S>& _b, Rall1d<T, V, S>& _result)
{
  LinCombR(_alfa, _a.t, _beta, _b.t, _result.t);
  LinCombR(_alfa, _a.grad, _beta, _b.grad, _result.grad);
}

template <class T, class V, class S>
inline void setToZero(Rall1d<T, V, S>& _value)
{
  setToZero(_value.grad);
  setToZero(_value.t);
}

template <class T, class V, class S>
inline void setToIdentity(Rall1d<T, V, S>& _value)
{
  setToIdentity(_value.t);
  setToZero(_value.grad);
}
}  // namespace kdl
}  // namespace tobas
