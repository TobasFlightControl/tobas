// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/utilities/rall2d.h
// Modified by Tobas, Inc.

/*****************************************************************************
 * \file
 *      class for automatic differentiation on scalar values and 1st
 *      derivatives and 2nd derivative.
 *
 *  \author
 *      Erwin Aertbelien, Div. PMA, Dep. of Mech. Eng., K.U.Leuven
 *
 *  \version
 *      ORO_Geometry V0.2
 *
 *  \par Note
 *      VC6++ contains a bug, concerning the use of inlined friend functions
 *      in combination with namespaces.  So, try to avoid inlined friend
 *      functions !
 *
 *  \par History
 *      - $log$
 *
 *  \par Release
 *      $Id: rall2d.h,v 1.1.1.1 2002/08/26 14:14:21 rmoreas Exp $
 *      $Name:  $
 ****************************************************************************/

#pragma once

#include <cmath>

#include "./utility.hpp"

namespace tobas::kdl
{
/**
 * Rall2d contains a value, and its gradient and its 2nd derivative, and defines an algebraic
 * structure on this pair.
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
template <class T, class V = T, class S = T>
class Rall2d
{
public:
  T t;   ///< value
  V d;   ///< 1st derivative
  V dd;  ///< 2nd derivative

  // = Constructors
  inline explicit Rall2d() : t(), d(), dd() {};

  inline explicit Rall2d(typename TI<T>::Arg _c)
  {
    t = _c;
    setToZero(d);
    setToZero(dd);
  }

  inline explicit Rall2d(typename TI<T>::Arg _tn, const V& _afg) : t(_tn), d(_afg)
  {
    setToZero(dd);
  }

  inline explicit Rall2d(typename TI<T>::Arg _tn, const V& _afg, const V& _afg2) : t(_tn), d(_afg), dd(_afg2)
  {
  }

  inline Rall2d(const Rall2d<T, V, S>& _r) : t(_r.t), d(_r.d), dd(_r.dd)
  {
  }

  inline T& Value()
  {
    return t;
  }

  inline V& D()
  {
    return d;
  }

  inline V& DD()
  {
    return dd;
  }

  static inline Rall2d<T, V, S> Zero()
  {
    Rall2d<T, V, S> tmp;
    setToZero(tmp);
    return tmp;
  }

  static inline Rall2d<T, V, S> Identity()
  {
    Rall2d<T, V, S> tmp;
    setToIdentity(tmp);
    return tmp;
  }

  // = assignment operators
  inline Rall2d<T, V, S>& operator=(S _c)
  {
    t = _c;
    setToZero(d);
    setToZero(dd);
    return *this;
  }

  inline Rall2d<T, V, S>& operator=(const Rall2d<T, V, S>& _r)
  {
    t = _r.t;
    d = _r.d;
    dd = _r.dd;
    return *this;
  }

  inline Rall2d<T, V, S>& operator/=(const Rall2d<T, V, S>& _rhs)
  {
    t /= _rhs.t;
    d = (d - t * _rhs.d) / _rhs.t;
    dd = (dd - S(2) * d * _rhs.d - t * _rhs.dd) / _rhs.t;
    return *this;
  }

  inline Rall2d<T, V, S>& operator*=(const Rall2d<T, V, S>& _rhs)
  {
    t *= _rhs.t;
    d = (d * _rhs.t + t * _rhs.d);
    dd = (dd * _rhs.t + S(2) * d * _rhs.d + t * _rhs.dd);
    return *this;
  }

  inline Rall2d<T, V, S>& operator+=(const Rall2d<T, V, S>& _rhs)
  {
    t += _rhs.t;
    d += _rhs.d;
    dd += _rhs.dd;
    return *this;
  }

  inline Rall2d<T, V, S>& operator-=(const Rall2d<T, V, S>& _rhs)
  {
    t -= _rhs.t;
    d -= _rhs.d;
    dd -= _rhs.dd;
    return *this;
  }

  inline Rall2d<T, V, S>& operator/=(S _rhs)
  {
    t /= _rhs;
    d /= _rhs;
    dd /= _rhs;
    return *this;
  }

  inline Rall2d<T, V, S>& operator*=(S _rhs)
  {
    t *= _rhs;
    d *= _rhs;
    dd *= _rhs;
    return *this;
  }

  inline Rall2d<T, V, S>& operator-=(S _rhs)
  {
    t -= _rhs;
    return *this;
  }

  inline Rall2d<T, V, S>& operator+=(S _rhs)
  {
    t += _rhs;
    return *this;
  }
};

using doubleAcc = Rall2d<double>;

// = Operators between Rall2d objects
template <class T, class V, class S>
inline Rall2d<T, V, S> operator/(const Rall2d<T, V, S>& _lhs, const Rall2d<T, V, S>& _rhs)
{
  Rall2d<T, V, S> tmp;
  tmp.t = _lhs.t / _rhs.t;
  tmp.d = (_lhs.d - tmp.t * _rhs.d) / _rhs.t;
  tmp.dd = (_lhs.dd - S(2) * tmp.d * _rhs.d - tmp.t * _rhs.dd) / _rhs.t;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator*(const Rall2d<T, V, S>& _lhs, const Rall2d<T, V, S>& _rhs)
{
  Rall2d<T, V, S> tmp;
  tmp.t = _lhs.t * _rhs.t;
  tmp.d = (_lhs.d * _rhs.t + _lhs.t * _rhs.d);
  tmp.dd = (_lhs.dd * _rhs.t + S(2) * _lhs.d * _rhs.d + _lhs.t * _rhs.dd);
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator+(const Rall2d<T, V, S>& _lhs, const Rall2d<T, V, S>& _rhs)
{
  return Rall2d<T, V, S>(_lhs.t + _rhs.t, _lhs.d + _rhs.d, _lhs.dd + _rhs.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator-(const Rall2d<T, V, S>& _lhs, const Rall2d<T, V, S>& _rhs)
{
  return Rall2d<T, V, S>(_lhs.t - _rhs.t, _lhs.d - _rhs.d, _lhs.dd - _rhs.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator-(const Rall2d<T, V, S>& _arg)
{
  return Rall2d<T, V, S>(-_arg.t, -_arg.d, -_arg.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator*(S _s, const Rall2d<T, V, S>& _v)
{
  return Rall2d<T, V, S>(_s * _v.t, _s * _v.d, _s * _v.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator*(const Rall2d<T, V, S>& _v, S _s)
{
  return Rall2d<T, V, S>(_v.t * _s, _v.d * _s, _v.dd * _s);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator+(S _s, const Rall2d<T, V, S>& _v)
{
  return Rall2d<T, V, S>(_s + _v.t, _v.d, _v.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator+(const Rall2d<T, V, S>& _v, S _s)
{
  return Rall2d<T, V, S>(_v.t + _s, _v.d, _v.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator-(S _s, const Rall2d<T, V, S>& _v)
{
  return Rall2d<T, V, S>(_s - _v.t, -_v.d, -_v.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator-(const Rall2d<T, V, S>& _v, S _s)
{
  return Rall2d<T, V, S>(_v.t - _s, _v.d, _v.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator/(S _s, const Rall2d<T, V, S>& _rhs)
{
  assert(_rhs.t != 0);
  Rall2d<T, V, S> tmp;
  tmp.t = _s / _rhs.t;
  tmp.d = (-tmp.t * _rhs.d) / _rhs.t;
  tmp.dd = (-S(2) * tmp.d * _rhs.d - tmp.t * _rhs.dd) / _rhs.t;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> operator/(const Rall2d<T, V, S>& _v, S _s)
{
  assert(_s != 0);
  return Rall2d<T, V, S>(_v.t / _s, _v.d / _s, _v.dd / _s);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> exp(const Rall2d<T, V, S>& _arg)
{
  Rall2d<T, V, S> tmp;
  tmp.t = exp(_arg.t);
  tmp.d = tmp.t * _arg.d;
  tmp.dd = tmp.d * _arg.d + tmp.t * _arg.dd;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> log(const Rall2d<T, V, S>& _arg)
{
  Rall2d<T, V, S> tmp;
  tmp.t = log(_arg.t);
  tmp.d = _arg.d / _arg.t;
  tmp.dd = (_arg.dd - tmp.d * _arg.d) / _arg.t;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> sin(const Rall2d<T, V, S>& _arg)
{
  T v1 = sin(_arg.t);
  T v2 = cos(_arg.t);
  return Rall2d<T, V, S>(v1, v2 * _arg.d, v2 * _arg.dd - (v1 * _arg.d) * _arg.d);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> cos(const Rall2d<T, V, S>& _arg)
{
  T v1 = cos(_arg.t);
  T v2 = -sin(_arg.t);
  return Rall2d<T, V, S>(v1, v2 * _arg.d, v2 * _arg.dd - (v1 * _arg.d) * _arg.d);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> tan(const Rall2d<T, V, S>& _arg)
{
  T v1 = tan(_arg.t);
  T v2 = S(1) + sqr(v1);
  return Rall2d<T, V, S>(v1, v2 * _arg.d, v2 * (_arg.dd + (S(2) * v1 * sqr(_arg.d))));
}

template <class T, class V, class S>
inline Rall2d<T, V, S> sinh(const Rall2d<T, V, S>& _arg)
{
  T v1 = sinh(_arg.t);
  T v2 = cosh(_arg.t);
  return Rall2d<T, V, S>(v1, v2 * _arg.d, v2 * _arg.dd + (v1 * _arg.d) * _arg.d);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> cosh(const Rall2d<T, V, S>& _arg)
{
  T v1 = cosh(_arg.t);
  T v2 = sinh(_arg.t);
  return Rall2d<T, V, S>(v1, v2 * _arg.d, v2 * _arg.dd + (v1 * _arg.d) * _arg.d);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> tanh(const Rall2d<T, V, S>& _arg)
{
  T v1 = tanh(_arg.t);
  T v2 = S(1) - sqr(v1);
  return Rall2d<T, V, S>(v1, v2 * _arg.d, v2 * (_arg.dd - (S(2) * v1 * sqr(_arg.d))));
}

template <class T, class V, class S>
inline Rall2d<T, V, S> sqr(const Rall2d<T, V, S>& _arg)
{
  return Rall2d<T, V, S>(_arg.t * _arg.t, (S(2) * _arg.t) * _arg.d, S(2) * (sqr(_arg.d) + _arg.t * _arg.dd));
}

template <class T, class V, class S>
inline Rall2d<T, V, S> pow(const Rall2d<T, V, S>& _arg, double _m)
{
  Rall2d<T, V, S> tmp;
  tmp.t = pow(_arg.t, _m);
  T v2 = (_m / _arg.t) * tmp.t;
  tmp.d = v2 * _arg.d;
  tmp.dd = (S((_m - 1)) / _arg.t) * tmp.d * _arg.d + v2 * _arg.dd;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> sqrt(const Rall2d<T, V, S>& _arg)
{
  // By inversion of sqr(x)
  Rall2d<T, V, S> tmp;
  tmp.t = sqrt(_arg.t);
  tmp.d = (S(0.5) / tmp.t) * _arg.d;
  tmp.dd = (S(0.5) * _arg.dd - sqr(tmp.d)) / tmp.t;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> asin(const Rall2d<T, V, S>& _arg)
{
  // By inversion of sin(x)
  Rall2d<T, V, S> tmp;
  tmp.t = asin(_arg.t);
  T v = cos(tmp.t);
  tmp.d = _arg.d / v;
  tmp.dd = (_arg.dd + _arg.t * sqr(tmp.d)) / v;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> acos(const Rall2d<T, V, S>& _arg)
{
  // By inversion of cos(x)
  Rall2d<T, V, S> tmp;
  tmp.t = acos(_arg.t);
  T v = -sin(tmp.t);
  tmp.d = _arg.d / v;
  tmp.dd = (_arg.dd + _arg.t * sqr(tmp.d)) / v;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> atan(const Rall2d<T, V, S>& _x)
{
  // By inversion of tan(x)
  Rall2d<T, V, S> tmp;
  tmp.t = atan(_x.t);
  T v = S(1) + sqr(_x.t);
  tmp.d = _x.d / v;
  tmp.dd = _x.dd / v - (S(2) * _x.t) * sqr(tmp.d);
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> atan2(const Rall2d<T, V, S>& _y, const Rall2d<T, V, S>& _x)
{
  Rall2d<T, V, S> tmp;
  tmp.t = atan2(_y.t, _x.t);
  T v = sqr(_y.t) + sqr(_x.t);
  tmp.d = (_x.t * _y.d - _x.d * _y.t) / v;
  tmp.dd = (_x.t * _y.dd - _x.dd * _y.t - S(2) * (_x.t * _x.d + _y.t * _y.d) * tmp.d) / v;
  return tmp;
}

template <class T, class V, class S>
inline Rall2d<T, V, S> abs(const Rall2d<T, V, S>& _x)
{
  T v(Sign(_x));
  return Rall2d<T, V, S>(v * _x, v * _x.d, v * _x.dd);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> hypot(const Rall2d<T, V, S>& _y, const Rall2d<T, V, S>& _x)
{
  Rall2d<T, V, S> tmp;
  tmp.t = hypot(_y.t, _x.t);
  tmp.d = (_x.t * _x.d + _y.t * _y.d) / tmp.t;
  tmp.dd = (sqr(_x.d) + _x.t * _x.dd + sqr(_y.d) + _y.t * _y.dd - sqr(tmp.d)) / tmp.t;
  return tmp;
}

template <class T, class V, class S>
inline S norm(const Rall2d<T, V, S>& _value)
{
  return norm(_value.t);
}

template <class T, class V, class S>
inline Rall2d<T, V, S> LinComb(S _alfa, const Rall2d<T, V, S>& _a, const T& _beta, const Rall2d<T, V, S>& _b)
{
  return Rall2d<T, V, S>(
    LinComb(_alfa, _a.t, _beta, _b.t), LinComb(_alfa, _a.d, _beta, _b.d), LinComb(_alfa, _a.dd, _beta, _b.dd));
}

template <class T, class V, class S>
inline void
LinCombR(S _alfa, const Rall2d<T, V, S>& _a, const T& _beta, const Rall2d<T, V, S>& _b, Rall2d<T, V, S>& _result)
{
  LinCombR(_alfa, _a.t, _beta, _b.t, _result.t);
  LinCombR(_alfa, _a.d, _beta, _b.d, _result.d);
  LinCombR(_alfa, _a.dd, _beta, _b.dd, _result.dd);
}

template <class T, class V, class S>
inline void setToZero(Rall2d<T, V, S>& _value)
{
  setToZero(_value.t);
  setToZero(_value.d);
  setToZero(_value.dd);
}

template <class T, class V, class S>
inline void setToIdentity(Rall2d<T, V, S>& _value)
{
  setToZero(_value.d);
  setToIdentity(_value.t);
  setToZero(_value.dd);
}
}  // namespace tobas::kdl
