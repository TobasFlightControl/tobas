// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/frameacc.hpp
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/frameacc.inl
// Modified by Tobas, Inc.

/*****************************************************************************
 * \file
 *      This file contains the definition of classes for a
 *      Rall Algebra of (subset of) the classes defined in frames,
 *      i.e. classes that contain a set (value,derivative,2nd derivative)
 *      and define operations on that set
 *      this classes are useful for automatic differentiation ( <-> symbolic diff ,
 *      <-> numeric diff).
 *      Defines VectorAcc, RotationAcc, FrameAcc, doubleAcc.
 *      Look at the corresponding classes Vector Rotation Frame Twist and
 *      Wrench for the semantics of the methods.
 *
 *      It also contains the 2nd derivative <-> RFrames.h
 *
 *  \author
 *      Erwin Aertbelien, Div. PMA, Dep. of Mech. Eng., K.U.Leuven
 *
 *  \version
 *      ORO_Geometry V0.2
 *
 *  \par History
 *      - $log$
 *
 *  \par Release
 *      $Id: rrframes.h,v 1.1.1.1 2002/08/26 14:14:21 rmoreas Exp $
 *      $Name:  $
 ****************************************************************************/

#pragma once

#include "./rotation.hpp"
#include "./utilities/rall2d.hpp"
#include "./vector.hpp"

namespace tobas::kdl
{
class VectorAcc
{
public:
  Vector p;   ///< position vector
  Vector v;   ///< velocity vector
  Vector dv;  ///< acceleration vector

  inline explicit VectorAcc();
  inline explicit VectorAcc(const Vector& _p);
  inline explicit VectorAcc(const Vector& _p, const Vector& _v);
  inline explicit VectorAcc(const Vector& _p, const Vector& _v, const Vector& _dv);

  static inline VectorAcc Zero();

  inline void setZero();
  inline void setNaN();

  inline doubleAcc norm() const;

  inline doubleAcc dot(const Vector& _rhs) const;
  inline doubleAcc dot(const VectorAcc& _rhs) const;

  inline VectorAcc& operator+=(const VectorAcc& _arg);
  inline VectorAcc& operator-=(const VectorAcc& _arg);

  inline friend VectorAcc operator+(const VectorAcc& _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator-(const VectorAcc& _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator+(const Vector& _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator-(const Vector& _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator+(const VectorAcc& _r1, const Vector& _r2);
  inline friend VectorAcc operator-(const VectorAcc& _r1, const Vector& _r2);
  /** cross prod. */
  inline friend VectorAcc operator*(const VectorAcc& _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator*(const VectorAcc& _r1, const Vector& _r2);
  inline friend VectorAcc operator*(const Vector& _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator*(const VectorAcc& _r1, double _r2);
  /** scalar mult. */
  inline friend VectorAcc operator*(double _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator*(const doubleAcc& _r1, const VectorAcc& _r2);
  inline friend VectorAcc operator*(const VectorAcc& _r2, const doubleAcc& _r1);
  inline friend VectorAcc operator*(const Rotation& _r, const VectorAcc& _x);
  inline friend VectorAcc operator/(const VectorAcc& _r1, double _r2);
  inline friend VectorAcc operator/(const VectorAcc& _r2, const doubleAcc& _r1);
  /** unary - */
  inline friend VectorAcc operator-(const VectorAcc& _r);
};

inline VectorAcc::VectorAcc()
{
}

inline VectorAcc::VectorAcc(const Vector& _p) : p(_p), v(Vector::Zero()), dv(Vector::Zero())
{
}

inline VectorAcc::VectorAcc(const Vector& _p, const Vector& _v) : p(_p), v(_v), dv(Vector::Zero())
{
}

inline VectorAcc::VectorAcc(const Vector& _p, const Vector& _v, const Vector& _dv) : p(_p), v(_v), dv(_dv)
{
}

inline VectorAcc VectorAcc::Zero()
{
  return VectorAcc(Vector::Zero(), Vector::Zero(), Vector::Zero());
}

inline void VectorAcc::setZero()
{
  p.setZero();
  v.setZero();
  dv.setZero();
}

inline void VectorAcc::setNaN()
{
  p.setNaN();
  v.setNaN();
  dv.setNaN();
}

inline doubleAcc VectorAcc::norm() const
{
  doubleAcc res;
  res.t = p.norm();
  res.d = p.dot(v) / res.t;
  res.dd = (p.dot(dv) + v.dot(v) - res.d * res.d) / res.t;
  return res;
}

inline doubleAcc VectorAcc::dot(const Vector& _rhs) const
{
  return doubleAcc(p.dot(_rhs), v.dot(_rhs), dv.dot(_rhs));
}

inline doubleAcc VectorAcc::dot(const VectorAcc& _rhs) const
{
  return doubleAcc(p.dot(_rhs.p), p.dot(_rhs.v) + v.dot(_rhs.p), p.dot(_rhs.dv) + 2 * v.dot(_rhs.v) + dv.dot(_rhs.p));
}

inline VectorAcc& VectorAcc::operator+=(const VectorAcc& _arg)
{
  p += _arg.p;
  v += _arg.v;
  dv += _arg.dv;
  return *this;
}
inline VectorAcc& VectorAcc::operator-=(const VectorAcc& _arg)
{
  p -= _arg.p;
  v -= _arg.v;
  dv -= _arg.dv;
  return *this;
}

inline VectorAcc operator+(const VectorAcc& _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1.p + _r2.p, _r1.v + _r2.v, _r1.dv + _r2.dv);
}

inline VectorAcc operator-(const VectorAcc& _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1.p - _r2.p, _r1.v - _r2.v, _r1.dv - _r2.dv);
}
inline VectorAcc operator+(const Vector& _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1 + _r2.p, _r2.v, _r2.dv);
}

inline VectorAcc operator-(const Vector& _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1 - _r2.p, -_r2.v, -_r2.dv);
}

inline VectorAcc operator+(const VectorAcc& _r1, const Vector& _r2)
{
  return VectorAcc(_r1.p + _r2, _r1.v, _r1.dv);
}

inline VectorAcc operator-(const VectorAcc& _r1, const Vector& _r2)
{
  return VectorAcc(_r1.p - _r2, _r1.v, _r1.dv);
}

inline VectorAcc operator-(const VectorAcc& _r)
{
  return VectorAcc(-_r.p, -_r.v, -_r.dv);
}

inline VectorAcc operator*(const VectorAcc& _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1.p * _r2.p, _r1.p * _r2.v + _r1.v * _r2.p, _r1.dv * _r2.p + 2 * _r1.v * _r2.v + _r1.p * _r2.dv);
}

inline VectorAcc operator*(const VectorAcc& _r1, const Vector& _r2)
{
  return VectorAcc(_r1.p * _r2, _r1.v * _r2, _r1.dv * _r2);
}

inline VectorAcc operator*(const Vector& _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1 * _r2.p, _r1 * _r2.v, _r1 * _r2.dv);
}

inline VectorAcc operator*(double _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1 * _r2.p, _r1 * _r2.v, _r1 * _r2.dv);
}

inline VectorAcc operator*(const VectorAcc& _r1, double _r2)
{
  return VectorAcc(_r1.p * _r2, _r1.v * _r2, _r1.dv * _r2);
}

inline VectorAcc operator*(const doubleAcc& _r1, const VectorAcc& _r2)
{
  return VectorAcc(_r1.t * _r2.p, _r1.t * _r2.v + _r1.d * _r2.p, _r1.t * _r2.dv + 2 * _r1.d * _r2.v + _r1.dd * _r2.p);
}

inline VectorAcc operator*(const VectorAcc& _r2, const doubleAcc& _r1)
{
  return VectorAcc(_r1.t * _r2.p, _r1.t * _r2.v + _r1.d * _r2.p, _r1.t * _r2.dv + 2 * _r1.d * _r2.v + _r1.dd * _r2.p);
}

inline VectorAcc operator*(const Rotation& _r, const VectorAcc& _x)
{
  return VectorAcc(_r * _x.p, _r * _x.v, _r * _x.dv);
}

inline VectorAcc operator/(const VectorAcc& _r1, double _r2)
{
  assert(_r2 != 0.0);
  return VectorAcc(_r1.p / _r2, _r1.v / _r2, _r1.dv / _r2);
}

inline VectorAcc operator/(const VectorAcc& _r2, const doubleAcc& _r1)
{
  return _r2 * (1.0 / _r1);
}
}  // namespace tobas::kdl
