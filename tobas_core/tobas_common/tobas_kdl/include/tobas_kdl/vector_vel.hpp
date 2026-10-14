// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/framevel.hpp
// Modified by Tobas, Inc.

/*****************************************************************************
 * \file
 *      This file contains the definition of classes for a
 *      Rall Algebra of (subset of) the classes defined in frames,
 *      i.e. classes that contain a pair (value,derivative) and define operations on that pair
 *      this classes are useful for automatic differentiation ( <-> symbolic diff , <-> numeric diff)
 *      Defines VectorVel, RotationVel, FrameVel.  Look at Frames.h for details on how to work
 *      with Frame objects.
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
 *      $Id: rframes.h,v 1.1.1.1 2002/08/26 14:14:21 rmoreas Exp $
 *      $Name:  $
 ****************************************************************************/

#pragma once

#include "./rotation.hpp"
#include "./utilities/rall1d.hpp"
#include "./vector.hpp"

namespace tobas::kdl
{
class VectorVel
{
public:
  Vector p;  ///< position vector
  Vector v;  ///< velocity vector

  inline explicit VectorVel();
  inline explicit VectorVel(const Vector& _p, const Vector& _v);
  inline explicit VectorVel(const Vector& _p);

  static inline VectorVel Zero();

  inline void setZero();
  inline void setNaN();

  inline doubleVel norm() const;

  inline VectorVel& operator+=(const VectorVel& _rhs);
  inline VectorVel& operator-=(const VectorVel& _rhs);

  inline friend VectorVel operator+(const VectorVel& _r1, const VectorVel& _r2);
  inline friend VectorVel operator-(const VectorVel& _r1, const VectorVel& _r2);
  inline friend VectorVel operator+(const Vector& _r1, const VectorVel& _r2);
  inline friend VectorVel operator-(const Vector& _r1, const VectorVel& _r2);
  inline friend VectorVel operator+(const VectorVel& _r1, const Vector& _r2);
  inline friend VectorVel operator-(const VectorVel& _r1, const Vector& _r2);
  inline friend VectorVel operator*(const VectorVel& _r1, const VectorVel& _r2);
  inline friend VectorVel operator*(const VectorVel& _r1, const Vector& _r2);
  inline friend VectorVel operator*(const Vector& _r1, const VectorVel& _r2);
  inline friend VectorVel operator*(const VectorVel& _r1, double _r2);
  inline friend VectorVel operator*(double _r1, const VectorVel& _r2);
  inline friend VectorVel operator*(const doubleVel& _r1, const VectorVel& _r2);
  inline friend VectorVel operator*(const VectorVel& _r2, const doubleVel& _r1);
  inline friend VectorVel operator*(const Rotation& _r, const VectorVel& _x);

  inline friend VectorVel operator/(const VectorVel& _r1, double _r2);
  inline friend VectorVel operator/(const VectorVel& _r2, const doubleVel& _r1);

  inline friend VectorVel operator-(const VectorVel& _r);
};

inline VectorVel::VectorVel()
{
}

inline VectorVel::VectorVel(const Vector& _p, const Vector& _v) : p(_p), v(_v)
{
}

inline VectorVel::VectorVel(const Vector& _p) : p(_p), v(Vector::Zero())
{
}

VectorVel VectorVel::Zero()
{
  return VectorVel(Vector::Zero(), Vector::Zero());
}

inline void VectorVel::setZero()
{
  p.setZero();
  v.setZero();
}

inline void VectorVel::setNaN()
{
  p.setNaN();
  v.setNaN();
}

inline doubleVel VectorVel::norm() const
{
  const auto n = p.norm();
  if (n < std::numeric_limits<double>::epsilon()) {
    return doubleVel(0, 0);
  }
  return doubleVel(n, p.dot(v) / n);
}

inline VectorVel& VectorVel::operator+=(const VectorVel& _rhs)
{
  p += _rhs.p;
  v += _rhs.v;
  return *this;
}

inline VectorVel& VectorVel::operator-=(const VectorVel& _rhs)
{
  p -= _rhs.p;
  v -= _rhs.v;
  return *this;
}

inline VectorVel operator+(const VectorVel& _r1, const VectorVel& _r2)
{
  return VectorVel(_r1.p + _r2.p, _r1.v + _r2.v);
}

inline VectorVel operator-(const VectorVel& _r1, const VectorVel& _r2)
{
  return VectorVel(_r1.p - _r2.p, _r1.v - _r2.v);
}

inline VectorVel operator+(const VectorVel& _r1, const Vector& _r2)
{
  return VectorVel(_r1.p + _r2, _r1.v);
}

inline VectorVel operator-(const VectorVel& _r1, const Vector& _r2)
{
  return VectorVel(_r1.p - _r2, _r1.v);
}

inline VectorVel operator+(const Vector& _r1, const VectorVel& _r2)
{
  return VectorVel(_r1 + _r2.p, _r2.v);
}

inline VectorVel operator-(const Vector& _r1, const VectorVel& _r2)
{
  return VectorVel(_r1 - _r2.p, -_r2.v);
}

inline VectorVel operator-(const VectorVel& _r)
{
  return VectorVel(-_r.p, -_r.v);
}

inline VectorVel operator*(const VectorVel& _r1, const VectorVel& _r2)
{
  return VectorVel(_r1.p * _r2.p, _r1.p * _r2.v + _r1.v * _r2.p);
}

inline VectorVel operator*(const VectorVel& _r1, const Vector& _r2)
{
  return VectorVel(_r1.p * _r2, _r1.v * _r2);
}

inline VectorVel operator*(const Vector& _r1, const VectorVel& _r2)
{
  return VectorVel(_r1 * _r2.p, _r1 * _r2.v);
}

inline VectorVel operator*(double _r1, const VectorVel& _r2)
{
  return VectorVel(_r1 * _r2.p, _r1 * _r2.v);
}

inline VectorVel operator*(const VectorVel& _r1, double _r2)
{
  return VectorVel(_r1.p * _r2, _r1.v * _r2);
}

inline VectorVel operator*(const doubleVel& _r1, const VectorVel& _r2)
{
  return VectorVel(_r1.t * _r2.p, _r1.t * _r2.v + _r1.grad * _r2.p);
}

inline VectorVel operator*(const VectorVel& _r2, const doubleVel& _r1)
{
  return VectorVel(_r1.t * _r2.p, _r1.t * _r2.v + _r1.grad * _r2.p);
}

inline VectorVel operator/(const VectorVel& _r1, double _r2)
{
  assert(_r2 != 0);
  return VectorVel(_r1.p / _r2, _r1.v / _r2);
}

inline VectorVel operator/(const VectorVel& _r2, const doubleVel& _r1)
{
  assert(_r1.t != 0);
  assert(_r1.grad != 0);
  return VectorVel(_r2.p / _r1.t, _r2.v / _r1.t - _r2.p * _r1.grad / _r1.t / _r1.t);
}

inline VectorVel operator*(const Rotation& _r, const VectorVel& _x)
{
  return VectorVel(_r * _x.p, _r * _x.v);
}
}  // namespace tobas::kdl
