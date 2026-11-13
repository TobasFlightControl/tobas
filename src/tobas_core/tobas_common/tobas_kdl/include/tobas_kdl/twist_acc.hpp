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

#include "./accel.hpp"
#include "./twist.hpp"
#include "./vector_acc.hpp"

namespace tobas::kdl
{
class TwistAcc
{
public:
  VectorAcc vel;  ///< translational velocity and its 1st and 2nd derivative
  VectorAcc rot;  ///< rotational velocity and its 1st and 2nd derivative

  inline explicit TwistAcc();
  inline explicit TwistAcc(const VectorAcc& _vel, const VectorAcc& _rot);

  static inline TwistAcc Zero();

  inline void setZero();
  inline void setNaN();

  /**
   * Changes the reference point of the TwistAcc.
   * The RVector v_base_AB is expressed in the same base as the TwistAcc.
   * The RVector v_base_AB is a RVector from the old point to the new point.
   * Complexity : 6M+6A
   */
  inline TwistAcc refPoint(const VectorAcc& _v_base_ab);

  inline Twist getTwist() const;
  inline Accel getAccel() const;

  inline TwistAcc& operator+=(const TwistAcc& _arg);
  inline TwistAcc& operator-=(const TwistAcc& _arg);

  inline friend TwistAcc operator*(const TwistAcc& _lhs, double _rhs);
  inline friend TwistAcc operator*(double _lhs, const TwistAcc& _rhs);
  inline friend TwistAcc operator/(const TwistAcc& _lhs, double _rhs);

  inline friend TwistAcc operator*(const TwistAcc& _lhs, const doubleAcc& _rhs);
  inline friend TwistAcc operator*(const doubleAcc& _lhs, const TwistAcc& _rhs);
  inline friend TwistAcc operator/(const TwistAcc& _lhs, const doubleAcc& _rhs);

  inline friend TwistAcc operator+(const TwistAcc& _lhs, const TwistAcc& _rhs);
  inline friend TwistAcc operator-(const TwistAcc& _lhs, const TwistAcc& _rhs);
  inline friend TwistAcc operator-(const TwistAcc& _arg);

  friend class RotationAcc;
  friend class FrameAcc;
};

inline TwistAcc::TwistAcc()
{
}

inline TwistAcc::TwistAcc(const VectorAcc& _vel, const VectorAcc& _rot) : vel(_vel), rot(_rot)
{
}

inline TwistAcc TwistAcc::Zero()
{
  return TwistAcc(VectorAcc::Zero(), VectorAcc::Zero());
}

inline void TwistAcc::setZero()
{
  vel.setZero();
  rot.setZero();
}

inline void TwistAcc::setNaN()
{
  vel.setNaN();
  rot.setNaN();
}

inline TwistAcc TwistAcc::refPoint(const VectorAcc& _v_base_ab)
{
  return TwistAcc(vel + rot * _v_base_ab, rot);
}

inline Twist TwistAcc::getTwist() const
{
  return Twist(vel.p, rot.p);
}

inline Accel TwistAcc::getAccel() const
{
  return Accel(vel.v, rot.v);
}

inline TwistAcc& TwistAcc::operator+=(const TwistAcc& _arg)
{
  vel += _arg.vel;
  rot += _arg.rot;
  return *this;
}

inline TwistAcc& TwistAcc::operator-=(const TwistAcc& _arg)
{
  vel -= _arg.vel;
  rot -= _arg.rot;
  return *this;
}

inline TwistAcc operator*(const TwistAcc& _lhs, double _rhs)
{
  return TwistAcc(_lhs.vel * _rhs, _lhs.rot * _rhs);
}

inline TwistAcc operator*(double _lhs, const TwistAcc& _rhs)
{
  return TwistAcc(_lhs * _rhs.vel, _lhs * _rhs.rot);
}

inline TwistAcc operator/(const TwistAcc& _lhs, double _rhs)
{
  return TwistAcc(_lhs.vel / _rhs, _lhs.rot / _rhs);
}

inline TwistAcc operator*(const TwistAcc& _lhs, const doubleAcc& _rhs)
{
  return TwistAcc(_lhs.vel * _rhs, _lhs.rot * _rhs);
}

inline TwistAcc operator*(const doubleAcc& _lhs, const TwistAcc& _rhs)
{
  return TwistAcc(_lhs * _rhs.vel, _lhs * _rhs.rot);
}

inline TwistAcc operator/(const TwistAcc& _lhs, const doubleAcc& _rhs)
{
  return TwistAcc(_lhs.vel / _rhs, _lhs.rot / _rhs);
}

inline TwistAcc operator+(const TwistAcc& _lhs, const TwistAcc& _rhs)
{
  return TwistAcc(_lhs.vel + _rhs.vel, _lhs.rot + _rhs.rot);
}

inline TwistAcc operator-(const TwistAcc& _lhs, const TwistAcc& _rhs)
{
  return TwistAcc(_lhs.vel - _rhs.vel, _lhs.rot - _rhs.rot);
}

inline TwistAcc operator-(const TwistAcc& _arg)
{
  return TwistAcc(-_arg.vel, -_arg.rot);
}
}  // namespace tobas::kdl
