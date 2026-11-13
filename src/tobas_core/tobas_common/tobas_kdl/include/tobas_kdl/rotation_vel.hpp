// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/framevel.hpp
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/framevel.inl
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

#include "./vector_vel.hpp"

namespace tobas::kdl
{
class RotationVel
{
public:
  Rotation R;  ///< Rotation matrix
  Vector w;    ///< rotation vector

  inline explicit RotationVel();
  inline explicit RotationVel(const Rotation& _r);
  inline explicit RotationVel(const Rotation& _r, const Vector& _w);

  static inline RotationVel Identity();

  static inline RotationVel RotX(const doubleVel& _angle);
  static inline RotationVel RotY(const doubleVel& _angle);
  static inline RotationVel RotZ(const doubleVel& _angle);

  /** Axis must be normalized. Rotation around a constant vector. */
  static inline RotationVel Rot(const Vector& _axis, const doubleVel& _angle);

  inline void setIdentity();
  inline void setNaN();

  inline RotationVel inverse() const;
  inline VectorVel inverse(const VectorVel& _arg) const;
  inline VectorVel inverse(const Vector& _arg) const;

  inline VectorVel operator*(const VectorVel& _arg) const;
  inline VectorVel operator*(const Vector& _arg) const;

  inline friend RotationVel operator*(const RotationVel& _r1, const RotationVel& _r2);
  inline friend RotationVel operator*(const Rotation& _r1, const RotationVel& _r2);
  inline friend RotationVel operator*(const RotationVel& _r1, const Rotation& _r2);
};

inline RotationVel::RotationVel()
{
}

inline RotationVel::RotationVel(const Rotation& _r) : R(_r), w(Vector::Zero())
{
}

inline RotationVel::RotationVel(const Rotation& _r, const Vector& _w) : R(_r), w(_w)
{
}

inline RotationVel RotationVel::Identity()
{
  return RotationVel(Rotation::Identity(), Vector::Zero());
}

inline RotationVel RotationVel::RotX(const doubleVel& _angle)
{
  return RotationVel(Rotation::RotX(_angle.t), Vector(_angle.grad, 0, 0));
}

inline RotationVel RotationVel::RotY(const doubleVel& _angle)
{
  return RotationVel(Rotation::RotX(_angle.t), Vector(0, _angle.grad, 0));
}

inline RotationVel RotationVel::RotZ(const doubleVel& _angle)
{
  return RotationVel(Rotation::RotZ(_angle.t), Vector(0, 0, _angle.grad));
}

inline RotationVel RotationVel::Rot(const Vector& _axis, const doubleVel& _angle)
{
  return RotationVel(Rotation::Rot(_axis, _angle.t), _axis * _angle.grad);
}

inline void RotationVel::setIdentity()
{
  R.setIdentity();
  w.setZero();
}

inline void RotationVel::setNaN()
{
  R.setNaN();
  w.setNaN();
}

inline RotationVel RotationVel::inverse() const
{
  return RotationVel(R.inverse(), -R.inverse(w));
}

inline VectorVel RotationVel::inverse(const VectorVel& _arg) const
{
  const auto tmp = R.inverse(_arg.p);
  return VectorVel(tmp, R.inverse(_arg.v - w * _arg.p));
}

inline VectorVel RotationVel::inverse(const Vector& _arg) const
{
  const auto tmp = R.inverse(_arg);
  return VectorVel(tmp, R.inverse(-w * _arg));
}

inline VectorVel RotationVel::operator*(const VectorVel& _arg) const
{
  const auto tmp = R * _arg.p;
  return VectorVel(tmp, w * tmp + R * _arg.v);
}

inline VectorVel RotationVel::operator*(const Vector& _arg) const
{
  const auto tmp = R * _arg;
  return VectorVel(tmp, w * tmp);
}

inline RotationVel operator*(const RotationVel& _r1, const RotationVel& _r2)
{
  return RotationVel(_r1.R * _r2.R, _r1.w + _r1.R * _r2.w);
}

inline RotationVel operator*(const Rotation& _r1, const RotationVel& _r2)
{
  return RotationVel(_r1 * _r2.R, _r1 * _r2.w);
}

inline RotationVel operator*(const RotationVel& _r1, const Rotation& _r2)
{
  return RotationVel(_r1.R * _r2, _r1.w);
}
}  // namespace tobas::kdl
