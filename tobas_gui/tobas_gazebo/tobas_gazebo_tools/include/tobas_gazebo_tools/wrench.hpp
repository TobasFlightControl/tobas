// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace tobas
{
namespace gazebo
{
class Wrench
{
public:
  gz::math::Vector3d force;
  gz::math::Vector3d torque;

  inline explicit Wrench();
  inline explicit Wrench(const gz::math::Vector3d& _force, const gz::math::Vector3d& _torque);

  static inline Wrench Zero();

  inline void setZero();

  /**
   * Changes the reference point of the wrench.
   * The vector p is expressed in the same base as the wrench.
   * The vector p is a vector from the old point to the new point.
   */
  inline Wrench refPoint(const gz::math::Vector3d& _p) const;

  inline Wrench& operator+=(const Wrench& _arg);
  inline Wrench& operator-=(const Wrench& _arg);

  inline friend Wrench operator-(const Wrench& _arg);
  inline friend Wrench operator*(const Wrench& _lhs, double _rhs);
  inline friend Wrench operator*(double _lhs, const Wrench& _rhs);
  inline friend Wrench operator/(const Wrench& _lhs, double _rhs);
  inline friend Wrench operator+(const Wrench& _lhs, const Wrench& _rhs);
  inline friend Wrench operator-(const Wrench& _lhs, const Wrench& _rhs);

  /** Rotate wrench. */
  inline friend Wrench operator*(const gz::math::Quaterniond& _lhs, const Wrench& _rhs);
};

inline Wrench::Wrench()
{
}

inline Wrench::Wrench(const gz::math::Vector3d& _force, const gz::math::Vector3d& _torque)
  : force(_force), torque(_torque)
{
}

inline Wrench Wrench::Zero()
{
  return Wrench(gz::math::Vector3d::Zero, gz::math::Vector3d::Zero);
}

inline void Wrench::setZero()
{
  force = gz::math::Vector3d::Zero;
  torque = gz::math::Vector3d::Zero;
}

inline Wrench Wrench::refPoint(const gz::math::Vector3d& _p) const
{
  return Wrench(force, torque + force.Cross(_p));
}

inline Wrench& Wrench::operator+=(const Wrench& _arg)
{
  torque += _arg.torque;
  force += _arg.force;
  return *this;
}

inline Wrench& Wrench::operator-=(const Wrench& _arg)
{
  torque -= _arg.torque;
  force -= _arg.force;
  return *this;
}

inline Wrench operator-(const Wrench& _arg)
{
  return Wrench(-_arg.force, -_arg.torque);
}

inline Wrench operator*(const Wrench& _lhs, double _rhs)
{
  return Wrench(_lhs.force * _rhs, _lhs.torque * _rhs);
}

inline Wrench operator*(double _lhs, const Wrench& _rhs)
{
  return Wrench(_lhs * _rhs.force, _lhs * _rhs.torque);
}

inline Wrench operator/(const Wrench& _lhs, double _rhs)
{
  return Wrench(_lhs.force / _rhs, _lhs.torque / _rhs);
}

inline Wrench operator+(const Wrench& _lhs, const Wrench& _rhs)
{
  return Wrench(_lhs.force + _rhs.force, _lhs.torque + _rhs.torque);
}

inline Wrench operator-(const Wrench& _lhs, const Wrench& _rhs)
{
  return Wrench(_lhs.force - _rhs.force, _lhs.torque - _rhs.torque);
}

inline Wrench operator*(const gz::math::Quaterniond& _lhs, const Wrench& _rhs)
{
  return Wrench(_lhs.RotateVector(_rhs.force), _lhs.RotateVector(_rhs.torque));
}
}  // namespace gazebo
}  // namespace tobas
