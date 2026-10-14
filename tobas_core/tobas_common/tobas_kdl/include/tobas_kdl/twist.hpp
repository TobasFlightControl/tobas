// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/frames.hpp
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/frames.inl
// Modified by Tobas, Inc.

/***************************************************************************
                        frames.hpp `-  description
                       -------------------------
    begin                : June 2006
    copyright            : (C) 2006 Erwin Aertbelien
    email                : firstname.lastname@mech.kuleuven.be

 History (only major changes)( AUTHOR-Description ) :

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#pragma once

#include <tobas_eigen_tools/typedef.hpp>

#include "./accel.hpp"
#include "./wrench.hpp"

namespace tobas::kdl
{
class Twist;
using TwistMap = std::map<std::string, Twist>;

/**
 * represents both translational and rotational velocities.
 *
 * This class represents a twist. A twist is the combination of translational
 * velocity and rotational velocity applied at one point.
 */
class Twist
{
public:
  Vector vel;  ///< The linear velocity of that point
  Vector rot;  ///< The angular velocity of that point

  inline explicit Twist();
  inline explicit Twist(const Vector& _vel, const Vector& _rot);

  static inline Twist Zero();

  inline void setZero();
  inline void setNaN();

  inline Eigen::Vector6d ravel() const;

  /**
   * Changes the reference point of the twist.
   * The vector `p` is expressed in the same base as the twist.
   * The vector `p` is a vector from the old point to the new point.
   * Compute the twist of frame B in the same rigid body as self frame A.
   */
  inline Twist refPoint(const Vector& _p) const;

  /** index-based access to components, first vel(0..2), then rot(3..5) */
  inline double operator()(size_t _i) const;
  /** index-based access to components, first vel(0..2), then rot(3..5) */
  inline double& operator()(size_t _i);

  inline Twist& operator+=(const Twist& _arg);
  inline Twist& operator-=(const Twist& _arg);

  inline friend Twist operator-(const Twist& _arg);
  inline friend Twist operator*(const Twist& _lhs, double _rhs);
  inline friend Twist operator*(double _lhs, const Twist& _rhs);
  inline friend Twist operator/(const Twist& _lhs, double _rhs);
  inline friend Twist operator+(const Twist& _lhs, const Twist& _rhs);
  inline friend Twist operator-(const Twist& _lhs, const Twist& _rhs);

  /**
   * Spatial cross product for 6d motion vectors,
   * beware all of them have to be expressed in the same reference frame.
   */
  inline friend Accel operator*(const Twist& _lhs, const Twist& _rhs);

  inline friend std::ostream& operator<<(std::ostream& _os, const Twist& _arg);
};

inline Twist::Twist()
{
}

inline Twist::Twist(const Vector& _vel, const Vector& _rot) : vel(_vel), rot(_rot)
{
}

inline Twist Twist::Zero()
{
  return Twist(Vector::Zero(), Vector::Zero());
}

inline void Twist::setZero()
{
  vel.setZero();
  rot.setZero();
}

inline void Twist::setNaN()
{
  vel.setNaN();
  rot.setNaN();
}

inline Eigen::Vector6d Twist::ravel() const
{
  return (Eigen::Vector6d() << vel.data, rot.data).finished();
}

inline Twist Twist::refPoint(const Vector& _p) const
{
  return Twist(vel + rot * _p, rot);
}

inline double Twist::operator()(size_t _i) const
{
  return _i < 3 ? vel(_i) : rot(_i - 3);
}

inline double& Twist::operator()(size_t _i)
{
  return _i < 3 ? vel(_i) : rot(_i - 3);
}

inline Twist& Twist::operator+=(const Twist& _arg)
{
  vel += _arg.vel;
  rot += _arg.rot;
  return *this;
}

inline Twist& Twist::operator-=(const Twist& _arg)
{
  vel -= _arg.vel;
  rot -= _arg.rot;
  return *this;
}

inline Twist operator-(const Twist& _arg)
{
  return Twist(-_arg.vel, -_arg.rot);
}

inline Twist operator*(const Twist& _lhs, double _rhs)
{
  return Twist(_lhs.vel * _rhs, _lhs.rot * _rhs);
}

inline Twist operator*(double _lhs, const Twist& _rhs)
{
  return Twist(_lhs * _rhs.vel, _lhs * _rhs.rot);
}

inline Twist operator/(const Twist& _lhs, double _rhs)
{
  return Twist(_lhs.vel / _rhs, _lhs.rot / _rhs);
}

inline Twist operator+(const Twist& _lhs, const Twist& _rhs)
{
  return Twist(_lhs.vel + _rhs.vel, _lhs.rot + _rhs.rot);
}

inline Twist operator-(const Twist& _lhs, const Twist& _rhs)
{
  return Twist(_lhs.vel - _rhs.vel, _lhs.rot - _rhs.rot);
}

inline Accel operator*(const Twist& _lhs, const Twist& _rhs)
{
  return Accel(_lhs.rot * _rhs.vel + _lhs.vel * _rhs.rot, _lhs.rot * _rhs.rot);
}

inline std::ostream& operator<<(std::ostream& _os, const Twist& _arg)
{
  _os << "Linear: " << _arg.vel << ", Angular: " << _arg.rot;
  return _os;
}
}  // namespace tobas::kdl
