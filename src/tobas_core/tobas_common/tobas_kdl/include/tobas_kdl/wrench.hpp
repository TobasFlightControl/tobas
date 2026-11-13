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
 *   modify it under the terms of the GNU General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 3 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#pragma once

#include "./vector.hpp"

namespace tobas::kdl
{
class Wrench;
using Wrenches = std::vector<Wrench>;
using WrenchMap = std::map<std::string, Wrench>;

class Wrench
{
public:
  Vector force;   ///< Force that is applied at the origin of the current ref frame
  Vector torque;  ///< Torque that is applied at the origin of the current ref frame

  inline explicit Wrench();
  inline explicit Wrench(const Vector& _force, const Vector& _torque);

  static inline Wrench Zero();

  inline void setZero();
  inline void setNaN();

  /**
   * Changes the reference point of the wrench.
   * The vector p is expressed in the same base as the wrench.
   * The vector p is a vector from the old point to the new point.
   */
  inline Wrench refPoint(const Vector& _p) const;

  /** Index-based access to components, first force(0..2), then torque(3..5) */
  inline double operator()(size_t _index) const;
  inline double& operator()(size_t _index);

  inline Wrench& operator+=(const Wrench& _arg);
  inline Wrench& operator-=(const Wrench& _arg);

  inline friend Wrench operator-(const Wrench& _arg);
  inline friend Wrench operator*(const Wrench& _lhs, double _rhs);
  inline friend Wrench operator*(double _lhs, const Wrench& _rhs);
  inline friend Wrench operator/(const Wrench& _lhs, double _rhs);
  inline friend Wrench operator+(const Wrench& _lhs, const Wrench& _rhs);
  inline friend Wrench operator-(const Wrench& _lhs, const Wrench& _rhs);

  inline friend std::ostream& operator<<(std::ostream& _os, const Wrench& _arg);
};

inline Wrench::Wrench()
{
}

inline Wrench::Wrench(const Vector& _force, const Vector& _torque) : force(_force), torque(_torque)
{
}

inline Wrench Wrench::Zero()
{
  return Wrench(Vector::Zero(), Vector::Zero());
}

inline void Wrench::setZero()
{
  force.setZero();
  torque.setZero();
}

inline void Wrench::setNaN()
{
  force.setNaN();
  torque.setNaN();
}

inline Wrench Wrench::refPoint(const Vector& _p) const
{
  return Wrench(force, torque + force * _p);
}

inline double Wrench::operator()(size_t _index) const
{
  assert(_index < 6);
  return _index < 3 ? force(_index) : torque(_index - 3);
}

inline double& Wrench::operator()(size_t _index)
{
  assert(_index < 6);
  return _index < 3 ? force(_index) : torque(_index - 3);
}

inline Wrench& Wrench::operator+=(const Wrench& _arg)
{
  force += _arg.force;
  torque += _arg.torque;
  return *this;
}

inline Wrench& Wrench::operator-=(const Wrench& _arg)
{
  force -= _arg.force;
  torque -= _arg.torque;
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

inline std::ostream& operator<<(std::ostream& _os, const Wrench& _arg)
{
  _os << "Force: " << _arg.force << ", Torque: " << _arg.torque;
  return _os;
}
}  // namespace tobas::kdl
