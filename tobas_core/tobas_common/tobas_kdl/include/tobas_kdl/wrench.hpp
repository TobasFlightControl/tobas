// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./vector.hpp"

namespace tobas
{
namespace kdl
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
}  // namespace kdl
}  // namespace tobas
