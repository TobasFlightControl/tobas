// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/typedef.hpp>

#include "./vector.hpp"

namespace tobas
{
namespace kdl
{
class Accel;
using AccelMap = std::map<std::string, Accel>;

/** represents both linear and angular acceleration. */
class Accel
{
public:
  Vector linear;   ///< [m/s^2]
  Vector angular;  ///< [rad/s^2]

  inline explicit Accel();
  inline explicit Accel(const Vector& _linear, const Vector& _angular);

  static inline Accel Zero();

  inline void setZero();
  inline void setNaN();

  inline Eigen::Vector6d ravel() const;

  inline Accel& operator+=(const Accel& _arg);
  inline Accel& operator-=(const Accel& _arg);

  inline friend Accel operator-(const Accel& _arg);
  inline friend Accel operator*(const Accel& _lhs, double _rhs);
  inline friend Accel operator*(double _lhs, const Accel& _rhs);
  inline friend Accel operator/(const Accel& _lhs, double _rhs);
  inline friend Accel operator+(const Accel& _lhs, const Accel& _rhs);
  inline friend Accel operator-(const Accel& _lhs, const Accel& _rhs);

  inline friend std::ostream& operator<<(std::ostream& _os, const Accel& _arg);
};

inline Accel::Accel()
{
}

inline Accel::Accel(const Vector& _linear, const Vector& _angular) : linear(_linear), angular(_angular)
{
}

inline Accel Accel::Zero()
{
  return Accel(Vector::Zero(), Vector::Zero());
}

inline void Accel::setZero()
{
  linear.setZero();
  angular.setZero();
}

inline void Accel::setNaN()
{
  linear.setNaN();
  angular.setNaN();
}

inline Eigen::Vector6d Accel::ravel() const
{
  return (Eigen::Vector6d() << linear.data, angular.data).finished();
}

inline Accel& Accel::operator+=(const Accel& _arg)
{
  linear += _arg.linear;
  angular += _arg.angular;
  return *this;
}

inline Accel& Accel::operator-=(const Accel& _arg)
{
  linear -= _arg.linear;
  angular -= _arg.angular;
  return *this;
}

inline Accel operator-(const Accel& _arg)
{
  return Accel(-_arg.linear, -_arg.angular);
}

inline Accel operator*(const Accel& _lhs, double _rhs)
{
  return Accel(_lhs.linear * _rhs, _lhs.angular * _rhs);
}

inline Accel operator*(double _lhs, const Accel& _rhs)
{
  return Accel(_lhs * _rhs.linear, _lhs * _rhs.angular);
}

inline Accel operator/(const Accel& _lhs, double _rhs)
{
  return Accel(_lhs.linear / _rhs, _lhs.angular / _rhs);
}

inline Accel operator+(const Accel& _lhs, const Accel& _rhs)
{
  return Accel(_lhs.linear + _rhs.linear, _lhs.angular + _rhs.angular);
}

inline Accel operator-(const Accel& _lhs, const Accel& _rhs)
{
  return Accel(_lhs.linear - _rhs.linear, _lhs.angular - _rhs.angular);
}

inline std::ostream& operator<<(std::ostream& _os, const Accel& _arg)
{
  _os << "Linear: " << _arg.linear << ", Angular: " << _arg.angular;
  return _os;
}
}  // namespace kdl
}  // namespace tobas
