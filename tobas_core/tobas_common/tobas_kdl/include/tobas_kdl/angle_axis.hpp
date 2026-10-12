// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <iostream>

#include "./frames.hpp"

namespace tobas
{
namespace kdl
{
class AngleAxis
{
public:
  inline explicit AngleAxis();
  inline explicit AngleAxis(double _angle, const Vector& _axis);
  inline explicit AngleAxis(const Vector& _angle_axis);

  inline Vector toVector() const;

  inline const double& angle() const;
  inline const Vector& axis() const;

  inline friend std::ostream& operator<<(std::ostream& _os, const AngleAxis& _arg);

private:
  double angle_;
  Vector axis_;  ///< Axis of rotation (normalized)
};

inline AngleAxis::AngleAxis() : angle_(0), axis_(1, 0, 0)
{
}

inline AngleAxis::AngleAxis(double _angle, const Vector& _axis) : angle_(_angle), axis_(_axis)
{
  axis_.normalize();
}

inline AngleAxis::AngleAxis(const Vector& _angle_axis) : angle_(_angle_axis.norm()), axis_(_angle_axis)
{
  axis_.normalize();
}

inline Vector AngleAxis::toVector() const
{
  return angle_ * axis_;
}

inline const double& AngleAxis::angle() const
{
  return angle_;
}

inline const Vector& AngleAxis::axis() const
{
  return axis_;
}

inline std::ostream& operator<<(std::ostream& _os, const AngleAxis& _arg)
{
  _os << "Angle: " << _arg.angle() << ", Axis: " << _arg.axis();
  return _os;
}
}  // namespace kdl
}  // namespace tobas
