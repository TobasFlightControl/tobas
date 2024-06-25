#pragma once

#include <iostream>

#include "./frames.hpp"

namespace kdl
{
class AngleAxis
{
public:
  inline explicit AngleAxis();
  inline explicit AngleAxis(const double& angle, const Vector& axis);
  inline explicit AngleAxis(const Vector& angle_axis);

  inline Vector toVector() const;

  inline const double& angle() const;
  inline const Vector& axis() const;

  inline friend std::ostream& operator<<(std::ostream& os, const AngleAxis& arg);

private:
  double angle_;
  Vector axis_;  // Axis of rotation (normalized)
};

inline AngleAxis::AngleAxis() : angle_(0), axis_(1, 0, 0)
{
}

inline AngleAxis::AngleAxis(const double& angle, const Vector& axis) : angle_(angle), axis_(axis)
{
  axis_.normalize();
}

inline AngleAxis::AngleAxis(const Vector& angle_axis) : angle_(angle_axis.norm()), axis_(angle_axis)
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

inline std::ostream& operator<<(std::ostream& os, const AngleAxis& arg)
{
  os << "Angle: " << arg.angle() << ", Axis: " << arg.axis();
  return os;
}
}  // namespace kdl
