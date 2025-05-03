#pragma once

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

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

  // Changes the reference point of the wrench.
  // The vector p is expressed in the same base as the wrench.
  // The vector p is a vector from the old point to the new point.
  inline Wrench refPoint(const gz::math::Vector3d& p) const;

  inline Wrench& operator+=(const Wrench& arg);
  inline Wrench& operator-=(const Wrench& arg);

  inline friend Wrench operator-(const Wrench& arg);
  inline friend Wrench operator*(const Wrench& lhs, double rhs);
  inline friend Wrench operator*(double lhs, const Wrench& rhs);
  inline friend Wrench operator/(const Wrench& lhs, double rhs);
  inline friend Wrench operator+(const Wrench& lhs, const Wrench& rhs);
  inline friend Wrench operator-(const Wrench& lhs, const Wrench& rhs);

  /* Rotate wrench. */
  inline friend Wrench operator*(const gz::math::Quaterniond& lhs, const Wrench& rhs);
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

inline Wrench Wrench::refPoint(const gz::math::Vector3d& p) const
{
  return Wrench(force, torque + force.Cross(p));
}

inline Wrench& Wrench::operator+=(const Wrench& arg)
{
  torque += arg.torque;
  force += arg.force;
  return *this;
}

inline Wrench& Wrench::operator-=(const Wrench& arg)
{
  torque -= arg.torque;
  force -= arg.force;
  return *this;
}

inline Wrench operator-(const Wrench& arg)
{
  return Wrench(-arg.force, -arg.torque);
}

inline Wrench operator*(const Wrench& lhs, double rhs)
{
  return Wrench(lhs.force * rhs, lhs.torque * rhs);
}

inline Wrench operator*(double lhs, const Wrench& rhs)
{
  return Wrench(lhs * rhs.force, lhs * rhs.torque);
}

inline Wrench operator/(const Wrench& lhs, double rhs)
{
  return Wrench(lhs.force / rhs, lhs.torque / rhs);
}

inline Wrench operator+(const Wrench& lhs, const Wrench& rhs)
{
  return Wrench(lhs.force + rhs.force, lhs.torque + rhs.torque);
}

inline Wrench operator-(const Wrench& lhs, const Wrench& rhs)
{
  return Wrench(lhs.force - rhs.force, lhs.torque - rhs.torque);
}

inline Wrench operator*(const gz::math::Quaterniond& lhs, const Wrench& rhs)
{
  return Wrench(lhs.RotateVector(rhs.force), lhs.RotateVector(rhs.torque));
}
}  // namespace gazebo
