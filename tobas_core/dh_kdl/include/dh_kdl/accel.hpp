#pragma once

#include <dh_eigen_tools/typedef.hpp>

#include "./vector.hpp"

namespace KDL
{
class Accel;
using AccelMap = std::map<std::string, Accel>;

/**
 * \brief represents both linear and angular acceleration.
 */
class Accel
{
public:
  Vector linear;   // [m/s^2]
  Vector angular;  // [rad/s^2]

  inline explicit Accel();
  inline explicit Accel(const Vector& linear, const Vector& angular);

  inline static Accel Zero();

  inline void setZero();

  /**
   * @brief Changes the reference point of the accel.
   * The vector p is expressed in the same base as the accel.
   * The vector p is a vector from the old point to the new point.
   */
  inline Accel refPoint(const Vector& p) const;

  inline Accel& operator+=(const Accel& arg);
  inline Accel& operator-=(const Accel& arg);

  inline friend Accel operator-(const Accel& arg);
  inline friend Accel operator*(const Accel& lhs, double rhs);
  inline friend Accel operator*(double lhs, const Accel& rhs);
  inline friend Accel operator/(const Accel& lhs, double rhs);
  inline friend Accel operator+(const Accel& lhs, const Accel& rhs);
  inline friend Accel operator-(const Accel& lhs, const Accel& rhs);

  Eigen::Vector6d ravel() const;

  friend std::ostream& operator<<(std::ostream& os, const Accel& arg);
};

inline Accel::Accel()
{
}

inline Accel::Accel(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
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

inline Accel& Accel::operator+=(const Accel& arg)
{
  linear += arg.linear;
  angular += arg.angular;
  return *this;
}

inline Accel& Accel::operator-=(const Accel& arg)
{
  linear -= arg.linear;
  angular -= arg.angular;
  return *this;
}

inline Accel operator-(const Accel& arg)
{
  return Accel(-arg.linear, -arg.angular);
}

inline Accel operator*(const Accel& lhs, double rhs)
{
  return Accel(lhs.linear * rhs, lhs.angular * rhs);
}

inline Accel operator*(double lhs, const Accel& rhs)
{
  return Accel(lhs * rhs.linear, lhs * rhs.angular);
}

inline Accel operator/(const Accel& lhs, double rhs)
{
  return Accel(lhs.linear / rhs, lhs.angular / rhs);
}

inline Accel operator+(const Accel& lhs, const Accel& rhs)
{
  return Accel(lhs.linear + rhs.linear, lhs.angular + rhs.angular);
}

inline Accel operator-(const Accel& lhs, const Accel& rhs)
{
  return Accel(lhs.linear - rhs.linear, lhs.angular - rhs.angular);
}
}  // namespace KDL
