#pragma once

#include <tobas_eigen_tools/typedef.hpp>

#include "./vector.hpp"

namespace kdl
{
class Accel;
using AccelMap = std::map<std::string, Accel>;

/**
 * @brief represents both linear and angular acceleration.
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

  inline Eigen::Vector6d ravel() const;

  inline Accel& operator+=(const Accel& arg);
  inline Accel& operator-=(const Accel& arg);

  inline friend Accel operator-(const Accel& arg);
  inline friend Accel operator*(const Accel& lhs, double rhs);
  inline friend Accel operator*(double lhs, const Accel& rhs);
  inline friend Accel operator/(const Accel& lhs, double rhs);
  inline friend Accel operator+(const Accel& lhs, const Accel& rhs);
  inline friend Accel operator-(const Accel& lhs, const Accel& rhs);

  inline friend std::ostream& operator<<(std::ostream& os, const Accel& arg);
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

inline Eigen::Vector6d Accel::ravel() const
{
  return (Eigen::Vector6d() << linear.data, angular.data).finished();
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

inline std::ostream& operator<<(std::ostream& os, const Accel& arg)
{
  os << "Linear: " << arg.linear << ", Angular: " << arg.angular;
  return os;
}
}  // namespace kdl
