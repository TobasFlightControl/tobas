// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

#include <eigen3/Eigen/Core>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_math/float.hpp>

namespace tobas
{
namespace kdl
{
class Vector;
using VectorMap = std::map<std::string, Vector>;

/**
 * @brief A concrete implementation of a 3 dimensional vector class.
 */
class Vector
{
public:
  Eigen::Vector3d data;

  inline Vector();
  inline Vector(double x, double y, double z);
  inline Vector(const Eigen::Vector3d& data);

  static inline Vector Zero();
  static inline Vector Constant(const double& value);
  static inline Vector UnitX();
  static inline Vector UnitY();
  static inline Vector UnitZ();

  /* Access to elements, range checked when NDEBUG is not set, from 0..2 */
  inline double operator()(size_t index) const;
  /* Access to elements, range checked when NDEBUG is not set, from 0..2 */
  inline double& operator()(size_t index);

  inline const double& x() const;
  inline const double& y() const;
  inline const double& z() const;
  inline double& x();
  inline double& y();
  inline double& z();
  inline void x(double x);
  inline void y(double y);
  inline void z(double z);

  inline Vector clone() const;

  inline void set(double x, double y, double z);

  inline void fill(double value);

  /* Get the minimum element. */
  inline double min() const;

  /* Get the maximum element. */
  inline double max() const;

  /* Compute the element-wise minimum of two `Vector`s. */
  inline Vector min(const Vector& rhs) const;

  /* Compute the element-wise maximum of two `Vector`s. */
  inline Vector max(const Vector& rhs) const;

  /* Compute the dot product of two `Vector`s. */
  inline double dot(const Vector& rhs) const;

  /* Compute the element-wise product of two `Vector`s. */
  inline Vector hadamard(const Vector& rhs) const;

  /* Compute the angle between two `Vector`s, from 0 to pi [rad]. */
  inline double argument(const Vector& rhs) const;

  /* Determine whether two `Vector`s are perpendicular. */
  inline bool isPerpendicular(const Vector& rhs) const;

  /* Determine whether two `Vector`s are parallel. */
  bool
  isParallel(const Vector& rhs, bool same_direction_only = false, double angle_tol_rad = 1e-3, double zero_tol = 1e-12)
    const;

  /* Clamp each value. */
  inline Vector clamp(const double& lb, const double& ub) const;
  inline Vector clamp(const Vector& lb, const Vector& ub) const;

  inline void setZero();
  inline void setNaN();

  inline double norm() const;
  inline double squaredNorm() const;
  inline void normalize();
  inline Vector normalized() const;

  inline Vector sqr() const;
  inline Vector sqrt() const;
  inline Vector cube() const;
  inline Vector inverse() const;
  inline Vector abs() const;

  inline bool isFinite() const;

  /* An exact comparison. */
  inline bool operator==(const Vector& rhs) const;

  /* Adds a vector from the Vector object itself. */
  inline Vector& operator+=(const Vector& arg);
  /* Subtracts a vector from the Vector object itself. */
  inline Vector& operator-=(const Vector& arg);

  inline friend Vector operator-(const Vector& arg);
  inline friend Vector operator+(const Vector& lhs, double rhs);
  inline friend Vector operator+(double lhs, const Vector& rhs);
  inline friend Vector operator-(const Vector& lhs, double rhs);
  inline friend Vector operator-(double lhs, const Vector& rhs);
  inline friend Vector operator*(const Vector& lhs, double rhs);
  inline friend Vector operator*(double lhs, const Vector& rhs);
  inline friend Vector operator/(const Vector& lhs, double rhs);
  inline friend Vector operator/(double lhs, const Vector& rhs);
  inline friend Vector operator+(const Vector& lhs, const Vector& rhs);
  inline friend Vector operator-(const Vector& lhs, const Vector& rhs);
  inline friend Vector operator*(const Vector& lhs, const Vector& rhs);

  inline friend std::ostream& operator<<(std::ostream& os, const Vector& arg);
};

inline Vector::Vector()
{
}

inline Vector::Vector(double x, double y, double z) : data(x, y, z)
{
}

inline Vector::Vector(const Eigen::Vector3d& _data) : data(_data)
{
}

inline Vector Vector::Zero()
{
  return Vector(Eigen::Vector3d::Zero());
}

inline Vector Vector::Constant(const double& value)
{
  return Vector(Eigen::Vector3d::Constant(value));
}

inline Vector Vector::UnitX()
{
  return Vector(Eigen::Vector3d::UnitX());
}

inline Vector Vector::UnitY()
{
  return Vector(Eigen::Vector3d::UnitY());
}

inline Vector Vector::UnitZ()
{
  return Vector(Eigen::Vector3d::UnitZ());
}

inline double Vector::operator()(size_t index) const
{
  return data(index);
}

inline double& Vector::operator()(size_t index)
{
  return data(index);
}

inline const double& Vector::x() const
{
  return data.x();
}

inline const double& Vector::y() const
{
  return data.y();
}

inline const double& Vector::z() const
{
  return data.z();
}

inline double& Vector::x()
{
  return data.x();
}

inline double& Vector::y()
{
  return data.y();
}

inline double& Vector::z()
{
  return data.z();
}

inline void Vector::x(double _x)
{
  data.x() = _x;
}

inline void Vector::y(double _y)
{
  data.y() = _y;
}

inline void Vector::z(double _z)
{
  data.z() = _z;
}

inline Vector Vector::clone() const
{
  return *this;
}

inline void Vector::set(double _x, double _y, double _z)
{
  data << _x, _y, _z;
}

inline void Vector::fill(double value)
{
  data.fill(value);
}

inline double Vector::min() const
{
  return data.minCoeff();
}

inline double Vector::max() const
{
  return data.maxCoeff();
}

inline double Vector::dot(const Vector& rhs) const
{
  return data.dot(rhs.data);
}

inline Vector Vector::min(const Vector& rhs) const
{
  return Vector(data.cwiseMin(rhs.data));
}

inline Vector Vector::max(const Vector& rhs) const
{
  return Vector(data.cwiseMax(rhs.data));
}

inline Vector Vector::hadamard(const Vector& rhs) const
{
  return Vector(data.cwiseProduct(rhs.data));
}

inline double Vector::argument(const Vector& rhs) const
{
  return std::acos(normalized().dot(rhs.normalized()));
}

inline bool Vector::isPerpendicular(const Vector& rhs) const
{
  return math::isClose(dot(rhs), 0.0);
}

inline Vector Vector::clamp(const double& lb, const double& ub) const
{
  return Vector(data.cwiseMax(lb).cwiseMin(ub));
}

inline Vector Vector::clamp(const Vector& lb, const Vector& ub) const
{
  return Vector(data.cwiseMax(lb.data).cwiseMin(ub.data));
}

inline void Vector::setZero()
{
  data.setZero();
}

inline void Vector::setNaN()
{
  data.setConstant(NAN);
}

inline double Vector::norm() const
{
  return data.norm();
}

inline double Vector::squaredNorm() const
{
  return data.squaredNorm();
}

inline void Vector::normalize()
{
  assert(squaredNorm() > 0.0);
  data.normalize();
}

inline Vector Vector::normalized() const
{
  assert(squaredNorm() > 0.0);
  return Vector(data.normalized());
}

inline Vector Vector::sqr() const
{
  return Vector(data.cwiseAbs2());
}

inline Vector Vector::sqrt() const
{
  assert((data.array() > 0.0).all());
  return Vector(data.cwiseSqrt());
}

inline Vector Vector::cube() const
{
  return Vector(data.cwiseProduct(data).cwiseProduct(data));
}

inline Vector Vector::inverse() const
{
  assert((data.array() != 0.0).all());
  return Vector(data.cwiseInverse());
}

inline Vector Vector::abs() const
{
  return Vector(data.cwiseAbs());
}

bool Vector::isFinite() const
{
  return eigen::isFinite(data);
}

inline bool Vector::operator==(const Vector& rhs) const
{
  return data.cwiseEqual(rhs.data).all();
}

inline Vector& Vector::operator+=(const Vector& arg)
{
  data += arg.data;
  return *this;
}

inline Vector& Vector::operator-=(const Vector& arg)
{
  data -= arg.data;
  return *this;
}

inline Vector operator-(const Vector& arg)
{
  return Vector(-arg.data);
}

inline Vector operator+(const Vector& lhs, double rhs)
{
  return Vector(lhs.data.array() + rhs);
}

inline Vector operator+(double lhs, const Vector& rhs)
{
  return Vector(lhs + rhs.data.array());
}

inline Vector operator-(const Vector& lhs, double rhs)
{
  return Vector(lhs.data.array() - rhs);
}

inline Vector operator-(double lhs, const Vector& rhs)
{
  return Vector(lhs - rhs.data.array());
}

inline Vector operator*(const Vector& lhs, double rhs)
{
  return Vector(lhs.data * rhs);
}

inline Vector operator*(double lhs, const Vector& rhs)
{
  return Vector(lhs * rhs.data);
}

inline Vector operator/(const Vector& lhs, double rhs)
{
  assert(rhs != 0);
  return Vector(lhs.data / rhs);
}

inline Vector operator/(double lhs, const Vector& rhs)
{
  return lhs * rhs.inverse();
}

inline Vector operator+(const Vector& lhs, const Vector& rhs)
{
  return Vector(lhs.data + rhs.data);
}

inline Vector operator-(const Vector& lhs, const Vector& rhs)
{
  return Vector(lhs.data - rhs.data);
}

inline Vector operator*(const Vector& lhs, const Vector& rhs)
{
  return Vector(lhs.data.cross(rhs.data));
}

inline std::ostream& operator<<(std::ostream& os, const Vector& arg)
{
  os << arg.data.transpose();
  return os;
}
}  // namespace kdl
}  // namespace tobas
