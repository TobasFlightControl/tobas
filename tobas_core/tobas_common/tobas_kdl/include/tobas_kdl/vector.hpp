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

/** A concrete implementation of a 3 dimensional vector class. */
class Vector
{
public:
  Eigen::Vector3d data;

  inline Vector();
  inline Vector(double _x, double _y, double _z);
  inline Vector(const Eigen::Vector3d& _data);

  static inline Vector Zero();
  static inline Vector Constant(double _value);
  static inline Vector UnitX();
  static inline Vector UnitY();
  static inline Vector UnitZ();

  /** Access to elements, range checked when NDEBUG is not set, from 0..2 */
  inline double operator()(size_t _index) const;
  /** Access to elements, range checked when NDEBUG is not set, from 0..2 */
  inline double& operator()(size_t _index);

  inline const double& x() const;
  inline const double& y() const;
  inline const double& z() const;
  inline double& x();
  inline double& y();
  inline double& z();
  inline void x(double _x);
  inline void y(double _y);
  inline void z(double _z);

  inline Vector clone() const;

  inline void set(double _x, double _y, double _z);

  inline void fill(double _value);

  /** Get the minimum element. */
  inline double min() const;

  /** Get the maximum element. */
  inline double max() const;

  /** Compute the element-wise minimum of two `Vector`s. */
  inline Vector min(const Vector& _rhs) const;

  /** Compute the element-wise maximum of two `Vector`s. */
  inline Vector max(const Vector& _rhs) const;

  /** Compute the dot product of two `Vector`s. */
  inline double dot(const Vector& _rhs) const;

  /** Compute the element-wise product of two `Vector`s. */
  inline Vector hadamard(const Vector& _rhs) const;

  /** Compute the angle between two `Vector`s, from 0 to pi [rad]. */
  inline double argument(const Vector& _rhs) const;

  /** Determine whether two `Vector`s are perpendicular. */
  inline bool isPerpendicular(const Vector& _rhs) const;

  /** Determine whether two `Vector`s are parallel. */
  bool isParallel(const Vector& _rhs, bool _same_direction_only = false, double _angle_tol_rad = 1e-3) const;

  /** Clamp each value. */
  inline Vector clamp(double _lb, double _ub) const;
  inline Vector clamp(const Vector& _lb, const Vector& _ub) const;

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

  /** An exact comparison. */
  inline bool operator==(const Vector& _rhs) const;

  /** Adds a vector from the Vector object itself. */
  inline Vector& operator+=(const Vector& _arg);
  /** Subtracts a vector from the Vector object itself. */
  inline Vector& operator-=(const Vector& _arg);

  inline friend Vector operator-(const Vector& _arg);
  inline friend Vector operator+(const Vector& _lhs, double _rhs);
  inline friend Vector operator+(double _lhs, const Vector& _rhs);
  inline friend Vector operator-(const Vector& _lhs, double _rhs);
  inline friend Vector operator-(double _lhs, const Vector& _rhs);
  inline friend Vector operator*(const Vector& _lhs, double _rhs);
  inline friend Vector operator*(double _lhs, const Vector& _rhs);
  inline friend Vector operator/(const Vector& _lhs, double _rhs);
  inline friend Vector operator/(double _lhs, const Vector& _rhs);
  inline friend Vector operator+(const Vector& _lhs, const Vector& _rhs);
  inline friend Vector operator-(const Vector& _lhs, const Vector& _rhs);
  inline friend Vector operator*(const Vector& _lhs, const Vector& _rhs);

  inline friend std::ostream& operator<<(std::ostream& _os, const Vector& _arg);
};

inline Vector::Vector()
{
}

inline Vector::Vector(double _x, double _y, double _z) : data(_x, _y, _z)
{
}

inline Vector::Vector(const Eigen::Vector3d& _data) : data(_data)
{
}

inline Vector Vector::Zero()
{
  return Vector(Eigen::Vector3d::Zero());
}

inline Vector Vector::Constant(double _value)
{
  return Vector(Eigen::Vector3d::Constant(_value));
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

inline double Vector::operator()(size_t _index) const
{
  return data(_index);
}

inline double& Vector::operator()(size_t _index)
{
  return data(_index);
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

inline void Vector::fill(double _value)
{
  data.fill(_value);
}

inline double Vector::min() const
{
  return data.minCoeff();
}

inline double Vector::max() const
{
  return data.maxCoeff();
}

inline double Vector::dot(const Vector& _rhs) const
{
  return data.dot(_rhs.data);
}

inline Vector Vector::min(const Vector& _rhs) const
{
  return Vector(data.cwiseMin(_rhs.data));
}

inline Vector Vector::max(const Vector& _rhs) const
{
  return Vector(data.cwiseMax(_rhs.data));
}

inline Vector Vector::hadamard(const Vector& _rhs) const
{
  return Vector(data.cwiseProduct(_rhs.data));
}

inline double Vector::argument(const Vector& _rhs) const
{
  return std::acos(normalized().dot(_rhs.normalized()));
}

inline bool Vector::isPerpendicular(const Vector& _rhs) const
{
  return math::isClose(dot(_rhs), 0.0);
}

inline Vector Vector::clamp(double _lb, double _ub) const
{
  return Vector(data.cwiseMax(_lb).cwiseMin(_ub));
}

inline Vector Vector::clamp(const Vector& _lb, const Vector& _ub) const
{
  return Vector(data.cwiseMax(_lb.data).cwiseMin(_ub.data));
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

inline bool Vector::operator==(const Vector& _rhs) const
{
  return data.cwiseEqual(_rhs.data).all();
}

inline Vector& Vector::operator+=(const Vector& _arg)
{
  data += _arg.data;
  return *this;
}

inline Vector& Vector::operator-=(const Vector& _arg)
{
  data -= _arg.data;
  return *this;
}

inline Vector operator-(const Vector& _arg)
{
  return Vector(-_arg.data);
}

inline Vector operator+(const Vector& _lhs, double _rhs)
{
  return Vector(_lhs.data.array() + _rhs);
}

inline Vector operator+(double _lhs, const Vector& _rhs)
{
  return Vector(_lhs + _rhs.data.array());
}

inline Vector operator-(const Vector& _lhs, double _rhs)
{
  return Vector(_lhs.data.array() - _rhs);
}

inline Vector operator-(double _lhs, const Vector& _rhs)
{
  return Vector(_lhs - _rhs.data.array());
}

inline Vector operator*(const Vector& _lhs, double _rhs)
{
  return Vector(_lhs.data * _rhs);
}

inline Vector operator*(double _lhs, const Vector& _rhs)
{
  return Vector(_lhs * _rhs.data);
}

inline Vector operator/(const Vector& _lhs, double _rhs)
{
  assert(_rhs != 0);
  return Vector(_lhs.data / _rhs);
}

inline Vector operator/(double _lhs, const Vector& _rhs)
{
  return _lhs * _rhs.inverse();
}

inline Vector operator+(const Vector& _lhs, const Vector& _rhs)
{
  return Vector(_lhs.data + _rhs.data);
}

inline Vector operator-(const Vector& _lhs, const Vector& _rhs)
{
  return Vector(_lhs.data - _rhs.data);
}

inline Vector operator*(const Vector& _lhs, const Vector& _rhs)
{
  return Vector(_lhs.data.cross(_rhs.data));
}

inline std::ostream& operator<<(std::ostream& _os, const Vector& _arg)
{
  _os << _arg.data.transpose();
  return _os;
}
}  // namespace kdl
}  // namespace tobas
