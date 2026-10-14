// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_math/core.hpp>
#include <tobas_math/float.hpp>
#include <tobas_std_tools/geometry.hpp>

#include "./rotation.hpp"
#include "./vector.hpp"

namespace tobas
{
namespace kdl
{
class Quaternion
{
public:
  double x, y, z, w;

  inline explicit Quaternion(double _x, double _y, double _z, double _w);
  inline explicit Quaternion(const Rotation& _rot);
  inline explicit Quaternion();

  /** Identity `Quaternion`. */
  static inline Quaternion Identity();

  /** Create a `Quaternion` from an equivalent angle-axis `Vector`. */
  static inline Quaternion AngleAxis(const Vector& _a);

  /** Create a `Quaternion` from Euler angles. */
  static inline Quaternion RPY(double _roll, double _pitch, double _yaw);

  /** Convert a `Quaternion` to Euler angles. */
  inline void getRPY(double& _roll, double& _pitch, double& _yaw) const;

  /** Complex conjugate `Quaternion`. */
  inline Quaternion complexConjugate() const;

  /** Inverse `Quaternion`. */
  inline Quaternion inverse() const;

  /** Sum of squared elements. */
  inline double squaredNorm() const;

  /** L2 norm. */
  inline double norm() const;

  /** Normalize. */
  inline Quaternion normalize() const;

  /** Return true if this is a normalized `Quaternion`. */
  inline bool isNormalized() const;

  /** Time derivative of the `Quaternion`. Note that angular velocity is defined locally. */
  inline Quaternion differential(const Vector& _angvel) const;

  /** Divide all elements by a scalar. */
  inline Quaternion operator/(double _rhs) const;

  /** Composition of two rotations. */
  inline Quaternion operator*(const Quaternion& _rhs) const;

  /** Rotate a 3D `Vector`. */
  inline Vector operator*(const Vector& _v) const;

  inline friend std::ostream& operator<<(std::ostream& _os, const Quaternion& _arg);
};

inline Quaternion::Quaternion(double _x, double _y, double _z, double _w) : x(_x), y(_y), z(_z), w(_w)
{
}

inline Quaternion::Quaternion(const Rotation& _rot)
{
  _rot.getQuaternion(x, y, z, w);
}

inline Quaternion::Quaternion() : x(0), y(0), z(0), w(1)
{
}

inline Quaternion Quaternion::Identity()
{
  return Quaternion(0, 0, 0, 1);
}

inline Quaternion Quaternion::AngleAxis(const Vector& _a)
{
  const auto angle = _a.norm();

  if (angle < std::numeric_limits<double>::epsilon()) {
    return Quaternion::Identity();
  }

  const auto axis = _a / angle;
  const auto mag = std::sin(angle / 2);
  return Quaternion(mag * axis.x(), mag * axis.y(), mag * axis.z(), std::cos(angle / 2));
}

inline Quaternion Quaternion::RPY(double _roll, double _pitch, double _yaw)
{
  return Quaternion(Rotation::RPY(_roll, _pitch, _yaw));
}

inline void Quaternion::getRPY(double& _roll, double& _pitch, double& _yaw) const
{
  std::tie(_roll, _pitch, _yaw) = st::eulerFromQuaternion(x, y, z, w);
}

inline Quaternion Quaternion::complexConjugate() const
{
  return Quaternion(-x, -y, -z, w);
}

inline Quaternion Quaternion::inverse() const
{
  return complexConjugate() / squaredNorm();
}

inline double Quaternion::squaredNorm() const
{
  return math::sqr(x) + math::sqr(y) + math::sqr(z) + math::sqr(w);
}

inline double Quaternion::norm() const
{
  return std::sqrt(squaredNorm());
}

inline Quaternion Quaternion::normalize() const
{
  return *this / norm();
}

inline bool Quaternion::isNormalized() const
{
  return math::isClose(squaredNorm(), 1.0);
}

inline Quaternion Quaternion::differential(const Vector& _angvel) const
{
  const auto a = _angvel / 2;
  return *this * Quaternion(a.x(), a.y(), a.z(), 0);
}

inline Quaternion Quaternion::operator/(double _rhs) const
{
  assert(_rhs != 0);
  return Quaternion(x / _rhs, y / _rhs, z / _rhs, w / _rhs);
}

inline Quaternion Quaternion::operator*(const Quaternion& _rhs) const
{
  const auto nw = w * _rhs.w - x * _rhs.x - y * _rhs.y - z * _rhs.z;
  const auto nx = w * _rhs.x + x * _rhs.w + y * _rhs.z - z * _rhs.y;
  const auto ny = w * _rhs.y - x * _rhs.z + y * _rhs.w + z * _rhs.x;
  const auto nz = w * _rhs.z + x * _rhs.y - y * _rhs.x + z * _rhs.w;
  return Quaternion(nx, ny, nz, nw);
}

inline Vector Quaternion::operator*(const Vector& _v) const
{
  const auto res = *this * Quaternion(_v.x(), _v.y(), _v.z(), 0) * complexConjugate();
  return Vector(res.x, res.y, res.z);
}

inline std::ostream& operator<<(std::ostream& _os, const Quaternion& _arg)
{
  _os << "w: " << _arg.w << ", x: " << _arg.x << ", y: " << _arg.y << ", z: " << _arg.z;
  return _os;
}
}  // namespace kdl
}  // namespace tobas
