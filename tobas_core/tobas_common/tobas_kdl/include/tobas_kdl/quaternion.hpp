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
  inline explicit Quaternion(const Rotation& rot);
  inline explicit Quaternion();

  /* Identity `Quaternion`. */
  static inline Quaternion Identity();

  /* Create a `Quaternion` from an equivalent angle-axis `Vector`. */
  static inline Quaternion AngleAxis(const Vector& a);

  /* Create a `Quaternion` from Euler angles. */
  static inline Quaternion RPY(double roll, double pitch, double yaw);

  /* Convert a `Quaternion` to Euler angles. */
  inline void getRPY(double& roll, double& pitch, double& yaw) const;

  /* Complex conjugate `Quaternion`. */
  inline Quaternion complexConjugate() const;

  /* Inverse `Quaternion`. */
  inline Quaternion inverse() const;

  /* Sum of squared elements. */
  inline double squaredNorm() const;

  /* L2 norm. */
  inline double norm() const;

  /* Normalize. */
  inline Quaternion normalize() const;

  /* Return true if this is a normalized `Quaternion`. */
  inline bool isNormalized() const;

  /* Time derivative of the `Quaternion`. Note that angular velocity is defined locally. */
  inline Quaternion differential(const Vector& angvel) const;

  /* Divide all elements by a scalar. */
  inline Quaternion operator/(double rhs) const;

  /* Composition of two rotations. */
  inline Quaternion operator*(const Quaternion& rhs) const;

  /* Rotate a 3D `Vector`. */
  inline Vector operator*(const Vector& v) const;

  inline friend std::ostream& operator<<(std::ostream& os, const Quaternion& arg);
};

inline Quaternion::Quaternion(double _x, double _y, double _z, double _w) : x(_x), y(_y), z(_z), w(_w)
{
}

inline Quaternion::Quaternion(const Rotation& rot)
{
  rot.getQuaternion(x, y, z, w);
}

inline Quaternion::Quaternion() : x(0), y(0), z(0), w(1)
{
}

inline Quaternion Quaternion::Identity()
{
  return Quaternion(0, 0, 0, 1);
}

inline Quaternion Quaternion::AngleAxis(const Vector& a)
{
  const auto angle = a.norm();

  if (angle < std::numeric_limits<double>::epsilon()) {
    return Quaternion::Identity();
  }

  const auto axis = a / angle;
  const auto mag = std::sin(angle / 2);
  return Quaternion(mag * axis.x(), mag * axis.y(), mag * axis.z(), std::cos(angle / 2));
}

inline Quaternion Quaternion::RPY(double roll, double pitch, double yaw)
{
  return Quaternion(Rotation::RPY(roll, pitch, yaw));
}

inline void Quaternion::getRPY(double& roll, double& pitch, double& yaw) const
{
  std::tie(roll, pitch, yaw) = st::eulerFromQuaternion(x, y, z, w);
}

inline Quaternion Quaternion::complexConjugate() const
{
  return Quaternion(-x, -y, -z, w);
}

inline Quaternion Quaternion::inverse() const
{
  return this->complexConjugate() / this->squaredNorm();
}

inline double Quaternion::squaredNorm() const
{
  return math::sqr(x) + math::sqr(y) + math::sqr(z) + math::sqr(w);
}

inline double Quaternion::norm() const
{
  return std::sqrt(this->squaredNorm());
}

inline Quaternion Quaternion::normalize() const
{
  return *this / this->norm();
}

inline bool Quaternion::isNormalized() const
{
  return math::isClose(this->squaredNorm(), 1.0);
}

inline Quaternion Quaternion::differential(const Vector& angvel) const
{
  const auto a = angvel / 2;
  return *this * Quaternion(a.x(), a.y(), a.z(), 0);
}

inline Quaternion Quaternion::operator/(double rhs) const
{
  assert(rhs != 0);
  return Quaternion(x / rhs, y / rhs, z / rhs, w / rhs);
}

inline Quaternion Quaternion::operator*(const Quaternion& rhs) const
{
  const auto nw = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
  const auto nx = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
  const auto ny = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
  const auto nz = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
  return Quaternion(nx, ny, nz, nw);
}

inline Vector Quaternion::operator*(const Vector& v) const
{
  const auto res = *this * Quaternion(v.x(), v.y(), v.z(), 0) * this->complexConjugate();
  return Vector(res.x, res.y, res.z);
}

inline std::ostream& operator<<(std::ostream& os, const Quaternion& arg)
{
  os << "w: " << arg.w << ", x: " << arg.x << ", y: " << arg.y << ", z: " << arg.z;
  return os;
}
}  // namespace kdl
}  // namespace tobas
