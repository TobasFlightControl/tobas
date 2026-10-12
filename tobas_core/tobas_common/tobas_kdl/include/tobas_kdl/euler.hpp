// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_std_tools/geometry.hpp>

#include "./angle_axis.hpp"
#include "./quaternion.hpp"
#include "./rotation.hpp"
#include "./vector.hpp"

namespace tobas
{
namespace kdl
{
/** ZYX Euler angles. */
class Euler
{
public:
  double roll, pitch, yaw;

  inline explicit Euler();
  inline explicit Euler(double _roll, double _pitch, double _yaw);
  inline explicit Euler(const Vector& _rpy);
  inline explicit Euler(const Rotation& _rot);

  static inline Euler Zero();

  inline Euler clone() const;

  inline void setZero();
  inline void setNaN();

  inline void set(double _roll, double _pitch, double _yaw);

  inline void fill(double _value);

  inline Rotation toRotation() const;
  inline Quaternion toQuaternion() const;
  inline Vector toAngleAxis() const;

  /**
   * Return the difference between two rotations as an equivalent angle-axis `Vector`.
   * `O_AngleAxis_AB = O_Rot_B - O_Rot_A`.
   */
  inline AngleAxis operator-(const Euler& _rhs) const;

  /** Rotate a 3D `Vector`. */
  inline Vector operator*(const Vector& _v) const;

  /** Apply the inverse rotation to a 3D `Vector`. */
  inline Vector inverse(const Vector& _v) const;

  inline bool isFinite() const;

  inline friend std::ostream& operator<<(std::ostream& _os, const Euler& _arg);
};

inline Euler::Euler() : roll(0), pitch(0), yaw(0)
{
}

inline Euler::Euler(double _roll, double _pitch, double _yaw) : roll(_roll), pitch(_pitch), yaw(_yaw)
{
}

inline Euler::Euler(const Vector& _rpy) : roll(_rpy.x()), pitch(_rpy.y()), yaw(_rpy.z())
{
}

inline Euler::Euler(const Rotation& _rot)
{
  _rot.getRPY(roll, pitch, yaw);
}

inline Euler Euler::Zero()
{
  return Euler(0.0, 0.0, 0.0);
}

inline Euler Euler::clone() const
{
  return *this;
}

inline void Euler::setZero()
{
  fill(0.0);
}

inline void Euler::setNaN()
{
  fill(NAN);
}

inline void Euler::set(double _roll, double _pitch, double _yaw)
{
  roll = _roll;
  pitch = _pitch;
  yaw = _yaw;
}

inline void Euler::fill(double _value)
{
  set(_value, _value, _value);
}

inline Rotation Euler::toRotation() const
{
  return Rotation::RPY(roll, pitch, yaw);
}

inline Vector Euler::toAngleAxis() const
{
  return toRotation().getRot();
}

inline Quaternion Euler::toQuaternion() const
{
  const auto [x, y, z, w] = st::quaternionFromEuler(roll, pitch, yaw);
  return Quaternion(x, y, z, w);
}

inline AngleAxis Euler::operator-(const Euler& _rhs) const
{
  return AngleAxis((toRotation() * _rhs.toRotation().inverse()).getRot());
}

inline Vector Euler::operator*(const Vector& _v) const
{
  return Rotation::RPY(roll, pitch, yaw) * _v;
}

inline Vector Euler::inverse(const Vector& _v) const
{
  return Rotation::RPY(roll, pitch, yaw).inverse(_v);
}

inline bool Euler::isFinite() const
{
  return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
}

inline std::ostream& operator<<(std::ostream& _os, const Euler& _arg)
{
  _os << "roll: " << _arg.roll << ", pitch: " << _arg.pitch << ", yaw: " << _arg.yaw;
  return _os;
}
}  // namespace kdl
}  // namespace tobas
