// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./accel.hpp"
#include "./segment_jacobian.hpp"
#include "./twist.hpp"
#include "./vector.hpp"
#include "./wrench.hpp"

namespace tobas::kdl
{
class Rotation;
using RotationMap = std::map<std::string, Rotation>;

class Rotation
{
public:
  Eigen::Matrix3d data;

  inline explicit Rotation();
  inline explicit Rotation(
    double _xx,
    double _yx,
    double _zx,
    double _xy,
    double _yy,
    double _zy,
    double _xz,
    double _yz,
    double _zz);
  inline explicit Rotation(const std::array<double, 9>& _data);
  inline explicit Rotation(const Vector& _x, const Vector& _y, const Vector& _z);
  inline explicit Rotation(const Eigen::Matrix3d& _data);

  /** Gives back an identity rotation matrix. */
  static inline Rotation Identity();

  /** The Rot... static functions give the value of the appropriate rotation matrix back. */
  static Rotation RotX(double _angle);
  /** The Rot... static functions give the value of the appropriate rotation matrix back. */
  static Rotation RotY(double _angle);
  /** The Rot... static functions give the value of the appropriate rotation matrix back. */
  static Rotation RotZ(double _angle);

  /**
   * Along an arbitrary axes. Axis must be normalized.
   * Returns the identity rotation matrix if the norm of the axis is too small to use.
   * R = std::exp(ω)
   */
  static Rotation Rot(const Vector& _axis, double _angle);
  static Rotation Rot(const Vector& _vec);

  /** Gives back a rotation matrix specified with ZYX euler angles. */
  static Rotation RPY(double _roll, double _pitch, double _yaw);

  /** Gives back a rotation matrix specified with Quaternion convention. */
  static Rotation Quaternion(double _x, double _y, double _z, double _w);

  /** Check validity. */
  std::expected<void, std::string> validate() const;

  inline bool isFinite() const;

  inline void setIdentity();
  inline void setInverse();
  inline void setNaN();

  /** Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set. */
  inline double& operator()(int _i, int _j);
  /** Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set. */
  inline double operator()(int _i, int _j) const;

  inline Rotation operator*(const Rotation& _rhs) const;
  inline Vector operator*(const Vector& _rhs) const;
  inline Twist operator*(const Twist& _rhs) const;
  inline Accel operator*(const Accel& _rhs) const;
  inline Wrench operator*(const Wrench& _rhs) const;
  inline SegmentJacobian operator*(const SegmentJacobian& _rhs) const;

  /** Compute the difference of two rotations wrt. the same frame. */
  inline Rotation operator-(const Rotation& _rhs) const;

  /** Gives back the inverse rotation matrix of *this. */
  inline Rotation inverse() const;
  /** The same as R.inverse()*v but more efficient. */
  inline Vector inverse(const Vector& _v) const;
  /** The same as R.inverse()*arg but more efficient. */
  inline Twist inverse(const Twist& _arg) const;
  /** The same as R.inverse()*arg but more efficient. */
  inline Accel inverse(const Accel& _arg) const;
  /** The same as R.inverse()*arg but more efficient. */
  inline Wrench inverse(const Wrench& _arg) const;
  /** The same as R.inverse()*arg but more efficient. */
  inline SegmentJacobian inverse(const SegmentJacobian& _arg) const;

  /** X axis of the child frame wrt. the parent frame. The same as R*(1,0,0). */
  inline Vector axisX() const;
  /** Y axis of the child frame wrt. the parent frame. The same as R*(0,1,0). */
  inline Vector axisY() const;
  /** Z axis of the child frame wrt. the parent frame. The same as R*(0,0,1). */
  inline Vector axisZ() const;

  /**
   * Returns a vector with the direction of the equiv. axis and its norm is angle.
   * ω = log(R)
   */
  Vector getRot() const;

  /** Returns the rotation angle around the equiv. axis. */
  std::pair<double, Vector> getAngleAxis() const;

  /** Get the quaternion of this matrix. */
  void getQuaternion(double& _x, double& _y, double& _z, double& _w) const;

  /** Get ZYX euler angles. */
  void getRPY(double& _roll, double& _pitch, double& _yaw) const;
  std::tuple<double, double, double> getRPY() const;

  inline double getPitch() const;
  inline double getYaw() const;

  inline double trace() const;

  inline friend std::ostream& operator<<(std::ostream& _os, const Rotation& _arg);
};

inline Rotation::Rotation()
{
  *this = Rotation::Identity();
}

inline Rotation::Rotation(
  double _xx,
  double _yx,
  double _zx,
  double _xy,
  double _yy,
  double _zy,
  double _xz,
  double _yz,
  double _zz)
{
  data << _xx, _yx, _zx, _xy, _yy, _zy, _xz, _yz, _zz;
}

inline Rotation::Rotation(const std::array<double, 9>& _data)
  : Rotation(_data[0], _data[1], _data[2], _data[3], _data[4], _data[5], _data[6], _data[7], _data[8])
{
}

inline Rotation::Rotation(const Vector& _x, const Vector& _y, const Vector& _z)
{
  data.col(0) = _x.data;
  data.col(1) = _y.data;
  data.col(2) = _z.data;
}

inline Rotation::Rotation(const Eigen::Matrix3d& _data) : data(_data)
{
}

inline Rotation Rotation::Identity()
{
  return Rotation(Eigen::Matrix3d::Identity());
}

bool Rotation::isFinite() const
{
  return eigen::isFinite(data);
}

inline void Rotation::setIdentity()
{
  data.setIdentity();
}

inline void Rotation::setInverse()
{
  data.transposeInPlace();
}

inline void Rotation::setNaN()
{
  data.setConstant(NAN);
}

inline double& Rotation::operator()(int _i, int _j)
{
  assert(0 <= _i && _i <= 2 && 0 <= _j && _j <= 2);
  return data(_i, _j);
}

inline double Rotation::operator()(int _i, int _j) const
{
  assert(0 <= _i && _i <= 2 && 0 <= _j && _j <= 2);
  return data(_i, _j);
}

inline Rotation Rotation::operator*(const Rotation& _rhs) const
{
  return Rotation(data * _rhs.data);  // TODO: Add processing to preserve SO3.
}

inline Vector Rotation::operator*(const Vector& _rhs) const
{
  return Vector(data * _rhs.data);
}

inline Twist Rotation::operator*(const Twist& _rhs) const
{
  return Twist((*this) * _rhs.vel, (*this) * _rhs.rot);
}

inline Accel Rotation::operator*(const Accel& _rhs) const
{
  return Accel((*this) * _rhs.linear, (*this) * _rhs.angular);
}

inline Wrench Rotation::operator*(const Wrench& _rhs) const
{
  return Wrench((*this) * _rhs.force, (*this) * _rhs.torque);
}

inline SegmentJacobian Rotation::operator*(const SegmentJacobian& _rhs) const
{
  return SegmentJacobian((*this) * _rhs.linear, (*this) * _rhs.angular);
}

inline Rotation Rotation::operator-(const Rotation& _rhs) const
{
  return (*this) * _rhs.inverse();
}

inline Rotation Rotation::inverse() const
{
  return Rotation(data.transpose());
}

inline Vector Rotation::inverse(const Vector& _v) const
{
  return Vector(data.transpose() * _v.data);
}

inline Twist Rotation::inverse(const Twist& _arg) const
{
  return Twist(inverse(_arg.vel), inverse(_arg.rot));
}

inline Accel Rotation::inverse(const Accel& _arg) const
{
  return Accel(inverse(_arg.linear), inverse(_arg.angular));
}

inline Wrench Rotation::inverse(const Wrench& _arg) const
{
  return Wrench(inverse(_arg.force), inverse(_arg.torque));
}

inline SegmentJacobian Rotation::inverse(const SegmentJacobian& _arg) const
{
  return SegmentJacobian(inverse(_arg.linear), inverse(_arg.angular));
}

inline Vector Rotation::axisX() const
{
  return kdl::Vector(data.col(0));
}

inline Vector Rotation::axisY() const
{
  return kdl::Vector(data.col(1));
}

inline Vector Rotation::axisZ() const
{
  return kdl::Vector(data.col(2));
}

inline double Rotation::getPitch() const
{
  return std::atan2(-data(2, 0), std::hypot(data(0, 0), data(1, 0)));
}

inline double Rotation::getYaw() const
{
  return std::atan2(data(1, 0), data(0, 0));
}

inline double Rotation::trace() const
{
  return data.trace();
}

inline std::ostream& operator<<(std::ostream& _os, const Rotation& _arg)
{
  _os << _arg.data.reshaped(1, 9);
  return _os;
}
}  // namespace tobas::kdl
