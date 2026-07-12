// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_math/linalg.hpp>

#include "./accel.hpp"
#include "./segment_jacobian.hpp"
#include "./twist.hpp"
#include "./vector.hpp"
#include "./wrench.hpp"

namespace tobas
{
namespace kdl
{
class Rotation;
using RotationMap = std::map<std::string, Rotation>;

class Rotation
{
public:
  Eigen::Matrix3d data;

  inline explicit Rotation();
  inline explicit Rotation(
    double xx,
    double yx,
    double zx,
    double xy,
    double yy,
    double zy,
    double xz,
    double yz,
    double zz);
  inline explicit Rotation(const std::array<double, 9>& _data);
  inline explicit Rotation(const Vector& x, const Vector& y, const Vector& z);
  inline explicit Rotation(const Eigen::Matrix3d& _data);

  /* Gives back an identity rotation matrix. */
  static inline Rotation Identity();

  /* The Rot... static functions give the value of the appropriate rotation matrix back. */
  static Rotation RotX(double angle);
  /* The Rot... static functions give the value of the appropriate rotation matrix back. */
  static Rotation RotY(double angle);
  /* The Rot... static functions give the value of the appropriate rotation matrix back. */
  static Rotation RotZ(double angle);

  /**
   * @brief Along an arbitrary axes. Axis must be normalized.
   * Returns the identity rotation matrix if the norm of the axis is too small to use.
   * R = std::exp(ω)
   */
  static Rotation Rot(const Vector& axis, double angle);
  static Rotation Rot(const Vector& vec);

  /* Gives back a rotation matrix specified with ZYX euler angles. */
  static Rotation RPY(double roll, double pitch, double yaw);

  /* Gives back a rotation matrix specified with Quaternion convention. */
  static Rotation Quaternion(double x, double y, double z, double w);

  /* Check validity. */
  bool isValid(std::string& error_msg) const;

  inline bool isFinite() const;

  inline void setIdentity();
  inline void setInverse();
  inline void setNaN();

  /* Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set. */
  inline double& operator()(int i, int j);
  /* Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set. */
  inline double operator()(int i, int j) const;

  inline Rotation operator*(const Rotation& rhs) const;
  inline Vector operator*(const Vector& rhs) const;
  inline Twist operator*(const Twist& rhs) const;
  inline Accel operator*(const Accel& rhs) const;
  inline Wrench operator*(const Wrench& rhs) const;
  inline SegmentJacobian operator*(const SegmentJacobian& rhs) const;

  /* Compute the difference of two rotations wrt. the same frame. */
  inline Rotation operator-(const Rotation& rhs) const;

  /* Gives back the inverse rotation matrix of *this. */
  inline Rotation inverse() const;
  /* The same as R.inverse()*v but more efficient. */
  inline Vector inverse(const Vector& v) const;
  /* The same as R.inverse()*arg but more efficient. */
  inline Twist inverse(const Twist& arg) const;
  /* The same as R.inverse()*arg but more efficient. */
  inline Accel inverse(const Accel& arg) const;
  /* The same as R.inverse()*arg but more efficient. */
  inline Wrench inverse(const Wrench& arg) const;
  /* The same as R.inverse()*arg but more efficient. */
  inline SegmentJacobian inverse(const SegmentJacobian& arg) const;

  /* X axis of the child frame wrt. the parent frame. The same as R*(1,0,0). */
  inline Vector axisX() const;
  /* Y axis of the child frame wrt. the parent frame. The same as R*(0,1,0). */
  inline Vector axisY() const;
  /* Z axis of the child frame wrt. the parent frame. The same as R*(0,0,1). */
  inline Vector axisZ() const;

  /**
   * @brief Returns a vector with the direction of the equiv. axis and its norm is angle.
   * ω = log(R)
   */
  Vector getRot() const;

  /* Returns the rotation angle around the equiv. axis. */
  std::pair<double, Vector> getAngleAxis() const;

  /* Get the quaternion of this matrix. */
  void getQuaternion(double& x, double& y, double& z, double& w) const;

  /* Get ZYX euler angles. */
  void getRPY(double& roll, double& pitch, double& yaw) const;
  std::tuple<double, double, double> getRPY() const;

  inline double getPitch() const;
  inline double getYaw() const;

  inline double trace() const;

  inline friend std::ostream& operator<<(std::ostream& os, const Rotation& arg);
};

inline Rotation::Rotation()
{
  *this = Rotation::Identity();
}

inline Rotation::Rotation(double xx, double yx, double zx, double xy, double yy, double zy, double xz, double yz, double zz)
{
  data << xx, yx, zx, xy, yy, zy, xz, yz, zz;
}

inline Rotation::Rotation(const std::array<double, 9>& _data)
  : Rotation(_data[0], _data[1], _data[2], _data[3], _data[4], _data[5], _data[6], _data[7], _data[8])
{
}

inline Rotation::Rotation(const Vector& x, const Vector& y, const Vector& z)
{
  data.col(0) = x.data;
  data.col(1) = y.data;
  data.col(2) = z.data;
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

inline double& Rotation::operator()(int i, int j)
{
  assert(0 <= i && i <= 2 && 0 <= j && j <= 2);
  return data(i, j);
}

inline double Rotation::operator()(int i, int j) const
{
  assert(0 <= i && i <= 2 && 0 <= j && j <= 2);
  return data(i, j);
}

inline Rotation Rotation::operator*(const Rotation& rhs) const
{
  return Rotation(data * rhs.data);  // TODO: Add processing to preserve SO3.
}

inline Vector Rotation::operator*(const Vector& rhs) const
{
  return Vector(data * rhs.data);
}

inline Twist Rotation::operator*(const Twist& rhs) const
{
  return Twist((*this) * rhs.vel, (*this) * rhs.rot);
}

inline Accel Rotation::operator*(const Accel& rhs) const
{
  return Accel((*this) * rhs.linear, (*this) * rhs.angular);
}

inline Wrench Rotation::operator*(const Wrench& rhs) const
{
  return Wrench((*this) * rhs.force, (*this) * rhs.torque);
}

inline SegmentJacobian Rotation::operator*(const SegmentJacobian& rhs) const
{
  return SegmentJacobian((*this) * rhs.linear, (*this) * rhs.angular);
}

inline Rotation Rotation::operator-(const Rotation& rhs) const
{
  return (*this) * rhs.inverse();
}

inline Rotation Rotation::inverse() const
{
  return Rotation(data.transpose());
}

inline Vector Rotation::inverse(const Vector& v) const
{
  return Vector(data.transpose() * v.data);
}

inline Twist Rotation::inverse(const Twist& arg) const
{
  return Twist(inverse(arg.vel), inverse(arg.rot));
}

inline Accel Rotation::inverse(const Accel& arg) const
{
  return Accel(inverse(arg.linear), inverse(arg.angular));
}

inline Wrench Rotation::inverse(const Wrench& arg) const
{
  return Wrench(inverse(arg.force), inverse(arg.torque));
}

inline SegmentJacobian Rotation::inverse(const SegmentJacobian& arg) const
{
  return SegmentJacobian(inverse(arg.linear), inverse(arg.angular));
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
  return std::atan2(-data(2, 0), math::norm(data(0, 0), data(1, 0)));
}

inline double Rotation::getYaw() const
{
  return std::atan2(data(1, 0), data(0, 0));
}

inline double Rotation::trace() const
{
  return data.trace();
}

inline std::ostream& operator<<(std::ostream& os, const Rotation& arg)
{
  os << arg.data.reshaped(1, 9);
  return os;
}
}  // namespace kdl
}  // namespace tobas
