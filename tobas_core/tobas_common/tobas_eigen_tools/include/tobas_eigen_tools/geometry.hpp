// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tobas_math/float.hpp>

namespace tobas::eigen
{
/** Convert a 3D vector between FLU (Front-Left-Up) and FRD (Front-Right-Down). Equivalent to applying Rx(pi). */
inline void vectorFrdToFlu(const Eigen::Vector3d& _src, Eigen::Vector3d& _dst)
{
  _dst.x() = _src.x();
  _dst.y() = -_src.y();
  _dst.z() = -_src.z();
}

inline void vectorFluToFrd(const Eigen::Vector3d& _src, Eigen::Vector3d& _dst)
{
  vectorFrdToFlu(_src, _dst);
}

inline void vectorFrdToFlu(Eigen::Vector3d& _arg)
{
  vectorFrdToFlu(_arg, _arg);
}

inline void vectorFluToFrd(Eigen::Vector3d& _arg)
{
  vectorFluToFrd(_arg, _arg);
}

inline Eigen::AngleAxisd angleAxisFromVector(const Eigen::Vector3d& _w)
{
  const auto angle = _w.norm();
  const auto axis = (angle == 0) ? Eigen::Vector3d::UnitX() : _w.normalized();
  return Eigen::AngleAxisd(angle, axis);
}

inline Eigen::Vector3d vectorFromAngleAxis(const Eigen::AngleAxisd& _angle_axis)
{
  return _angle_axis.angle() * _angle_axis.axis();
}

/** Create a rotation matrix from an equivalent angle-axis vector. */
inline Eigen::Matrix3d dcmFromAngleAxis(const Eigen::Vector3d& _w)
{
  return angleAxisFromVector(_w).toRotationMatrix();
}

/** Create a quaternion from an equivalent angle-axis vector. */
inline Eigen::Quaterniond quaternionFromAngleAxis(const Eigen::Vector3d& _w)
{
  return Eigen::Quaterniond(angleAxisFromVector(_w));
}

/** Create an equivalent angle-axis vector from a quaternion. */
inline Eigen::Vector3d angleAxisFromQuaternion(const Eigen::Quaterniond& _q)
{
  Eigen::AngleAxisd angle_axis(_q);
  return vectorFromAngleAxis(angle_axis);
}

inline Eigen::Quaterniond quaternionFromRPY(double _roll, double _pitch, double _yaw)
{
  const Eigen::AngleAxisd rot_yaw(_yaw, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd rot_pitch(_pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rot_roll(_roll, Eigen::Vector3d::UnitX());
  return rot_yaw * rot_pitch * rot_roll;
}

inline Eigen::Matrix3d dcmFromRPY(double _roll, double _pitch, double _yaw)
{
  return quaternionFromRPY(_roll, _pitch, _yaw).toRotationMatrix();
}

inline double yawFromDCM(const Eigen::Matrix3d& _r)
{
  return std::atan2(_r(1, 0), _r(0, 0));
}

/** Convert Hamilton format (w,x,y,z) to `Eigen::Quaterniond`. */
inline Eigen::Quaterniond quaternionFromHamilton(const Eigen::Vector4d& _ham)
{
  return Eigen::Quaterniond((Eigen::Vector4d() << _ham.tail<3>(), _ham.head<1>()).finished());
}

/** Convert `Eigen::Quaterniond` to Hamilton format (w,x,y,z). */
inline Eigen::Vector4d hamiltonFromQuaternion(const Eigen::Quaterniond& _q)
{
  // Quaternion elements are not guaranteed to be contiguous in memory,
  // so calling `coeffs` may cause a compilation error.
  return Eigen::Vector4d(_q.w(), _q.x(), _q.y(), _q.z());
}

/** Create the matrix equivalent to a vector cross product. */
inline Eigen::Matrix3d skew(double _x, double _y, double _z)
{
  return (Eigen::Matrix3d() << 0, -_z, _y, _z, 0, -_x, -_y, _x, 0).finished();
}

/** Create the matrix equivalent to a vector cross product. */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& _v)
{
  return skew(_v(0), _v(1), _v(2));
}

/** Create the square of the matrix equivalent to a vector cross product. */
inline Eigen::Matrix3d skew2(double _x, double _y, double _z)
{
  const auto xx = _x * _x;
  const auto yy = _y * _y;
  const auto zz = _z * _z;
  const auto xy = _x * _y;
  const auto yz = _y * _z;
  const auto zx = _z * _x;

  return (Eigen::Matrix3d() << -(yy + zz), xy, zx, xy, -(zz + xx), yz, zx, yz, -(xx + yy)).finished();
}

/** Create the square of the matrix equivalent to a vector cross product. */
inline Eigen::Matrix3d skew2(const Eigen::Vector3d& _v)
{
  return skew2(_v(0), _v(1), _v(2));
}

/** Calculate the foot of the perpendicular from `p` to the line through `x0` in direction `v` (||`v`|| = 1). */
inline Eigen::Vector3d
projectPointOnToLine(const Eigen::Vector3d& _x0, const Eigen::Vector3d& _v, const Eigen::Vector3d& _p)
{
  assert(math::isClose(_v.norm(), 1.0));
  return _x0 + _v.dot(_p - _x0) * _v;
}
}  // namespace tobas::eigen
