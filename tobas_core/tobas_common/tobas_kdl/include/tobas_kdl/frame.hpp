// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/frames.hpp
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/frames.inl
// Modified by Tobas, Inc.

/***************************************************************************
                        frames.hpp `-  description
                       -------------------------
    begin                : June 2006
    copyright            : (C) 2006 Erwin Aertbelien
    email                : firstname.lastname@mech.kuleuven.be

 History (only major changes)( AUTHOR-Description ) :

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#pragma once

#include <expected>
#include <string>

#include "./accel.hpp"
#include "./impulse.hpp"
#include "./rotation.hpp"
#include "./twist.hpp"
#include "./vector.hpp"
#include "./wrench.hpp"

namespace tobas::kdl
{
class Frame;
using FrameMap = std::map<std::string, Frame>;

/**
 * represents a frame transformation in 3D space (rotation + translation)
 *
 * If V2 = Frame*V1 (V2 expressed in frame A, V1 expressed in frame B)
 * then V2 = Frame.M*V1+Frame.p.
 *
 * Frame.M contains columns that represent the axes of frame B wrt frame A.
 * Frame.p contains the origin of frame B expressed in frame A.
 */
class Frame
{
public:
  Vector p;    ///< Origin of the frame
  Rotation M;  ///< Orientation of the frame

  inline explicit Frame(const Rotation& _r, const Vector& _v);
  /** The rotation matrix defaults to identity. */
  inline explicit Frame(const Vector& _v);
  /** The position defaults to zero. */
  inline explicit Frame(const Rotation& _r);
  /** The position defaults to zero and the rotation matrix defaults to identity. */
  inline explicit Frame();

  /** The identity transformation `Frame(Rotation::Identity(), Vector::Zero())` */
  static inline Frame Identity();

  /**
   * Constructs a transformation matrix T_link(i-1)_link(i)
   * with the Denavit-Hartenberg convention as described in the Craigs book:
   * Craig, J. J., Introduction to ROBOTICS: Mechanics and Control, Addison-Wesley, isbn:0-201-10326-5, 1986.
   *
   * Note that the frame is a redundant way to express the information in the DH-convention.
   * Parameters in full : a(i-1), alpha(i-1), d(i), theta(i)
   *
   * Axis (i-1) is connected by link (i-1) to axis (i)
   * numbering axis (1) to axis (n) link (0) (immobile base) to link (n).
   *
   * Link length a(i-1): Length of the mutual perpendicular line (normal) between the 2 axes.
   * This normal runs from (i-1) to (i) axis.
   *
   * Link twist alpha(i-1): Construct plane perpendicular to the normal project axis(i-1) and axis(i)
   * into plane angle from (i-1) to (i) measured in the direction of the normal.
   *
   * Link offset d(i): Signed distance between normal (i-1) to (i) and normal (i) to (i+1)
   * along axis (i) joint angle theta(i) signed angle
   * between normal (i-1) to (i) and normal (i) to (i+1) along axis (i)
   *
   * First and last joints : a(0)= a(n) = 0
   * alpha(0) = alpha(n) = 0
   *
   * PRISMATIC : theta(1) = 0 d(1) arbitrarily
   * REVOLUTE  : theta(1) arbitrarily d(1) = 0
   *
   * a(i-1)     = distance from Z(i-1) to Z(i) along X(i-1)
   * alpha(i-1) = angle between Z(i-1) to Z(i) along X(i-1)
   * d(i)       = distance from X(i-1) to X(i) along Z(i)
   * theta(i)   = angle between X(i-1) to X(i) along X(i)
   */
  static inline Frame DH_Craig1989(double _a, double _alpha, double _d, double _theta);

  /**
   * Constructs a transformationmatrix T_link(i-1)_link(i) with the Denavit-Hartenberg convention
   * as described in the original publictation:
   * Denavit, J. and Hartenberg, R. S.,
   * A kinematic notation for lower-pair mechanisms based on matrices,
   * ASME Journal of Applied Mechanics, 23:215-221, 1955.
   */
  static inline Frame DH(double _a, double _alpha, double _d, double _theta);

  static inline Frame XYZRPY(double _x, double _y, double _z, double _roll, double _pitch, double _yaw);

  inline std::expected<void, std::string> validate() const;

  inline Eigen::Matrix4d matrix() const;

  inline void setIdentity();
  inline void setNaN();

  /**
   * Treats a frame as a 4x4 matrix and returns element i,j.
   * Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set.
   */
  inline double operator()(int _i, int _j);
  /**
   * Treats a frame as a 4x4 matrix and returns element i,j.
   * Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set.
   */
  inline double operator()(int _i, int _j) const;

  /**
   * The twist <t_this> is expressed wrt the current frame.
   * This frame is integrated into an updated frame with <samplefrequency>.
   * Very simple first order integration rule.
   */
  inline void integrate(const Twist& _t_this, double _frequency);

  /** Convert the `Frame` to a 6D `Twist`. */
  inline Twist toTwist() const;

  inline Frame inverse() const;
  inline Vector inverse(const Vector& _arg) const;
  inline Twist inverse(const Twist& _arg) const;
  inline Accel inverse(const Accel& _arg) const;
  inline Wrench inverse(const Wrench& _arg) const;

  inline Vector operator*(const Vector& _arg) const;
  inline Wrench operator*(const Wrench& _arg) const;
  inline SegmentInertia operator*(const SegmentInertia& _arg) const;
  inline Twist operator*(const Twist& _arg) const;
  inline Accel operator*(const Accel& _arg) const;
  inline SegmentJacobian operator*(const SegmentJacobian& _arg) const;
  inline Frame operator*(const Frame& _rhs) const;

  /** Compute the difference of two frames wrt. the same frame. */
  inline Frame operator-(const Frame& _rhs) const;

  inline friend std::ostream& operator<<(std::ostream& _os, const Frame& _arg);
};

inline Frame::Frame(const Rotation& _r, const Vector& _v) : p(_v), M(_r)
{
}

inline Frame::Frame(const Rotation& _r) : p(Vector::Zero()), M(_r)
{
}

inline Frame::Frame(const Vector& _v) : p(_v), M(Rotation::Identity())
{
}

inline Frame::Frame()
{
}

inline Frame Frame::Identity()
{
  return Frame(Rotation::Identity(), Vector::Zero());
}

inline Frame Frame::DH_Craig1989(double _a, double _alpha, double _d, double _theta)
{
  const auto ct = std::cos(_theta);
  const auto st = std::sin(_theta);
  const auto sa = std::sin(_alpha);
  const auto ca = std::cos(_alpha);
  return Frame(Rotation(ct, -st, 0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca), Vector(_a, -sa * _d, ca * _d));
}

inline Frame Frame::DH(double _a, double _alpha, double _d, double _theta)
{
  const auto ct = std::cos(_theta);
  const auto st = std::sin(_theta);
  const auto sa = std::sin(_alpha);
  const auto ca = std::cos(_alpha);
  return Frame(Rotation(ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0, sa, ca), Vector(_a * ct, _a * st, _d));
}

inline Frame Frame::XYZRPY(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
  return Frame(Rotation::RPY(_roll, _pitch, _yaw), Vector(_x, _y, _z));
}

inline std::expected<void, std::string> Frame::validate() const
{
  return M.validate();
}

inline Eigen::Matrix4d Frame::matrix() const
{
  Eigen::Matrix4d res;
  res.topLeftCorner<3, 3>() = M.data;
  res.topRightCorner<3, 1>() = p.data;
  res.bottomLeftCorner<1, 3>().setZero();
  res(3, 3) = 1;
  return res;
}

inline void Frame::setIdentity()
{
  p.setZero();
  M.setIdentity();
}

inline void Frame::setNaN()
{
  p.setNaN();
  M.setNaN();
}

inline double Frame::operator()(int _i, int _j)
{
  assert((0 <= _i) && (_i <= 3) && (0 <= _j) && (_j <= 3));
  if (_i == 3) {
    if (_j == 3) {
      return 1.0;
    }
    else {
      return 0.0;
    }
  }
  else {
    if (_j == 3) {
      return p(_i);
    }
    else {
      return M(_i, _j);
    }
  }
}

inline double Frame::operator()(int _i, int _j) const
{
  assert((0 <= _i) && (_i <= 3) && (0 <= _j) && (_j <= 3));
  if (_i == 3) {
    if (_j == 3) {
      return 1.0;
    }
    else {
      return 0.0;
    }
  }
  else {
    if (_j == 3) {
      return p(_i);
    }
    else {
      return M(_i, _j);
    }
  }
}

inline void Frame::integrate(const Twist& _t_this, double _sampling_freq)
{
  assert(_sampling_freq > 0);

  const auto n = _t_this.rot.norm() / _sampling_freq;
  if (n < std::numeric_limits<double>::epsilon()) {
    p += M * (_t_this.vel / _sampling_freq);
  }
  else {
    (*this) = (*this) * Frame(Rotation::Rot(_t_this.rot, n), _t_this.vel / _sampling_freq);
  }
}

inline Twist Frame::toTwist() const
{
  return Twist(p, M.getRot());
}

inline Frame Frame::inverse() const
{
  return Frame(M.inverse(), -M.inverse(p));
}

inline Vector Frame::inverse(const Vector& _arg) const
{
  return M.inverse(_arg - p);
}

inline Twist Frame::inverse(const Twist& _arg) const
{
  Twist tmp;
  tmp.vel = M.inverse(_arg.vel - p * _arg.rot);
  tmp.rot = M.inverse(_arg.rot);
  return tmp;
}

inline Accel Frame::inverse(const Accel& _arg) const
{
  Accel tmp;
  tmp.linear = M.inverse(_arg.linear - p * _arg.angular);
  tmp.angular = M.inverse(_arg.angular);
  return tmp;
}

inline Wrench Frame::inverse(const Wrench& _arg) const
{
  Wrench tmp;
  tmp.force = M.inverse(_arg.force);
  tmp.torque = M.inverse(_arg.torque - p * _arg.force);
  return tmp;
}

inline Vector Frame::operator*(const Vector& _arg) const
{
  return M * _arg + p;
}

inline Wrench Frame::operator*(const Wrench& _arg) const
{
  Wrench tmp;
  tmp.force = M * _arg.force;
  tmp.torque = M * _arg.torque + p * tmp.force;
  return tmp;
}

inline SegmentInertia Frame::operator*(const SegmentInertia& _arg) const
{
  SegmentInertia tmp;
  tmp.linear = M * _arg.linear;
  tmp.angular = M * _arg.angular + p * tmp.linear;
  return tmp;
}

inline Twist Frame::operator*(const Twist& _arg) const
{
  Twist tmp;
  tmp.rot = M * _arg.rot;
  tmp.vel = M * _arg.vel + p * tmp.rot;
  return tmp;
}

inline Accel Frame::operator*(const Accel& _arg) const
{
  Accel tmp;
  tmp.angular = M * _arg.angular;
  tmp.linear = M * _arg.linear + p * tmp.angular;
  return tmp;
}

inline SegmentJacobian Frame::operator*(const SegmentJacobian& _arg) const
{
  SegmentJacobian tmp;
  tmp.angular = M * _arg.angular;
  tmp.linear = M * _arg.linear + p * tmp.angular;
  return tmp;
}

inline Frame Frame::operator*(const Frame& _rhs) const
{
  return Frame(M * _rhs.M, M * _rhs.p + p);
}

inline Frame Frame::operator-(const Frame& _rhs) const
{
  return Frame(M - _rhs.M, p - _rhs.p);
}

inline std::ostream& operator<<(std::ostream& _os, const Frame& _arg)
{
  _os << "Translation: " << _arg.p << ", Rotation: " << _arg.M;
  return _os;
}
}  // namespace tobas::kdl
