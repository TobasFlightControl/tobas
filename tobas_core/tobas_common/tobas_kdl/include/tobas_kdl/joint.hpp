// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/joint.hpp
// Modified by Tobas, Inc.

// Copyright  (C)  2007  Ruben Smits <ruben dot smits at intermodalics dot eu>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at intermodalics dot eu>
// Maintainer: Ruben Smits <ruben dot smits at intermodalics dot eu>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#pragma once

#include <expected>
#include <string>

#include <tobas_eigen_tools/geometry.hpp>

#include "./frames.hpp"
#include "./segment_jacobian.hpp"

namespace tobas::kdl
{
/**
 * This class encapsulates a simple joint,
 * that is with one parameterized degree of freedom and with scalar dynamic properties.
 */
class Joint
{
public:
  enum JointType
  {
    kRotation,
    kTranslation,
    kFixed,
  };

  std::string name = "";
  Joint::JointType type = kFixed;
  Vector origin = Vector::Zero();  ///< The position of the drive shaft wrt. the parent frame
  double damping = 0.0;
  double friction = 0.0;
  double lower_limit = -INFINITY;
  double upper_limit = INFINITY;
  double max_effort = INFINITY;
  double max_velocity = INFINITY;

  inline explicit Joint();

  /** Check validity (except for the root joint). */
  std::expected<void, std::string> validate() const;

  /** Get the normalized joint axis wrt. the parent frame. */
  inline const Vector& axis() const;

  /** Set joint axis. */
  inline void axis(const Vector& _axis);

  /** Request the 6D-pose of the end of the joint wrt. the parent frame. */
  inline Frame pose(double _q) const;

  /** Request the resulting 6D-velocity of the end of the joint wrt. the parent frame. */
  inline Twist twist(double _qd) const;

  /** Request the resulting 6D-acceleration of the end of the joint wrt. the parent frame. */
  inline Accel accel(double _qdd) const;

  /** Request the jacobian for this joint wrt. the parent frame. */
  inline SegmentJacobian jacobian() const;

  /** Compute the first derivative of the rotation matrix with respect to the joint position. */
  inline Eigen::Matrix3d rotGrad(double _q) const;

  /** Compute the second derivative of the rotation matrix with respect to the joint position. */
  inline Eigen::Matrix3d rotGrad2(double _q) const;

  static const char* typeToText(JointType _type);

  friend std::ostream& operator<<(std::ostream& _os, const Joint& _arg);

private:
  Vector axis_ = Vector::UnitZ();  ///< The normalized joint axis wrt. the parent frame.
};

inline Joint::Joint()
{
}

inline const Vector& Joint::axis() const
{
  return axis_;
}

inline void Joint::axis(const Vector& _axis)
{
  assert(_axis.norm() > 0);
  axis_ = _axis.normalized();
}

inline Frame Joint::pose(double _q) const
{
  switch (type) {
    case kRotation:
      return Frame(Rotation::Rot(axis_, _q), origin);
    case kTranslation:
      return Frame(origin + (axis_ * _q));
    case kFixed:
      return Frame::Identity();
    default:
      throw;
  }
}

inline Twist Joint::twist(double _qd) const
{
  switch (type) {
    case kRotation:
      return Twist(Vector::Zero(), axis_ * _qd);
    case kTranslation:
      return Twist(axis_ * _qd, Vector::Zero());
    case kFixed:
      return Twist::Zero();
    default:
      throw;
  }
}

inline Accel Joint::accel(double _qdd) const
{
  switch (type) {
    case kRotation:
      return Accel(Vector::Zero(), axis_ * _qdd);
    case kTranslation:
      return Accel(axis_ * _qdd, Vector::Zero());
    case kFixed:
      return Accel::Zero();
    default:
      throw;
  }
}

inline SegmentJacobian Joint::jacobian() const
{
  switch (type) {
    case kRotation:
      return SegmentJacobian(Vector::Zero(), axis_);
    case kTranslation:
      return SegmentJacobian(axis_, Vector::Zero());
    case kFixed:
      return SegmentJacobian::Zero();
    default:
      throw;
  }
}

inline Eigen::Matrix3d Joint::rotGrad(double _q) const
{
  if (type == kRotation) {
    return eigen::skew(axis_.data) * Rotation::Rot(axis_, _q).data;
  }
  else {
    return Eigen::Matrix3d::Zero();
  }
}

inline Eigen::Matrix3d Joint::rotGrad2(double _q) const
{
  if (type == kRotation) {
    return eigen::skew2(axis_.data) * Rotation::Rot(axis_, _q).data;
  }
  else {
    return Eigen::Matrix3d::Zero();
  }
}
}  // namespace tobas::kdl
