// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./accel.hpp"
#include "./segment_inertia.hpp"
#include "./wrench.hpp"

namespace tobas::kdl
{
/** Jacobian for one segment. */
class SegmentJacobian
{
public:
  Vector linear;   ///< [m] (Revolute) or [-] (Prismatic)
  Vector angular;  ///< [-] (Revolute) or 0 (Prismatic)

  inline explicit SegmentJacobian();
  inline explicit SegmentJacobian(const Vector& _linear, const Vector& _angular);

  static inline SegmentJacobian Zero();

  inline void setZero();

  inline Eigen::Vector6d ravel() const;

  inline SegmentJacobian refPoint(const Vector& _p) const;

  /** Compute task-space acceleration from joint-space acceleration. */
  inline Accel accel(double _qdd) const;

  /** Compute the force [N or Nm] acting on the joint. */
  inline double dot(const Wrench& _rhs) const;
  /** Compute inertia [kg m^2 or kg] in joint space. */
  inline double dot(const SegmentInertia& _rhs) const;
};

inline SegmentJacobian::SegmentJacobian()
{
}

inline SegmentJacobian::SegmentJacobian(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline SegmentJacobian SegmentJacobian::Zero()
{
  return SegmentJacobian(Vector::Zero(), Vector::Zero());
}

inline void SegmentJacobian::setZero()
{
  linear.setZero();
  angular.setZero();
}

inline Eigen::Vector6d SegmentJacobian::ravel() const
{
  return (Eigen::Vector6d() << linear.data, angular.data).finished();
}

inline SegmentJacobian SegmentJacobian::refPoint(const Vector& _p) const
{
  return SegmentJacobian(linear + angular * _p, angular);
}

inline Accel SegmentJacobian::accel(double _qdd) const
{
  return Accel(linear * _qdd, angular * _qdd);
}

inline double SegmentJacobian::dot(const Wrench& _rhs) const
{
  return linear.dot(_rhs.force) + angular.dot(_rhs.torque);
}

inline double SegmentJacobian::dot(const SegmentInertia& _rhs) const
{
  return linear.dot(_rhs.linear) + angular.dot(_rhs.angular);
}
}  // namespace tobas::kdl
