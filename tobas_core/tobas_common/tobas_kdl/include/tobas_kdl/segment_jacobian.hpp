// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./accel.hpp"
#include "./segment_inertia.hpp"
#include "./wrench.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Jacobian for one segment.
 */
class SegmentJacobian
{
public:
  Vector linear;   // [m] (Revolute) or [-] (Prismatic)
  Vector angular;  // [-] (Revolute) or 0 (Prismatic)

  inline explicit SegmentJacobian();
  inline explicit SegmentJacobian(const Vector& linear, const Vector& angular);

  static inline SegmentJacobian Zero();

  inline void setZero();

  inline Eigen::Vector6d ravel() const;

  inline SegmentJacobian refPoint(const Vector& p) const;

  /* Compute task-space acceleration from joint-space acceleration. */
  inline Accel accel(const double& qdd) const;

  /* Compute the force [N or Nm] acting on the joint. */
  inline double dot(const Wrench& rhs) const;
  /* Compute inertia [kg m^2 or kg] in joint space. */
  inline double dot(const SegmentInertia& rhs) const;
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

inline SegmentJacobian SegmentJacobian::refPoint(const Vector& p) const
{
  return SegmentJacobian(linear + angular * p, angular);
}

inline Accel SegmentJacobian::accel(const double& qdd) const
{
  return Accel(linear * qdd, angular * qdd);
}

inline double SegmentJacobian::dot(const Wrench& rhs) const
{
  return linear.dot(rhs.force) + angular.dot(rhs.torque);
}

inline double SegmentJacobian::dot(const SegmentInertia& rhs) const
{
  return linear.dot(rhs.linear) + angular.dot(rhs.angular);
}
}  // namespace kdl
}  // namespace tobas
