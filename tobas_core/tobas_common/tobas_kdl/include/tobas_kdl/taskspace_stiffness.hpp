// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./accel.hpp"
#include "./frame.hpp"

namespace tobas::kdl
{
/** represents both linear and angular stiffness. */
class TaskSpaceStiffness
{
public:
  Vector linear;   ///< [N/m]
  Vector angular;  ///< [Nm/rad] Elastic coefficient for the equivalent angle-axis vector

  inline explicit TaskSpaceStiffness();
  inline explicit TaskSpaceStiffness(const Vector& _linear, const Vector& _angular);

  static inline TaskSpaceStiffness Zero();

  inline void setZero();

  inline Accel operator*(const Frame& _rhs);
};

inline TaskSpaceStiffness::TaskSpaceStiffness()
{
}

inline TaskSpaceStiffness::TaskSpaceStiffness(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline TaskSpaceStiffness TaskSpaceStiffness::Zero()
{
  return TaskSpaceStiffness(Vector::Zero(), Vector::Zero());
}

inline void TaskSpaceStiffness::setZero()
{
  linear.setZero();
  angular.setZero();
}

inline Accel TaskSpaceStiffness::operator*(const Frame& _rhs)
{
  return Accel(linear.hadamard(_rhs.p), angular.hadamard(_rhs.M.getRot()));
}
}  // namespace tobas::kdl
