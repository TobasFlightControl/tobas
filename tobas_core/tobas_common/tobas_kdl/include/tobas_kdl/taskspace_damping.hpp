// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./accel.hpp"
#include "./frame.hpp"
#include "./twist.hpp"

namespace tobas::kdl
{
/** represents both linear and angular damping. */
class TaskSpaceDamping
{
public:
  Vector linear;   ///< [Ns/m]
  Vector angular;  ///< [Nms/rad]

  inline explicit TaskSpaceDamping();
  inline explicit TaskSpaceDamping(const Vector& _linear, const Vector& _angular);

  static inline TaskSpaceDamping Zero();

  inline void setZero();

  inline Twist operator*(const Frame& _rhs);
  inline Accel operator*(const Twist& _rhs);
};

inline TaskSpaceDamping::TaskSpaceDamping()
{
}

inline TaskSpaceDamping::TaskSpaceDamping(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline TaskSpaceDamping TaskSpaceDamping::Zero()
{
  return TaskSpaceDamping(Vector::Zero(), Vector::Zero());
}

inline void TaskSpaceDamping::setZero()
{
  linear.setZero();
  angular.setZero();
}

inline Twist TaskSpaceDamping::operator*(const Frame& _rhs)
{
  return Twist(linear.hadamard(_rhs.p), angular.hadamard(_rhs.M.getRot()));
}

inline Accel TaskSpaceDamping::operator*(const Twist& _rhs)
{
  return Accel(linear.hadamard(_rhs.vel), angular.hadamard(_rhs.rot));
}
}  // namespace tobas::kdl
