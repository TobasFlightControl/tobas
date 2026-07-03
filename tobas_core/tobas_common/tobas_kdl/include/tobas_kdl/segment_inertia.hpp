// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./accel.hpp"
#include "./wrench.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Force generated on a segment per unit joint acceleration.
 */
class SegmentInertia
{
public:
  Vector linear;   // [kg m] (Revolute) or [kg] (Prismatic)
  Vector angular;  // [kg m^2] (Revolute) or 0 (Prismatic)

  inline explicit SegmentInertia();
  inline explicit SegmentInertia(const Vector& linear, const Vector& angular);

  static inline SegmentInertia Zero();
};

inline SegmentInertia::SegmentInertia()
{
}

inline SegmentInertia::SegmentInertia(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline SegmentInertia SegmentInertia::Zero()
{
  return SegmentInertia(Vector::Zero(), Vector::Zero());
}
}  // namespace kdl
}  // namespace tobas
