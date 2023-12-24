#pragma once

#include "./accel.hpp"
#include "./wrench.hpp"

namespace KDL
{
/**
 * @brief セグメントに対し，ジョイントの単位加速度あたりに発生する力．
 */
class SegmentInertia
{
public:
  Vector linear;   // [kg m] (Revolute) or [kg] (Prismatic)
  Vector angular;  // [kg m^2] (Revolute) or 0 (Prismatic)

  inline explicit SegmentInertia();
  inline explicit SegmentInertia(const Vector& linear, const Vector& angular);

  inline static SegmentInertia Zero();
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
}  // namespace KDL
