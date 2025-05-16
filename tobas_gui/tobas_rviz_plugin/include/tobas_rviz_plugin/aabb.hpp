#pragma once

#include <eigen3/Eigen/Geometry>

namespace tobas
{
/* Represents an axis-aligned bounding box. */
class AABB : public Eigen::AlignedBox3d
{
public:
  /* Extend with a box transformed by the given transform. */
  void extendWithTransformedBox(const Eigen::Isometry3d& transform, const Eigen::Vector3d& box);
};
}  // namespace tobas
