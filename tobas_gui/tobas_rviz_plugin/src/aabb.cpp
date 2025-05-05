#include "../include/tobas_rviz_plugin/aabb.hpp"

#include <geometric_shapes/check_isometry.h>

namespace tobas
{
void AABB::extendWithTransformedBox(const Eigen::Isometry3d& transform, const Eigen::Vector3d& box)
{
  // Method adapted from FCL src/shape/geometric_shapes_utility.cpp#computeBV<AABB, Box>(...) (BSD-licensed code):
  // https://github.com/flexible-collision-library/fcl/blob/fcl-0.4/src/shape/geometric_shapes_utility.cpp#L292
  // We don't call their code because it would need creating temporary objects, and their method is in floats.
  //
  // Here's a nice explanation why it works: https://zeuxcg.org/2010/10/17/aabb-from-obb-with-component-wise-abs/

  ASSERT_ISOMETRY(transform)  // unsanitized input, could contain non-isometry
  const Eigen::Matrix3d& r = transform.linear();
  const Eigen::Vector3d& t = transform.translation();

  double x_range = 0.5 * (fabs(r(0, 0) * box[0]) + fabs(r(0, 1) * box[1]) + fabs(r(0, 2) * box[2]));
  double y_range = 0.5 * (fabs(r(1, 0) * box[0]) + fabs(r(1, 1) * box[1]) + fabs(r(1, 2) * box[2]));
  double z_range = 0.5 * (fabs(r(2, 0) * box[0]) + fabs(r(2, 1) * box[1]) + fabs(r(2, 2) * box[2]));

  const Eigen::Vector3d v_delta(x_range, y_range, z_range);
  extend(t + v_delta);
  extend(t - v_delta);
}
}  // namespace tobas
