#include "tobas_eskf/util.hpp"

#include <tobas_math/core.hpp>

namespace eskf
{
double headingVarianceFromMag(const Eigen::Vector3d& mag, const Eigen::Matrix3d& cov)
{
  const auto mx = mag.x();
  const auto my = mag.y();
  const auto mx_std = sqrt(cov(0, 0));
  const auto my_std = sqrt(cov(1, 1));
  const auto head_std = (fabs(mx) * my_std + fabs(my) * mx_std) / (math::sqr(mx) + math::sqr(my));
  return math::sqr(head_std);
}
}  // namespace eskf
