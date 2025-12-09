#include "tobas_std_tools/geometry.hpp"

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_math/float.hpp>

namespace tbs
{
void quaternionFromEuler(
  const double& roll,
  const double& pitch,
  const double& yaw,
  double& x,
  double& y,
  double& z,
  double& w)
{
  const auto cx = cos(0.5 * roll);
  const auto sx = sin(0.5 * roll);
  const auto cy = cos(0.5 * pitch);
  const auto sy = sin(0.5 * pitch);
  const auto cz = cos(0.5 * yaw);
  const auto sz = sin(0.5 * yaw);

  x = sx * cy * cz - cx * sy * sz;
  y = sx * cy * sz + cx * sy * cz;
  z = -sx * sy * cz + cx * cy * sz;
  w = sx * sy * sz + cx * cy * cz;
}

void eulerFromQuaternion(
  const double& x,
  const double& y,
  const double& z,
  const double& w,
  double& roll,
  double& pitch,
  double& yaw)
{
  assert(math::isClose(math::sqr(x) + math::sqr(y) + math::sqr(z) + math::sqr(w), 1.));

  const auto sy = -2 * (x * z - y * w);

  pitch = asin(sy);

  if (math::isClose(fabs(sy), 1.)) {
    roll = 0.;
    yaw = atan2(-2 * (x * y - z * w), 2 * (math::sqr(w) + math::sqr(y)) - 1);
  }
  else {
    roll = atan2(2 * (y * z + x * w), 2 * (math::sqr(w) + math::sqr(z)) - 1);
    yaw = atan2(2 * (x * y + z * w), 2 * (math::sqr(w) + math::sqr(x)) - 1);
  }
}
}  // namespace tbs
