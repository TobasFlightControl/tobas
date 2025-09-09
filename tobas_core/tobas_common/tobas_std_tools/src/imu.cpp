#include "tobas_std_tools/imu.hpp"

#include <tobas_math/core.hpp>

namespace tobas_std
{
void eulerFromAccelMag(
  const double& ax,
  const double& ay,
  const double& az,
  const double& mx,
  const double& my,
  const double& mz,
  const double& mx_ref,
  const double& my_ref,
  const double& mz_ref,
  double& roll,
  double& pitch,
  double& yaw)
{
  (void)mz_ref;  // Avoid compile error

  roll = atan2(ay, az);
  pitch = atan2(ax, sqrt(math::sqr(ay) + math::sqr(az)));

  const auto x = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
  const auto y = my * cos(roll) - mz * sin(roll);
  yaw = atan2(my_ref * x - mx_ref * y, mx_ref * x + my_ref * y);
}
}  // namespace tobas_std
