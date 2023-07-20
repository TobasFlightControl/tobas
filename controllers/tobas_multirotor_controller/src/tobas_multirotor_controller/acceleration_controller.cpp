#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>

#include <tobas_tools/utils.hpp>

#include "../../include/tobas_multirotor_controller/acceleration_controller.hpp"
#include "../../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas_multirotor_controller
{
AccelerationController::AccelerationController(const tobas::Drone& drone)
  : drone_(drone), z_rotors_(drone, tobas::Axis::Z_POSITIVE)
{
  mass_ = tobas::getMass();
}

void AccelerationController::update(
  const Vector& tar_acc,
  const double& yaw,
  double& U_out,
  double& roll_out,
  double& pitch_out)
{
  const double x = mass_ * tar_acc.x();
  const double y = mass_ * tar_acc.y();
  const double z = mass_ * (tar_acc.z() + kGravity);

  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);

  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = z / (cos(pitch_out) * cos(roll_out));
}
}  // namespace tobas_multirotor_controller
