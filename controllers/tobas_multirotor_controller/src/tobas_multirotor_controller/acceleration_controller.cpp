#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include "../../include/tobas_multirotor_controller/acceleration_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_multirotor_controller
{
AccelerationController::AccelerationController(const Drone& drone, double gravity)
  : gravity_(gravity)
{
  TreeJntToInertiaSolver inertia_solver_(drone.tree());
  mass_ = inertia_solver_.JntToMass();

  max_U_ = 0.;
  for (const auto& rotor_idx : drone.rotorConfigIdxInAxis(Axis::Z_POSITIVE))
  {
    max_U_ += drone.maxThrust(rotor_idx);
  }
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
  const double z = mass_ * (tar_acc.z() + gravity_);

  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);

  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = z / (cos(pitch_out) * cos(roll_out));
}

const double& AccelerationController::maxU() const
{
  return max_U_;
}
}  // namespace tobas_multirotor_controller
