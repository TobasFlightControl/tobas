#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include "../../include/tobas_multirotor_controller/acceleration_controller.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

AccelerationController::AccelerationController(
  const KDL::Tree& tree,
  double gravity,
  double battery_voltage,
  const RotorConfigs& rotor_configs)
  : gravity_(gravity), rotor_configs_(rotor_configs)
{
  TreeJntToInertiaSolver inertia_solver_(tree);
  mass_ = inertia_solver_.JntToMass();

  max_U_ = 0.;
  for (const auto& rotor_config : rotor_configs_)
  {
    const double max_speed = dh_std::rpmToRadPerSec(battery_voltage * rotor_config.kv);
    const double max_thrust = rotor_config.motor_constant * sqr(max_speed);
    max_U_ += max_thrust;
  }
}

void AccelerationController::update(
  const Vector3d& tar_acc,
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
