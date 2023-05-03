#pragma once

#include <kdl/tree.hpp>
#include <Eigen/Core>
#include <kdl/tree.hpp>

#include <tobas_tools/rotor_property.hpp>

namespace tobas_multirotor_controller
{
class AccelerationController
{
public:
  explicit AccelerationController(
    const KDL::Tree& tree,
    double gravity,
    double battery_voltage,
    const RotorConfigs& rotor_configs);

  void update(
    const Eigen::Vector3d& tar_acc,
    const double& yaw,
    double& U_out,
    double& roll_out,
    double& pitch_out);

  const double& maxU() const;

private:
  const double gravity_;
  const RotorConfigs rotor_configs_;
  double mass_;
  double max_U_;
};
}  // namespace tobas_multirotor_controller
