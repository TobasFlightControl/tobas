#pragma once

#include <kdl/tree.hpp>
#include <Eigen/Core>
#include <kdl/tree.hpp>

#include <tobas_tools/drone.hpp>

namespace tobas_multirotor_controller
{
class AccelerationController
{
public:
  explicit AccelerationController(const Drone& drone, const KDL::Tree& tree, double gravity);

  void update(
    const Eigen::Vector3d& tar_acc,
    const double& yaw,
    double& U_out,
    double& roll_out,
    double& pitch_out);

  const double& maxU() const;

private:
  const double gravity_;
  double mass_;
  double max_U_;
};
}  // namespace tobas_multirotor_controller
