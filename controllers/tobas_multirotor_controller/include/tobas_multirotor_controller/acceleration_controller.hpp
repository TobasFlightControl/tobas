#pragma once

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>

namespace tobas_multirotor_controller
{
class AccelerationController
{
public:
  explicit AccelerationController();

  void update(
    const KDL::Vector& tar_acc,
    const double& yaw,
    double& U_out,
    double& roll_out,
    double& pitch_out);

private:
  double mass_;
};
}  // namespace tobas_multirotor_controller
