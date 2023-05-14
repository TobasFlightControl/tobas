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
  explicit AccelerationController(const tobas::Drone& drone, double gravity);

  void update(
    const KDL::Vector& tar_acc,
    const double& yaw,
    double& U_out,
    double& roll_out,
    double& pitch_out);

  const double& maxU() const;

private:
  const tobas::Drone& drone_;

  tobas::RotorAxisExtractor z_rotors_;

  const double gravity_;
  double mass_;
  double max_U_;
};
}  // namespace tobas_multirotor_controller
