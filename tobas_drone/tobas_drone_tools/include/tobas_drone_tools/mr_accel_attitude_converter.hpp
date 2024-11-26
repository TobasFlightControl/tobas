#pragma once

#include "./mr_dynamics.hpp"

namespace tobas
{
class AccelAttitudeConverter
{
public:
  explicit AccelAttitudeConverter(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures();

  void update(
    const kdl::Rotation& cur_rot,
    const kdl::Vector& tar_acc_W,
    double& thrust_out,
    double& roll_out,
    double& pitch_out);

  bool setMaxAttitude(double p);

private:
  // Config
  double max_attitude_ = M_PI_4;  // [rad]

  const Drone& drone_;
  const kdl::Tree& tree_;

  tobas::MultirotorDynamicsComponents dynamics_;

  const kdl::Vector grav_W_;
  const kdl::Vector zero_;

  double roll_, pitch_, yaw_;
};
}  // namespace tobas
