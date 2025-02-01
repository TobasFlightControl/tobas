#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>

namespace tobas
{
class AccelAttitudeConverter
{
public:
  explicit AccelAttitudeConverter(const kdl::Tree& tree);

  bool updateInternalDataStructures();

  void update(
    const kdl::Rotation& cur_rot,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& ext_force_W,
    double& thrust_out,
    double& roll_out,
    double& pitch_out);

  bool setMaxAttitude(double p);

private:
  // Config
  double max_attitude_ = M_PI_4;  // [rad]

  kdl::TreeMassHolder mass_holder_;

  const kdl::Vector grav_W_;
  double roll_, pitch_, yaw_;
};
}  // namespace tobas
