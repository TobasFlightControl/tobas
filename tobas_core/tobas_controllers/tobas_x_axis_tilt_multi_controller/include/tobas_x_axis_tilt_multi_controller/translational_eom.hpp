#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>

namespace tobas
{
namespace x_axis_tilt_multicopter
{
/* memo: 3-38 */
class TranslationalEoM
{
public:
  explicit TranslationalEoM(const kdl::Tree& tree);

  bool updateInternalDataStructures();

  bool update(
    const kdl::Vector& tar_acc_W,
    const double& tar_pitch,
    const double& tar_yaw,
    const kdl::Vector& ext_force_W,
    double& ux_out,
    double& uz_out,
    kdl::Rotation& rot_out);

private:
  kdl::TreeMassHolder mass_holder_;

  const kdl::Vector grav_W_;
};
}  // namespace x_axis_tilt_multicopter
}  // namespace tobas
