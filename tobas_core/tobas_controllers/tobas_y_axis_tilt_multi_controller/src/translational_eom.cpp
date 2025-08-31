#include "tobas_y_axis_tilt_multi_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_constants/constants.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;

namespace tobas
{
namespace y_axis_tilt_multicopter
{
TranslationalEoM::TranslationalEoM(const kdl::Tree& tree) : mass_holder_(tree), grav_W_(0, 0, -tobas_std::kGravity)
{
}

bool TranslationalEoM::updateInternalDataStructures()
{
  return mass_holder_.updateInternalDataStructures();
}

bool TranslationalEoM::update(
  const kdl::Vector& tar_acc_W,
  const double& tar_pitch,
  const double& tar_yaw,
  const kdl::Vector& ext_force_W,
  double& ux_out,
  double& uz_out,
  kdl::Rotation& rot_out)
{
  const auto& mass = mass_holder_.getMass();
  const auto f = kdl::Rotation::RPY(0., tar_pitch, tar_yaw).inverse(mass * (tar_acc_W - grav_W_) - ext_force_W);
  const auto& fx = f.x();
  const auto& fy = f.y();
  const auto& fz = f.z();

  const auto cos_pitch = cos(tar_pitch);
  const auto sin_pitch = sin(tar_pitch);

  const auto den = fx * sin_pitch - fz * cos_pitch;
  if (den == 0.) {
    cerr << "Free fall is commanded." << endl;
    return false;
  }
  const auto sin_phi = clamp(fy / den, -1., 1.);
  const auto phi = asin(sin_phi);

  const kdl::Vector n(cos_pitch, 0., sin_pitch);
  const auto rot_x = kdl::Rotation::Rot(n, phi);
  rot_out = kdl::Rotation::RPY(0., tar_pitch, tar_yaw) * rot_x;

  const auto u = rot_x.inverse(f);
  ux_out = u.x();
  uz_out = u.z();

  return true;
}
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
