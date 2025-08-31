#include "tobas_y_axis_tilt_multi_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_constants/constants.hpp>
#include <tobas_std_tools/float.hpp>
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

  // 3つ目の回転軸回りの回転角を計算
  const auto den = fx * sin_pitch - fz * cos_pitch;
  if (den == 0.) {
    cerr << "Free fall is commanded." << endl;
    return false;
  }
  const auto sin_phi = clamp(fy / den, -1., 1.);
  const auto phi = asin(sin_phi);

  // 3つ目の回転行列を計算
  const kdl::Vector n(cos_pitch, 0., sin_pitch);
  const auto rot_x = kdl::Rotation::Rot(n, phi);

  // 機体座標系から見た推力和を計算
  const auto u = rot_x.inverse(f);
  assert(tobas_std::isClose(u.y(), 0., 1e-3));
  ux_out = u.x();
  uz_out = u.z();

  // 非線形方程式を解いた後の目標姿勢行列を計算
  rot_out = kdl::Rotation::RPY(0., tar_pitch, tar_yaw) * rot_x;

  return true;
}
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
