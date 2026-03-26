#include "tobas_y_axis_tilt_multi_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_math/float.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;

namespace tobas
{
namespace y_axis_tilt_multicopter
{
TranslationalEoM::TranslationalEoM(const kdl::Tree& tree) : mass_holder_(tree), grav_W_(0, 0, -tbs::kGravity)
{
}

bool TranslationalEoM::updateInternalDataStructures()
{
  return mass_holder_.updateInternalDataStructures();
}

bool TranslationalEoM::solve(
  const kdl::Vector& tar_acc_W,
  const double& tar_pitch,
  const double& tar_yaw,
  const kdl::Vector& ext_force_W,
  double& ux_out,
  double& uz_out,
  kdl::Rotation& rot_out)
{
  // 世界座標系から見た目標力を計算
  const auto& mass = mass_holder_.getMass();
  auto f_W = mass * (tar_acc_W - grav_W_) - ext_force_W;

  // 着陸時など加速度の絶対値が小さいとチルト角の解の変化率が相対的に大きくなる．
  // ミキサーはチルト角の追従の遅延を無視しているため，チルト角の変位が大きくなるのは避けたい．
  // そのため，最低限鉛直上方向に推力を出すことを保証しておく．
  f_W.z(max(f_W.z(), mass * kMinVerticalForcePerMass));

  // 目標力をローカル座標系 (ロールする前) に変換
  const auto f_L = kdl::Rotation::RPY(0., tar_pitch, tar_yaw).inverse(f_W);
  const auto& fx = f_L.x();
  const auto& fy = f_L.y();
  const auto& fz = f_L.z();

  // 3つ目の回転軸回りの回転角を計算
  const auto cos_pitch = cos(tar_pitch);
  const auto sin_pitch = sin(tar_pitch);
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
  const auto u = rot_x.inverse(f_L);
  assert(math::isClose(u.y(), 0., 1e-3));
  ux_out = u.x();
  uz_out = u.z();

  // 非線形方程式を解いた後の目標姿勢行列を計算
  rot_out = kdl::Rotation::RPY(0., tar_pitch, tar_yaw) * rot_x;

  return true;
}
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
