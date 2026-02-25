#include "tobas_planar_multi_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace planar_multicopter
{
TranslationalEoM::TranslationalEoM(const kdl::Tree& tree) : mass_holder_(tree), grav_W_(0, 0, -tbs::kGravity)
{
}

bool TranslationalEoM::updateInternalDataStructures()
{
  return mass_holder_.updateInternalDataStructures();
}

bool TranslationalEoM::solve(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& ext_force_W,
  double& thrust_out,
  double& roll_out,
  double& pitch_out)
{
  // 並進EoMの左辺
  const auto xyz = mass_holder_.getMass() * (tar_acc_W - grav_W_) - ext_force_W;
  const auto x = xyz.x();
  const auto y = xyz.y();
  const auto z = std::max(xyz.z(), 0.);  // 鉛直下方向に推力は出せない

  // 現在のオイラー角を計算
  cur_rot.getRPY(roll_, pitch_, yaw_);

  // 姿勢角が90度を超える場合は実現できない
  if (std::abs(roll_) > M_PI_2 || std::abs(pitch_) > M_PI_2) {
    std::cerr << "Cannot solve translational EoM because the aircraft is upside-down." << std::endl;
    return false;
  }

  // 姿勢追従と方位追従を分離するために現在の方位角で目標姿勢角を計算
  const auto cos_yaw = cos(yaw_);
  const auto sin_yaw = sin(yaw_);
  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);

  // 高度追従と姿勢追従を分離するために現在の姿勢で目標推力を計算
  thrust_out = z / (cos(roll_) * cos(pitch_));

  return true;
}
}  // namespace planar_multicopter
}  // namespace tobas
