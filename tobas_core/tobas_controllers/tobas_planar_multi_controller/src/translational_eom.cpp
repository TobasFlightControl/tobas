#include "tobas_planar_multi_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;

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
  auto x = xyz.x();
  auto y = xyz.y();
  auto z = xyz.z();

  // 鉛直下方向に推力は出せないことを考慮して垂直成分をクランプ
  z = max(z, 0.);

  // 姿勢の制限を考慮して水平成分をクランプ
  const auto tan_max_atti = tan(cfg_.max_attitude);
  const auto max_xy_norm = z * tan_max_atti * sqrt(2 + tan_max_atti);  // sqrt(x^2 + y^2)の最大値
  algo::clamp2d(x, y, max_xy_norm);

  // 現在のオイラー角を計算
  cur_rot.getRPY(roll_, pitch_, yaw_);

  // 姿勢角が90度を超える場合は実現できない
  if (std::abs(roll_) > M_PI_2 || std::abs(pitch_) > M_PI_2) {
    cerr << "Cannot solve translational EoM because the aircraft is upside-down." << endl;
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

bool TranslationalEoM::setMaxAttitude(double p)
{
  if (p <= 0.) {
    cerr << "Maximum attitude must be positive." << endl;
    return false;
  }

  cfg_.max_attitude = p;
  return true;
}
}  // namespace planar_multicopter
}  // namespace tobas
