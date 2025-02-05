#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_algorithm/core.hpp>

#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/mr_accel_attitude_converter.hpp"

using namespace std;

namespace tobas
{
AccelAttitudeConverter::AccelAttitudeConverter(const kdl::Tree& tree)
  : mass_holder_(tree), grav_W_(0, 0, -tobas_std::kGravity)
{
}

bool AccelAttitudeConverter::updateInternalDataStructures()
{
  return mass_holder_.updateInternalDataStructures();
}

void AccelAttitudeConverter::update(
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
  const auto& z = xyz.z();

  // 姿勢の制限を考慮してx, yをクランプ
  const auto tan_max_atti = tan(cfg_.max_attitude);
  const auto max_xy_norm = z * tan_max_atti * sqrt(2 + tan_max_atti);  // sqrt(x^2 + y^2)の最大値
  algo::clamp2d(x, y, max_xy_norm);

  // 3元非線形方程式の解析解を計算
  cur_rot.getRPY(roll_, pitch_, yaw_);
  const auto cos_yaw = cos(yaw_);
  const auto sin_yaw = sin(yaw_);
  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  // thrust_out = z / (cos(pitch_out) * cos(roll_out));  // 非線形方程式の解析解
  thrust_out = z / (cos(roll_) * cos(pitch_));  // 現在の姿勢でZ軸加速度を満たす解
}

bool AccelAttitudeConverter::setMaxAttitude(double p)
{
  if (p <= 0.)
  {
    cerr << "Maximum attitude must be positive." << endl;
    return false;
  }

  cfg_.max_attitude = p;
  return true;
}
}  // namespace tobas
