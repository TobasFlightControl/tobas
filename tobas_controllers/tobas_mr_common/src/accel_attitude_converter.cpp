#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/check.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_common/accel_attitude_converter.hpp"

using namespace std;

namespace tobas_mr_common
{
AccelAttitudeConverter::AccelAttitudeConverter(const tobas::Drone& drone)
  : drone_(drone), dynamics_(drone), grav_W_(0, 0, tobas::kGravity), zero_(kdl::Vector::Zero())
{
  updateInternalDataStructures();
}

void AccelAttitudeConverter::updateInternalDataStructures()
{
  dynamics_.updateInternalDataStructures();
}

void AccelAttitudeConverter::update(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_vel_B,
  const kdl::Vector& cur_wind_W,
  const vector<double>& cur_rotor_speeds,
  const kdl::Vector& tar_acc_W,
  double& thrust_out,
  double& roll_out,
  double& pitch_out)
{
  // 現在の空気効力
  // TODO: 本来は空気効力に含まれる姿勢も未知数として扱う必要がある
  const auto air_drag_W = cur_rot * dynamics_.horizontalForce(cur_rot, cur_vel_B, cur_wind_W, cur_rotor_speeds);

  // 並進EoMの左辺
  const auto& mass = dynamics_.mass();
  const auto xyz = mass * (tar_acc_W + grav_W_) - cfg_.h_force_comp_rate * air_drag_W;
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

void AccelAttitudeConverter::update(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& tar_acc_W,
  double& thrust_out,
  double& roll_out,
  double& pitch_out)
{
  update(cur_rot, zero_, zero_, vector<double>(drone_.numRotors(), 0), tar_acc_W, thrust_out, roll_out, pitch_out);
}

void AccelAttitudeConverter::configure(const AccelAttitudeConverterConfig& cfg)
{
  TOBAS_CHECK(0 <= cfg.max_attitude && cfg.max_attitude < M_PI_2);
  TOBAS_CHECK(0 <= cfg.h_force_comp_rate && cfg.h_force_comp_rate <= 1);

  cfg_ = cfg;
}
}  // namespace tobas_mr_common
