#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_common/accel_attitude_converter.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_common
{
AccelAttitudeConverter::AccelAttitudeConverter(const tobas::Drone& drone)
  : drone_(drone), dynamics_(drone), grav_W_(0, 0, tobas::kGravity), zero_(KDL::Vector::Zero())
{
  updateInternalDataStructures();
}

void AccelAttitudeConverter::updateInternalDataStructures()
{
  dynamics_.updateInternalDataStructures();
}

void AccelAttitudeConverter::update(
  const Euler& cur_rpy,
  const Vector& cur_vel_B,
  const Vector& cur_wind_W,
  const vector<double>& cur_rotor_speeds,
  Vector tar_acc_W,
  double& U_out,
  double& roll_out,
  double& pitch_out)
{
  // 現在の空気効力
  // TODO: 本来は空気効力に含まれる姿勢も未知数として扱う必要がある
  const auto air_drag_W =
    cur_rpy * dynamics_.horizontalForce(cur_rpy, cur_vel_B, cur_wind_W, cur_rotor_speeds);

  // 並進EoMの左辺
  const auto& mass = dynamics_.mass();
  const auto xyz = mass * (tar_acc_W + grav_W_);  // FIXME: 風の補償項を入れるとオーバーシュート過大
  // const auto xyz = mass * (tar_acc_W + grav_W_) - cfg_.h_force_comp_rate * air_drag_W;
  auto x = xyz.x();
  auto y = xyz.y();
  const auto& z = xyz.z();

  // 姿勢の制限を考慮してx, yをクランプ
  const auto tan_max_atti = tan(cfg_.max_attitude);
  const auto max_xy_norm = z * tan_max_atti * sqrt(2 + tan_max_atti);  // sqrt(x^2 + y^2)の最大値
  dh_std::clamp2d(x, y, max_xy_norm);

  // 3元非線形方程式の解析解を計算
  // TODO: 姿勢制御と同様にH-forceを考慮する
  const auto cos_yaw = cos(cur_rpy.yaw);
  const auto sin_yaw = sin(cur_rpy.yaw);
  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = z / (cos(pitch_out) * cos(roll_out));
}

void AccelAttitudeConverter::update(
  const Euler& cur_rpy,
  Vector tar_acc_W,
  double& U_out,
  double& roll_out,
  double& pitch_out)
{
  update(
    cur_rpy, zero_, zero_, vector<double>(drone_.numRotors(), 0), tar_acc_W, U_out, roll_out,
    pitch_out);
}

void AccelAttitudeConverter::configure(const AccelAttitudeConverterConfig& cfg)
{
  assert(0 <= cfg.max_attitude && cfg.max_attitude < M_PI_2);
  assert(0 <= cfg.h_force_comp_rate && cfg.h_force_comp_rate <= 1);

  cfg_ = cfg;
}
}  // namespace tobas_mr_common
