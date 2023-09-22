#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>

#include <tobas_tools/utils.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_rotation_mpc/acceleration_controller.hpp"
#include "../include/tobas_mr_rotation_mpc/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_rotation_mpc
{
AccelerationController::AccelerationController()
{
  mass_ = tobas::getMass();
}

void AccelerationController::update(
  const Vector& tar_acc,
  const double& yaw,
  double& U_out,
  double& roll_out,
  double& pitch_out)
{
  // 目標加速度を制限
  auto tar_ax = tar_acc.x();
  auto tar_ay = tar_acc.y();
  dh_std::clamp2d(tar_ax, tar_ay, config_.max_hor_acc);
  const auto tar_az = clamp(tar_acc.z(), -config_.max_ver_acc, config_.max_ver_acc);

  // 並進のEoMの左辺
  auto x = mass_ * tar_ax;
  auto y = mass_ * tar_ay;
  const auto z = mass_ * (tar_az + tobas::kGravity);

  // 姿勢の限界を考慮してx, yを制限
  // さもないと姿勢制御器での目標姿勢角のクランプにより推力が過剰になる恐れがある
  const auto tan_max_atti = tan(config_.max_attitude);
  const auto max_xy_norm = z * tan_max_atti * sqrt(2 + tan_max_atti);  // sqrt(x^2 + y^2)の最大値
  dh_std::clamp2d(x, y, max_xy_norm);

  // 3元非線形方程式の解析解を計算
  // TODO: 姿勢制御と同様にH-forceを考慮する
  const auto cos_yaw = cos(yaw);
  const auto sin_yaw = sin(yaw);
  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = z / (cos(pitch_out) * cos(roll_out));
}

void AccelerationController::configure(const AccelerationControllerConfig& config)
{
  assert(config.max_hor_acc > 0.);
  assert(config.max_ver_acc > 0.);
  assert(0. <= config.max_attitude && config.max_attitude < M_PI_2);

  config_ = config;
}
}  // namespace tobas_mr_rotation_mpc
