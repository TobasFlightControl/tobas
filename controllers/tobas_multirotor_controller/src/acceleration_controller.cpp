#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>

#include <tobas_tools/utils.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_multirotor_controller/acceleration_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_multirotor_controller
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
  dh_std::clamp2d(tar_ax, tar_ay, max_hor_acc_);
  const auto tar_az = clamp(tar_acc.z(), -max_ver_acc_, max_ver_acc_);

  // 3元非線形方程式の解析解を計算
  const auto x = mass_ * tar_ax;
  const auto y = mass_ * tar_ay;
  const auto z = mass_ * (tar_az + tobas::kGravity);

  const auto cos_yaw = cos(yaw);
  const auto sin_yaw = sin(yaw);

  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = z / (cos(pitch_out) * cos(roll_out));
}

void AccelerationController::reconfigure(const AccelerationControllerDynamicParams& params)
{
  assert(params.max_hor_acc > 0.);
  assert(params.max_ver_acc > 0.);

  max_hor_acc_ = params.max_hor_acc;
  max_ver_acc_ = params.max_ver_acc;
}
}  // namespace tobas_multirotor_controller
