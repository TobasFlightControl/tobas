#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/algorithm.hpp>

#include "../include/tobas_mr_translation_lqr/controller.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_translation_lqr
{
VelocityController::VelocityController()
{
  lqd_.resize(kStateSize, kInputSize);

  lqd_.dynamics.A.block<3, 3>(kPosIdx, kVelIdx).diagonal().setOnes();
  lqd_.dynamics.A.block<3, 3>(kVelIdx, kAccIdx).diagonal().setOnes();

  lqd_.state_scale.setOnes();
  lqd_.input_scale.setOnes();
}

void VelocityController::update(
  const Vector& cp,
  const Vector& cv,
  const Vector& ca,
  Vector tp,
  Vector tv,
  const double& dt,
  Vector& ta)
{
  // 目標位置をクランプ
  auto ep = tp - cp;
  dh_std::clamp2d(ep[0], ep[1], max_hor_pos_error_);
  ep.z(clamp(ep.z(), -max_ver_pos_error_, max_ver_pos_error_));
  tp = cp + ep;

  // 目標速度をクランプ
  dh_std::clamp2d(tv[0], tv[1], max_hor_vel_);
  tv.z(clamp(tv.z(), -max_ver_vel_, max_ver_vel_));

  // 現在の状態と設定値を埋める
  lqd_.current_state << cp.x(), cp.y(), cp.z(), cv.x(), cv.y(), cv.z(), ca.x(), ca.y(), ca.z();
  lqd_.target_state << tp.x(), tp.y(), tp.z(), tv.x(), tv.y(), tv.z(), 0, 0, 0;

  // 最適指令加速度を計算
  const auto ta_eigen = lqd_.solve(dt, false);  // LTIシステムなのでゲインの再計算は行わない
  tf::vectorEigenToKDL(ta_eigen, ta);
}

void VelocityController::configure(const Config& config)
{
  assert(config.acc_delay_time_const > 0);
  assert(config.hor_pos_weight >= 0);
  assert(config.ver_pos_weight >= 0);
  assert(config.hor_vel_weight >= 0);
  assert(config.ver_vel_weight >= 0);
  assert(config.hor_acc_weight >= 0);
  assert(config.ver_acc_weight >= 0);
  assert(config.max_hor_pos_error >= 0);
  assert(config.max_ver_pos_error >= 0);
  assert(config.max_hor_vel >= 0);
  assert(config.max_ver_vel >= 0);

  // 水平方向の加速度は姿勢制御の追従遅れを経て実現される
  lqd_.dynamics.A.block(kAccIdx, kAccIdx, 2, 2).diagonal().fill(-1 / config.acc_delay_time_const);
  lqd_.dynamics.B.block(kAccIdx, 0, 2, 2).diagonal().fill(1 / config.acc_delay_time_const);

  // 垂直方向の加速度は非常に短時間で実現されるとする
  lqd_.dynamics.A(kAccIdx + 2, kAccIdx + 2) = -1 / kVerAccDecayTimeConst;
  lqd_.dynamics.B(kAccIdx + 2, 2) = 1 / kVerAccDecayTimeConst;

  // 重みを更新
  lqd_.state_weight.block(kPosIdx, 0, 2, 1).fill(config.hor_pos_weight);
  lqd_.state_weight(kPosIdx + 2) = config.ver_pos_weight;
  lqd_.state_weight.block(kVelIdx, 0, 2, 1).fill(config.hor_vel_weight);
  lqd_.state_weight(kVelIdx + 2) = config.ver_vel_weight;
  lqd_.state_weight.block(kAccIdx, 0, 2, 1).fill(config.hor_acc_weight);
  lqd_.state_weight(kAccIdx + 2) = config.ver_acc_weight;

  lqd_.input_weight.setZero();  // 加速度の目標値は実際の加速度ではないため重みはかけない
  lqd_.input_rate_weight.fill(exp10(config.jerk_weight_log10));  // 加速度の観測ノイズの補償

  lqd_.updateGain();

  max_hor_pos_error_ = config.max_hor_pos_error;
  max_ver_pos_error_ = config.max_ver_pos_error;
  max_hor_vel_ = config.max_hor_vel;
  max_ver_vel_ = config.max_ver_vel;
}
}  // namespace tobas_mr_translation_lqr
