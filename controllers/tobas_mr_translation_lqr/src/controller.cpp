#include <dh_std_tools/algorithm.hpp>

#include "../include/tobas_mr_translation_lqr/controller.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_translation_lqr
{
TranslationController::TranslationController() : lqid_(kStateSize, 3, 3)
{
  lqid_.dynamics.A.block<3, 3>(kPosIdx, kVelIdx).diagonal().setOnes();
  lqid_.dynamics.A.block<3, 3>(kVelIdx, kAccIdx).diagonal().setOnes();

  lqid_.C.block<3, 3>(0, kPosIdx).diagonal().setOnes();  // 位置に対してI制御を行う
}

void TranslationController::update(
  const Vector& cp,
  const Vector& cv,
  const Vector& ca,
  const Vector& tp,
  Vector tv,
  const double& dt,
  Vector& ta)
{
  // 目標速度をクランプ
  // TODO: MPCで最大速度をハード制約にする
  dh_std::clamp2d(tv[0], tv[1], max_hor_vel_);
  tv.z(clamp(tv.z(), -max_ver_vel_, max_ver_vel_));

  // 現在の状態と設定値を埋める
  lqid_.current_state << cp.x(), cp.y(), cp.z(), cv.x(), cv.y(), cv.z(), ca.x(), ca.y(), ca.z();
  lqid_.target_state << tp.x(), tp.y(), tp.z(), tv.x(), tv.y(), tv.z(), 0, 0, 0;

  // 最適指令加速度を計算
  ta.data = lqid_.solve(dt, false);  // LTIシステムなのでゲインの再計算は行わない
}

void TranslationController::configure(const Config& config)
{
  assert(config.acc_delay_time_const > 0);
  assert(config.hor_pos_weight >= 0);
  assert(config.ver_pos_weight >= 0);
  assert(config.hor_vel_weight >= 0);
  assert(config.ver_vel_weight >= 0);
  assert(config.hor_acc_weight >= 0);
  assert(config.ver_acc_weight >= 0);
  assert(config.hor_posint_weight >= 0);
  assert(config.ver_posint_weight >= 0);
  assert(config.max_hor_posint_error >= 0);
  assert(config.max_ver_posint_error >= 0);
  assert(config.max_hor_vel >= 0);
  assert(config.max_ver_vel >= 0);

  // 水平方向の加速度は姿勢制御の追従遅れを経て実現される
  lqid_.dynamics.A.block<2, 2>(kAccIdx, kAccIdx).diagonal().fill(-1 / config.acc_delay_time_const);
  lqid_.dynamics.B.block<2, 2>(kAccIdx, 0).diagonal().fill(1 / config.acc_delay_time_const);

  // 垂直方向の加速度は非常に短時間で実現されるとする
  lqid_.dynamics.A(kAccIdx + 2, kAccIdx + 2) = -1 / kVerAccDecayTimeConst;
  lqid_.dynamics.B(kAccIdx + 2, 2) = 1 / kVerAccDecayTimeConst;

  // 重みを更新
  lqid_.state_weight.block<2, 1>(kPosIdx, 0).fill(config.hor_pos_weight);
  lqid_.state_weight(kPosIdx + 2) = config.ver_pos_weight;
  lqid_.state_weight.block<2, 1>(kVelIdx, 0).fill(config.hor_vel_weight);
  lqid_.state_weight(kVelIdx + 2) = config.ver_vel_weight;
  lqid_.state_weight.block<2, 1>(kAccIdx, 0).fill(config.hor_acc_weight);
  lqid_.state_weight(kAccIdx + 2) = config.ver_acc_weight;

  lqid_.integrated_error_weight.block<2, 1>(0, 0).fill(config.hor_posint_weight);
  lqid_.integrated_error_weight(2) = config.ver_posint_weight;

  lqid_.input_weight.setZero();  // 加速度の目標値は実際の加速度ではないため重みはかけない
  lqid_.input_rate_weight.fill(exp10(config.jerk_weight_log10));  // 加速度の観測ノイズの補償

  lqid_.max_integrated_error.block<2, 1>(0, 0).fill(config.max_hor_posint_error);
  lqid_.max_integrated_error(2) = config.max_ver_posint_error;

  lqid_.updateGain();

  max_hor_vel_ = config.max_hor_vel;
  max_ver_vel_ = config.max_ver_vel;
}

Vector3d TranslationController::positionIntegralError() const
{
  return lqid_.integralError();
}
}  // namespace tobas_mr_translation_lqr
