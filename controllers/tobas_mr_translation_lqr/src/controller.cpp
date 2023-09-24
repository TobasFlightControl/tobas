#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/algorithm.hpp>

#include "../include/tobas_mr_translation_lqr/controller.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_translation_lqr
{
VelocityController::VelocityController() : lqid_(kStateSize, kInputSize, kIntegrateSize)
{
  lqid_.dynamics.A.block<3, 3>(kVelIdx, kAccIdx).diagonal().setOnes();
  lqid_.C.block<kVelSize, kVelSize>(0, kVelIdx).diagonal().setOnes();
}

void VelocityController::update(
  const Vector& cv,
  const Vector& ca,
  const Vector& tv,
  const double& dt,
  Vector& ta)
{
  auto tvx = tv.x();
  auto tvy = tv.y();
  dh_std::clamp2d(tvx, tvy, max_hor_vel_);
  const auto tvz = clamp(tv.z(), -max_ver_vel_, max_ver_vel_);

  lqid_.current_state << cv.x(), cv.y(), cv.z(), ca.x(), ca.y(), ca.z();
  lqid_.target_state << tvx, tvy, tvz, 0, 0, 0;

  const auto ta_eigen = lqid_.solve(dt, false);  // LTIシステムなのでゲインの再計算は行わない
  tf::vectorEigenToKDL(ta_eigen, ta);

  // cout << lqid_ << endl;
}

void VelocityController::configure(const Config& config)
{
  assert(config.acc_delay_time_const > 0);
  assert(config.hor_pos_weight > 0);
  assert(config.ver_pos_weight > 0);
  assert(config.hor_vel_weight > 0);
  assert(config.ver_vel_weight > 0);
  assert(config.hor_acc_weight > 0);
  assert(config.ver_acc_weight > 0);
  assert(config.jerk_weight > 0);
  assert(config.max_hor_pos_error > 0);
  assert(config.max_ver_pos_error > 0);
  assert(config.max_hor_vel > 0);
  assert(config.max_ver_vel > 0);

  // 水平方向の加速度は姿勢制御の追従遅れを経て実現される
  lqid_.dynamics.A.block(kAccIdx, kAccIdx, 2, 2).diagonal().fill(-1 / config.acc_delay_time_const);
  lqid_.dynamics.B.block(kAccIdx, 0, 2, 2).diagonal().fill(1 / config.acc_delay_time_const);

  // 垂直方向の加速度は非常に短時間で実現されるとする
  lqid_.dynamics.A(kAccIdx + 2, kAccIdx + 2) = -1 / kVerAccDecayTimeConst;
  lqid_.dynamics.B(kAccIdx + 2, 2) = 1 / kVerAccDecayTimeConst;

  // 重みを更新
  lqid_.state_weight.block(kVelIdx, 0, 2, 1).fill(config.hor_vel_weight);
  lqid_.state_weight(kVelIdx + 2) = config.ver_vel_weight;
  lqid_.state_weight.block(kAccIdx, 0, 2, 1).fill(config.hor_acc_weight);
  lqid_.state_weight(kAccIdx + 2) = config.ver_acc_weight;

  lqid_.integrated_error_weight.block(0, 0, 2, 1).fill(config.hor_pos_weight);
  lqid_.integrated_error_weight(2) = config.ver_pos_weight;

  lqid_.input_weight.setZero();  // 加速度の目標値は実際の加速度ではないため重みはかけない
  lqid_.input_rate_weight.fill(config.jerk_weight);  // 加速度の観測ノイズの補償

  // 位置誤差の最大値
  lqid_.max_integrated_error.block(0, 0, 2, 1).fill(config.max_hor_pos_error);
  lqid_.max_integrated_error(2) = config.max_ver_pos_error;

  lqid_.updateGain();

  max_hor_vel_ = config.max_hor_vel;
  max_ver_vel_ = config.max_ver_vel;
}
}  // namespace tobas_mr_translation_lqr
