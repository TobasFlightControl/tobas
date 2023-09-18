#include <eigen_conversions/eigen_kdl.h>

#include "../include/tobas_mr_translation_lqr/controller.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_translation_lqr
{
Controller::Controller()
{
  lqd_.resize(kStateSize, kInputSize);

  // ダイナミクスの固定部分を埋める
  lqd_.dynamics.A.block<3, 3>(kPosIdx, kVelIdx).diagonal().fill(1);
  lqd_.dynamics.A.block<3, 3>(kVelIdx, kAccIdx).diagonal().fill(1);

  // 変数のスケールは全て同じとする
  lqd_.state_scale.fill(1);
  lqd_.input_scale.fill(1);
}

void Controller::update(
  const Vector& cp,
  const Vector& cv,
  const Vector& ca,
  const Vector& tp,
  const double& dt,
  Vector& ta)
{
  lqd_.current_state << cp.x(), cp.y(), cp.z(), cv.x(), cv.y(), cv.z(), ca.x(), ca.y(), ca.z();
  lqd_.target_state << tp.x(), tp.y(), tp.z(), 0, 0, 0, 0, 0, 0;  // 速度と加速度の目標値は0

  const auto ta_eigen = lqd_.solveLQD(dt, false);  // LTIシステムなのでゲインの再計算は行わない
  tf::vectorEigenToKDL(ta_eigen, ta);
}

void Controller::configure(const Config& config)
{
  assert(config.acc_delay_time_const > 0);
  assert(config.hor_pos_weight > 0);
  assert(config.ver_pos_weight > 0);
  assert(config.hor_vel_weight > 0);
  assert(config.ver_vel_weight > 0);
  assert(config.acc_weight > 0);
  assert(config.jerk_weight > 0);

  // 水平方向の加速度は姿勢制御の追従遅れを経て実現される
  lqd_.dynamics.A.block<2, 2>(kAccIdx, kAccIdx).diagonal().fill(-1 / config.acc_delay_time_const);
  lqd_.dynamics.B.block<2, 2>(kAccIdx, 0).diagonal().fill(1 / config.acc_delay_time_const);

  // 垂直方向の加速度は非常に短時間で実現されるとする
  lqd_.dynamics.A(kAccIdx + 2, kAccIdx + 2) = -1 / kVerAccDecayTimeConst;
  lqd_.dynamics.B(kAccIdx + 2, 2) = 1 / kVerAccDecayTimeConst;

  lqd_.state_weight.block<2, 1>(kPosIdx, 0).fill(config.hor_pos_weight);
  lqd_.state_weight(kPosIdx + 2, 0) = config.ver_pos_weight;
  lqd_.state_weight.block<2, 1>(kVelIdx, 0).fill(config.hor_vel_weight);
  lqd_.state_weight(kVelIdx + 2, 0) = config.ver_vel_weight;
  lqd_.state_weight.block<3, 1>(kAccIdx, 0).fill(config.acc_weight);
  lqd_.input_weight.fill(0);  // 加速度の目標値は実際の加速度ではないため重みはかけない
  lqd_.input_rate_weight.fill(config.jerk_weight);  // 加速度の観測ノイズの補償

  lqd_.updateGain();
}
}  // namespace tobas_mr_translation_lqr
