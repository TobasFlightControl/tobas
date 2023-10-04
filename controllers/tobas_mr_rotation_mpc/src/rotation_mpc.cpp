#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_linear_control/util.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../include/tobas_mr_rotation_mpc/rotation_mpc.hpp"
#include "../include/tobas_mr_rotation_mpc/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_rotation_mpc
{
RotationMpc::RotationMpc(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE),
    cont_(drone),
    c2d_(kStateSize, z_rotors_.count())
{
  mpc_.Cz = MatrixXd::Zero(kCtrlSize, kStateSize);
  mpc_.Cz.block<kCtrlSize, kCtrlSize>(kRotIdx, kRotIdx).diagonal().setOnes();

  mpc_.decay_time_consts.resize(kCtrlSize);

  // 状態変数のスケール
  mpc_.state_scale.resize(kStateSize);
  mpc_.state_scale.block<3, 1>(kRotIdx, 0).fill(M_PI);
  mpc_.state_scale.block<3, 1>(kGyroIdx, 0).fill(M_PI);
  mpc_.state_scale.block<3, 1>(kHForceIdx, 0).fill(kHMomentScale);

  // 制御変数のスケールは状態変数と等しい
  mpc_.control_scale = mpc_.state_scale.topRows(kCtrlSize);

  mpc_.control_weight.resize(kCtrlSize);
  mpc_.current_state.resize(kStateSize);
  mpc_.set_state.resize(kCtrlSize);

  updateInternalDataStructures();
}

void RotationMpc::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  c2d_.resize(kStateSize, z_rotors_.count());

  // 制御入力のスケール
  mpc_.input_scale.resize(z_rotors_.count());
  mpc_.input_scale.fill(tobas::getMass() * tobas::kGravity / z_rotors_.count());

  // 推力の和が一定だから，推力の二乗和の重みが大きいほど各プロペラの推力を均等にしようとする力が働く．
  // 回転翼機の制御においてそれは致命傷になりうるため，推力の重みは0で固定する．
  mpc_.input_weight = VectorXd::Zero(z_rotors_.count());
  mpc_.input_rate_weight.resize(z_rotors_.count());

  mpc_.last_input = VectorXd::Zero(z_rotors_.count());

  fillInputConstraintFixedParts();
}

VectorXd RotationMpc::update(
  const Euler& cur_rpy,
  const Twist& cur_twist_B,
  const JntArray& q,
  const double& battery_voltage,
  const double& tar_U,
  const Euler& tar_rpy)
{
  assert(battery_voltage > 0.);

  // 目標姿勢を制限
  const double tar_roll = clamp(tar_rpy.roll, -max_attitude_, max_attitude_);
  const double tar_pitch = clamp(tar_rpy.pitch, -max_attitude_, max_attitude_);
  const double yaw_error =
    clamp(tar_rpy.yaw - cur_rpy.yaw, -max_heading_error_, max_heading_error_);
  const double tar_yaw = cur_rpy.yaw + yaw_error;

  // 目標とする姿勢と合計推力から，重力方向の推力を計算
  const double thrust_z = tar_U * cos(tar_roll) * cos(tar_pitch);

  // MPCの最適制御問題を構築
  updateCurrentState(cur_rpy, cur_twist_B, q, thrust_z);
  updateSetState(tar_roll, tar_pitch, tar_yaw);

  const double min_voltage = battery_voltage * tobas::kMotorSpinArm;
  const double max_thrust_sum = maxThrustSum(battery_voltage);
  const double min_thrust_sum = minThrustSum(battery_voltage);

  // ダイナミクスと制御入力の制約を更新
  // 姿勢や推力の目標値をそのまま使うと追従性能が悪い場合に想定外の動きになるため，
  // 参照起動からダイナミクスや制約を構成する．
  for (uint32_t k = 0; k < mpc_.prediction_steps; ++k)
  {
    const double t = mpc_.time_step * k;  // 計画開始時刻 (= 0) からの経過時間

    // ダイナミクスを更新
    const double roll_k =
      ctrl::firstOrderPos(cur_rpy.roll, tar_roll, mpc_.decay_time_consts(kRollIdx), t);
    const double pitch_k =
      ctrl::firstOrderPos(cur_rpy.pitch, tar_pitch, mpc_.decay_time_consts(kPitchIdx), t);
    cont_.update(roll_k, pitch_k, q);
    mpc_.discrete_dynamics[k] = c2d_.convert(cont_, mpc_.time_step);

    // 個々のプロペラの推力の限界に関する不等式制約
    for (uint32_t i = 0; i < z_rotors_.count(); ++i)
    {
      mpc_.input_constraints[k].b(i) = z_rotors_.thrustFromVoltage(i, battery_voltage);
      mpc_.input_constraints[k].b(z_rotors_.count() + i) =
        -z_rotors_.thrustFromVoltage(i, min_voltage);
    }

    // 全てのプロペラの推力の合計に関する等式制約
    const double thrust_k =
      clamp(thrust_z / (cos(roll_k) * cos(pitch_k)), min_thrust_sum, max_thrust_sum);
    mpc_.input_constraints[k].b(z_rotors_.count() * 2) = thrust_k;
    mpc_.input_constraints[k].b(z_rotors_.count() * 2 + 1) = -thrust_k;
  }

  // MPCを解く
  return mpc_.solve();
}

void RotationMpc::configure(const RotationMpcConfig& params)
{
  assert(0. <= params.max_attitude && params.max_attitude < M_PI_2);
  assert(params.max_heading_error >= 0.);
  assert(0. <= params.h_force_comp_rate && params.h_force_comp_rate <= 1.);
  assert(params.pred_horizon > 0.);
  assert(params.pred_steps > 0);
  assert(params.attitude_decay >= 0.);
  assert(params.heading_decay >= 0.);
  assert(params.angvel_decay >= 0.);
  assert(params.attitude_weight > 0.);
  assert(params.heading_weight > 0.);
  assert(params.angvel_weight > 0.);

  max_attitude_ = params.max_attitude;
  max_heading_error_ = params.max_heading_error;
  h_force_coef_ = params.h_force_comp_rate;

  mpc_.time_step = params.pred_horizon / params.pred_steps;
  mpc_.prediction_steps = mpc_.input_steps = params.pred_steps;

  mpc_.decay_time_consts(kRollIdx) = mpc_.decay_time_consts(kPitchIdx) = params.attitude_decay;
  mpc_.decay_time_consts(kYawIdx) = params.heading_decay;
  mpc_.decay_time_consts.block<3, 1>(kGyroIdx, 0).fill(params.angvel_decay);

  mpc_.discrete_dynamics.resize(
    params.pred_steps, ctrl::LinearDynamics(kStateSize, z_rotors_.count()));

  mpc_.control_weight(kRollIdx) = mpc_.control_weight(kPitchIdx) = params.attitude_weight;
  mpc_.control_weight(kYawIdx) = params.heading_weight;
  mpc_.control_weight.block<3, 1>(kGyroIdx, 0).fill(params.angvel_weight);
  mpc_.input_rate_weight.fill(exp10(params.thrust_rate_weight_log10));

  mpc_.input_rate_constraints.resize(params.pred_steps, ctrl::LinearEquation(z_rotors_.count(), 0));
  mpc_.control_constraints.resize(params.pred_steps, ctrl::LinearEquation(kCtrlSize, 0));

  mpc_.input_constraints.resize(mpc_.prediction_steps);
  fillInputConstraintFixedParts();
}

double RotationMpc::maxThrustSum(const double& battery_voltage) const
{
  double res = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, battery_voltage);
  }
  return res;
}

double RotationMpc::minThrustSum(const double& battery_voltage) const
{
  const auto min_voltage = battery_voltage * tobas::kMotorSpinArm;
  double res = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, min_voltage);
  }
  return res;
}

void RotationMpc::updateCurrentState(
  const Euler& cur_rpy,
  const Twist& cur_twist_B,
  const JntArray& q,
  const double& thrust_z)
{
  const auto& vel = cur_twist_B.vel;
  const auto& gyro = cur_twist_B.rot;

  // 機体速度のプロペラに対する水平成分を求める．機体座標系ではZ成分のみ0としたベクトルに等しい．
  // TODO: 正確には機体フレームではなくプロペラの位置の速度を使う
  const Vector vel_perp(vel.x(), vel.y(), 0);

  // 重心を求める
  inertia_solver_.JntToCart(q, P_base_cog_, I_cog_);

  // 簡単のため全プロペラの推力が等しいとしてH-forceの和を計算
  // TODO: より真値に近い回転数を用いて計算
  const double thrust = thrust_z / (cos(cur_rpy.roll) * cos(cur_rpy.pitch));  // 合計推力
  const double thrust_mean = thrust / z_rotors_.count();
  Vector sum = Vector::Zero();
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    // CoG -> Rotor の位置を求める
    fk_solver_.JntToCart(q, z_rotors_.linkName(i), T_base_rotor_);
    const Vector P_cog_rotor = T_base_rotor_.p - P_base_cog_;

    const double rot_speed = z_rotors_.rotSpeedFromThrust(i, thrust_mean);
    sum += z_rotors_.dragConstant(i) * rot_speed * P_cog_rotor;
  }
  const Vector h_moment_raw = -sum * vel_perp;
  const Vector h_moment_comp = h_moment_raw * h_force_coef_;  // H-forceによるモーメントの補償分

  // 現在の状態を更新
  mpc_.current_state << cur_rpy.roll, cur_rpy.pitch, cur_rpy.yaw, gyro.x(), gyro.y(), gyro.z(),
    h_moment_comp.x(), h_moment_comp.y(), h_moment_comp.z();
}

void RotationMpc::updateSetState(
  const double& tar_roll,
  const double& tar_pitch,
  const double& tar_yaw)
{
  mpc_.set_state << tar_roll, tar_pitch, tar_yaw, 0., 0., 0.;
}

void RotationMpc::fillInputConstraintFixedParts()
{
  for (auto& u_const : mpc_.input_constraints)
  {
    u_const.resize(z_rotors_.count(), z_rotors_.count() * 2 + 2);
    u_const.setZero();

    u_const.A.block(0, 0, z_rotors_.count(), z_rotors_.count()).diagonal().fill(1);
    u_const.A.block(z_rotors_.count(), 0, z_rotors_.count(), z_rotors_.count()).diagonal().fill(-1);
    u_const.A.block(z_rotors_.count() * 2, 0, 1, z_rotors_.count()).fill(1);
    u_const.A.block(z_rotors_.count() * 2 + 1, 0, 1, z_rotors_.count()).fill(-1);
  }
}
}  // namespace tobas_mr_rotation_mpc
