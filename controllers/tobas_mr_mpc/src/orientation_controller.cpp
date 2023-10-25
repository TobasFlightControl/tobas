#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/geometry.hpp>
#include <dh_linear_control/util.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../include/tobas_mr_mpc/orientation_controller.hpp"
#include "../include/tobas_mr_mpc/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_mpc
{
OrientationController::OrientationController(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE),
    mixer_(drone),
    cont_(drone),
    c2d_(kStateSize, z_rotors_.count()),
    stopwatch_(tobas::kStopwatchSamples)
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
  mpc_.control_scale = mpc_.state_scale.head(kCtrlSize);

  mpc_.control_weight.resize(kCtrlSize);
  mpc_.current_state.resize(kStateSize);
  mpc_.set_state.resize(kCtrlSize);

  updateInternalDataStructures();
}

void OrientationController::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  c2d_.resize(kStateSize, z_rotors_.count());

  // 制御入力のスケール
  mpc_.input_scale.resize(z_rotors_.count());
  mpc_.input_scale.fill(tobas::getMass() * tobas::kGravity / z_rotors_.count());

  // 推力の和が一定だから，推力の二乗和の重みが大きいほど各プロペラの推力を均等にしようとする力が働く．
  // 回転翼機の制御においてそれは致命傷になりうるため，推力の重みは0で固定する．
  mpc_.input_weight = VectorXd::Zero(z_rotors_.count());
  mpc_.input_rate_weight.resize(z_rotors_.count());

  fillInputConstraintFixedParts();
}

VectorXd OrientationController::solve(
  const Euler& cur_rpy,
  const Twist& cur_twist_B,
  const Vector& cur_wind_W,
  const JntArray& q,
  const double& battery_voltage,
  const double& tar_U,
  Euler tar_rpy)
{
  assert(battery_voltage > 0);

  // 目標姿勢を制限
  tar_rpy.roll = clamp(tar_rpy.roll, -max_attitude_, max_attitude_);
  tar_rpy.pitch = clamp(tar_rpy.pitch, -max_attitude_, max_attitude_);
  const double yaw_error =
    clamp(tar_rpy.yaw - cur_rpy.yaw, -max_heading_error_, max_heading_error_);
  tar_rpy.yaw = cur_rpy.yaw + yaw_error;

  // 目標とする姿勢と合計推力から，重力方向の推力を計算
  const double thrust_z = tar_U * cos(tar_rpy.roll) * cos(tar_rpy.pitch);

  // MPCの最適制御問題を構築
  updateCurrentState(cur_rpy, cur_twist_B, cur_wind_W, q, thrust_z);
  updateSetState(tar_rpy);

  const double min_voltage = battery_voltage * tobas::kMotorSpinArm;
  const double max_thrust_sum = maxThrustSum(battery_voltage);
  const double min_thrust_sum = minThrustSum(battery_voltage);

  // ダイナミクスと制御入力の制約を更新
  // 姿勢や推力の目標値をそのまま使うと追従性能が悪い場合に想定外の動きになるため，
  // 参照起動からダイナミクスや制約を構成する．
  // 初期状態のダイナミクスを保持するため，後ろから順に処理する
  for (int k = mpc_.prediction_steps - 1; k >= 0; --k)
  {
    const double t = mpc_.time_step * k;  // 計画開始時刻 (= 0) からの経過時間

    // ダイナミクスを更新
    const double roll_k =
      ctrl::firstOrderPos(cur_rpy.roll, tar_rpy.roll, mpc_.decay_time_consts(kRollIdx), t);
    const double pitch_k =
      ctrl::firstOrderPos(cur_rpy.pitch, tar_rpy.pitch, mpc_.decay_time_consts(kPitchIdx), t);
    cont_.update(roll_k, pitch_k, q);
    mpc_.discrete_dynamics[k] = c2d_.convert(cont_, mpc_.time_step);

    // 個々のプロペラの推力の限界に関する不等式制約
    for (uint32_t i = 0; i < z_rotors_.count(); ++i)
    {
      mpc_.input_ineqs[k].b(i) = z_rotors_.thrustFromVoltage(i, battery_voltage);
      mpc_.input_ineqs[k].b(z_rotors_.count() + i) = -z_rotors_.thrustFromVoltage(i, min_voltage);
    }

    // 全てのプロペラの推力の合計に関する等式制約
    const double thrust_k =
      clamp(thrust_z / (cos(roll_k) * cos(pitch_k)), min_thrust_sum, max_thrust_sum);
    mpc_.input_eqs[k].b(0) = thrust_k;
  }

  // MPCを解く
  // stopwatch_.start();
  mpc_.solve();
  // stopwatch_.stop();

  // MPCの解
  const VectorXd& thrusts_des = mpc_.optimalControlInput();
  const VectorXd xd = cont_.dynamics(mpc_.current_state, thrusts_des);
  const Vector3d dgyro_mpc = xd.block<3, 1>(kGyroIdx, 0);

  // 外乱補償用の微分先行型PD
  const Vector error_B = (cur_rpy.toRotation().Inverse() * tar_rpy.toRotation()).GetRot();
  const Vector dgyro_pd = kp_ * error_B - kd_ * cur_twist_B.rot;

  // Mixerで最終的な推力を計算
  const Vector3d dgyro_des = dgyro_mpc + dgyro_pd.data;  // FF + FB (二自由度制御)
  return mixer_.solve(battery_voltage, q, cur_twist_B.rot.data, dgyro_des, thrusts_des);
}

void OrientationController::configure(const OrientationControllerConfig& config)
{
  assert(0 <= config.max_attitude && config.max_attitude < M_PI_2);
  assert(config.max_heading_error >= 0);
  assert(0 <= config.h_force_comp_rate && config.h_force_comp_rate <= 1);
  assert(config.kp >= 0);
  assert(config.kd >= 0);
  assert(config.pred_horizon > 0);
  assert(config.pred_steps > 0);
  assert(config.attitude_decay >= 0);
  assert(config.heading_decay >= 0);
  assert(config.angvel_decay >= 0);
  assert(config.attitude_weight > 0);
  assert(config.heading_weight > 0);
  assert(config.angvel_weight > 0);

  max_attitude_ = config.max_attitude;
  max_heading_error_ = config.max_heading_error;
  h_force_coef_ = config.h_force_comp_rate;
  kp_ = config.kp;
  kd_ = config.kd;

  mpc_.time_step = config.pred_horizon / config.pred_steps;
  mpc_.prediction_steps = mpc_.input_steps = config.pred_steps;

  mpc_.decay_time_consts(kRollIdx) = mpc_.decay_time_consts(kPitchIdx) = config.attitude_decay;
  mpc_.decay_time_consts(kYawIdx) = config.heading_decay;
  mpc_.decay_time_consts.block<3, 1>(kGyroIdx, 0).fill(config.angvel_decay);

  mpc_.discrete_dynamics.resize(config.pred_steps);

  mpc_.control_weight(kRollIdx) = mpc_.control_weight(kPitchIdx) = config.attitude_weight;
  mpc_.control_weight(kYawIdx) = config.heading_weight;
  mpc_.control_weight.block<3, 1>(kGyroIdx, 0).fill(config.angvel_weight);
  mpc_.input_rate_weight.fill(exp10(config.thrust_rate_weight_log10));

  mpc_.input_rate_eqs.resize(config.pred_steps, ctrl::LinearEquation(z_rotors_.count(), 0));
  mpc_.control_eqs.resize(config.pred_steps, ctrl::LinearEquation(kCtrlSize, 0));
  mpc_.input_rate_ineqs.resize(config.pred_steps, ctrl::LinearEquation(z_rotors_.count(), 0));
  mpc_.control_ineqs.resize(config.pred_steps, ctrl::LinearEquation(kCtrlSize, 0));

  mpc_.input_eqs.resize(mpc_.prediction_steps);
  mpc_.input_ineqs.resize(mpc_.prediction_steps);
  fillInputConstraintFixedParts();
}

const VectorXd& OrientationController::mpcThrusts() const
{
  return mpc_.optimalControlInput();
}

double OrientationController::maxThrustSum(const double& battery_voltage) const
{
  double res = 0;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, battery_voltage);
  }
  return res;
}

double OrientationController::minThrustSum(const double& battery_voltage) const
{
  const auto min_voltage = battery_voltage * tobas::kMotorSpinArm;
  double res = 0;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, min_voltage);
  }
  return res;
}

void OrientationController::updateCurrentState(
  const Euler& cur_rpy,
  const Twist& cur_twist_B,
  const Vector& cur_wind_W,
  const JntArray& q,
  const double& thrust_z)
{
  // 機体速度のプロペラに対する水平成分を求める．機体座標系ではZ成分のみ0としたベクトルに等しい．
  // TODO: 正確には機体フレームではなくプロペラの位置の速度を使う
  const auto relative_vel_B = cur_twist_B.vel - cur_rpy.Inverse(cur_wind_W);  // 風に対する相対速度
  const Vector vel_perp(relative_vel_B.x(), relative_vel_B.y(), 0);

  // 重心を求める
  const auto I_base = inertia_solver_.JntToCart(q);
  const auto P_base_cog = I_base.getCOG();

  // 簡単のため全プロペラの推力が等しいとしてH-forceの和を計算
  // TODO: 予測区間での推力の変化を反映し，より真値に近い回転数を用いて計算
  const double thrust = thrust_z / (cos(cur_rpy.roll) * cos(cur_rpy.pitch));  // 合計推力
  const double thrust_mean = thrust / z_rotors_.count();
  Vector sum = Vector::Zero();
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    // CoG -> Rotor の位置を求める
    const auto T_base_rotor = fk_solver_.JntToCart(q, z_rotors_.linkName(i));
    const auto P_cog_rotor = T_base_rotor.p - P_base_cog;

    const double rot_speed = z_rotors_.rotSpeedFromThrust(i, thrust_mean);
    sum += z_rotors_.dragConstant(i) * rot_speed * P_cog_rotor;
  }
  const Vector h_moment_raw = -sum * vel_perp;  // H-forceによるモーメント (予測区間で一定)
  const Vector h_moment_comp = h_moment_raw * h_force_coef_;  // H-forceによるモーメントの補償分

  // 現在の状態を更新
  mpc_.current_state << cur_rpy.roll, cur_rpy.pitch, cur_rpy.yaw, cur_twist_B.rot.x(),
    cur_twist_B.rot.y(), cur_twist_B.rot.z(), h_moment_comp.x(), h_moment_comp.y(),
    h_moment_comp.z();
}

void OrientationController::updateSetState(const Euler& tar_rpy)
{
  mpc_.set_state << tar_rpy.roll, tar_rpy.pitch, tar_rpy.yaw, 0, 0, 0;
}

void OrientationController::fillInputConstraintFixedParts()
{
  for (auto& u_eq : mpc_.input_eqs)
  {
    u_eq.resize(z_rotors_.count(), 1);
    u_eq.A.setOnes();
  }

  for (auto& u_ineq : mpc_.input_ineqs)
  {
    u_ineq.resize(z_rotors_.count(), z_rotors_.count() * 2);
    u_ineq.setZero();

    u_ineq.A.topRows(z_rotors_.count()).diagonal().fill(1);
    u_ineq.A.bottomRows(z_rotors_.count()).diagonal().fill(-1);
  }
}
}  // namespace tobas_mr_mpc
