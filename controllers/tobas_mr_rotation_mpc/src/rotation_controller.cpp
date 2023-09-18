#include <kdl/frames_io.hpp>

#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_linear_control/util.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../include/tobas_mr_rotation_mpc/rotation_controller.hpp"
#include "../include/tobas_mr_rotation_mpc/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_rotation_mpc
{
RotationController::RotationController(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE),
    cont_(drone),
    c2d_(kStateSize, z_rotors_.count())
{
  mpc_.Cz = MatrixXd::Zero(kCtrlSize, kStateSize);
  mpc_.Cz.block<kCtrlSize, kCtrlSize>(kRotIdx, kRotIdx).diagonal().fill(1);

  mpc_.decay_time_consts.resize(kCtrlSize);
  setScales();
  mpc_.input_rate_weight.resize(z_rotors_.count());
  mpc_.input_weight.resize(z_rotors_.count());
  mpc_.control_weight.resize(kCtrlSize);
  mpc_.input_rate_constraint.resize(z_rotors_.count(), 0);
  setInputConstraintBase();
  mpc_.control_constraint.resize(kCtrlSize, 0);
  mpc_.current_state.resize(kStateSize);
  mpc_.set_state.resize(kCtrlSize);
  mpc_.last_input = VectorXd::Zero(z_rotors_.count());
}

void RotationController::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  c2d_.resize(kStateSize, z_rotors_.count());

  setScales();
  setInputConstraintBase();
  mpc_.last_input = VectorXd::Zero(z_rotors_.count());
  mpc_.input_rate_weight.resize(z_rotors_.count());
  mpc_.input_rate_constraint.resize(z_rotors_.count(), 0);

  // 推力の和が一定だから，推力の二乗和の重みが大きいほど各プロペラの推力を均等にしようとする力が働く．
  // 回転翼機の制御においてそれは致命傷になりうるため，推力の重みは0で固定する．
  mpc_.input_weight = VectorXd::Zero(z_rotors_.count());
}

void RotationController::update(
  const Euler& cur_rpy,
  const Twist& cur_twist_B,
  const JntArray& q,
  double battery_voltage,
  double thrust_sum,
  const Euler& tar_rpy,
  VectorXd& u_opt)
{
  assert(battery_voltage > 0.);

  // 目標姿勢を制限
  const auto tar_roll = clamp(tar_rpy.roll, -kMaxAttitude, kMaxAttitude);
  const auto tar_pitch = clamp(tar_rpy.pitch, -kMaxAttitude, kMaxAttitude);
  const auto yaw_error = clamp(tar_rpy.yaw - cur_rpy.yaw, -kMaxHeadingError, kMaxHeadingError);
  const auto tar_yaw = cur_rpy.yaw + yaw_error;

  // 目標推力を制限
  const auto max_thrust_sum = maxThrustSum(battery_voltage);
  const auto min_thrust_sum = minThrustSum(battery_voltage);
  thrust_sum = clamp(thrust_sum, min_thrust_sum, max_thrust_sum);

  // MPCの最適制御問題を構築
  updateCurrentState(cur_rpy, cur_twist_B, q, thrust_sum);
  updateSetState(tar_roll, tar_pitch, tar_yaw);
  updateDynamics(cur_rpy, tar_roll, tar_pitch, q);
  updateInputConstraint(battery_voltage, thrust_sum);

  // MPCを解く
  u_opt = mpc_.solveMPC();

  // For debug
  // cout << mpc_ << endl;
}

void RotationController::configure(const RotationControllerDynamicParams& params)
{
  assert(params.pred_horizon > 0.);
  assert(params.pred_steps > 0);
  assert(params.attitude_decay >= 0.);
  assert(params.heading_decay >= 0.);
  assert(params.angvel_decay >= 0.);
  assert(params.attitude_weight > 0.);
  assert(params.heading_weight > 0.);
  assert(params.angvel_weight > 0.);

  mpc_.time_step = params.pred_horizon / params.pred_steps;
  mpc_.prediction_steps = mpc_.input_steps = params.pred_steps;
  mpc_.decay_time_consts(kRollIdx) = mpc_.decay_time_consts(kPitchIdx) = params.attitude_decay;
  mpc_.decay_time_consts(kYawIdx) = params.heading_decay;
  mpc_.decay_time_consts.block<3, 1>(kGyroIdx, 0).fill(params.angvel_decay);

  mpc_.discrete_dynamics.resize(
    params.pred_steps, ctrl::LinearDynamics(kStateSize, z_rotors_.count()));

  // Update weights
  mpc_.control_weight(kRollIdx) = mpc_.control_weight(kPitchIdx) = params.attitude_weight;
  mpc_.control_weight(kYawIdx) = params.heading_weight;
  mpc_.control_weight.block<3, 1>(kGyroIdx, 0).fill(params.angvel_weight);
  mpc_.input_rate_weight.fill(exp10(params.thrust_rate_weight_log10));
}

double RotationController::maxThrustSum(double battery_voltage) const
{
  double res = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, battery_voltage);
  }
  return res;
}

double RotationController::minThrustSum(double battery_voltage) const
{
  const auto min_voltage = battery_voltage * tobas::kMotorSpinArm;
  double res = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, min_voltage);
  }
  return res;
}

void RotationController::updateCurrentState(
  const Euler& cur_rpy,
  const Twist& cur_twist_B,
  const JntArray& q,
  double thrust_sum)
{
  const auto& vel = cur_twist_B.vel;
  const auto& gyro = cur_twist_B.rot;

  // 機体速度のプロペラに対する水平成分を求める．機体座標系ではZ成分のみ0としたベクトルに等しい．
  const Vector vel_perp(vel.x(), vel.y(), 0);

  // 重心を求める
  inertia_solver_.JntToCart(q, P_base_cog_, I_cog_);

  // 簡単のため全プロペラの推力が等しいとしてH-forceの和を計算
  // TODO: より真値に近い回転数を用いて計算
  const double thrust_mean = thrust_sum / z_rotors_.count();
  Vector sum = Vector::Zero();
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    // CoG -> Rotor の位置を求める
    fk_solver_.JntToCart(q, z_rotors_.linkName(i), T_base_rotor_);
    const Vector P_cog_rotor = T_base_rotor_.p - P_base_cog_;

    const double rot_speed = z_rotors_.rotSpeedFromThrust(i, thrust_mean);
    sum += z_rotors_.dragConstant(i) * rot_speed * P_cog_rotor;
  }
  const Vector h_moment = -sum * vel_perp;
  // cout << "H-moment [Nm]: " << h_moment << endl;

  // 現在の状態を更新
  mpc_.current_state << cur_rpy.roll, cur_rpy.pitch, cur_rpy.yaw, gyro.x(), gyro.y(), gyro.z(),
    h_moment.x(), h_moment.y(), h_moment.z();
}

void RotationController::updateSetState(double tar_roll, double tar_pitch, double tar_yaw)
{
  mpc_.set_state << tar_roll, tar_pitch, tar_yaw, 0., 0., 0.;
}

void RotationController::updateDynamics(
  const Euler& cur_rpy,
  double tar_roll,
  double tar_pitch,
  const JntArray& q)
{
  double t;
  double roll_k, pitch_k;

  for (uint32_t k = 0; k < mpc_.prediction_steps; ++k)
  {
    t = mpc_.time_step * k;  // 計画開始時刻(= 0)からの経過時間

    // 時刻tにおけるドローンの姿勢の参照値
    roll_k = ctrl::firstOrderPos(cur_rpy.roll, tar_roll, mpc_.decay_time_consts(kRollIdx), t);
    pitch_k = ctrl::firstOrderPos(cur_rpy.pitch, tar_pitch, mpc_.decay_time_consts(kPitchIdx), t);

    cont_.update(roll_k, pitch_k, q);
    mpc_.discrete_dynamics[k] = c2d_.convert(cont_, mpc_.time_step);

    // For debug
    // cout << cont_ << endl;
  }
}

void RotationController::setScales()
{
  // 状態変数のスケール
  mpc_.state_scale.resize(kStateSize);
  mpc_.state_scale.block<3, 1>(kRotIdx, 0).fill(M_PI);
  mpc_.state_scale.block<3, 1>(kGyroIdx, 0).fill(M_PI);

  // 制御変数は状態変数と等しい
  mpc_.control_scale = mpc_.state_scale.topRows(kCtrlSize);

  // 制御入力のスケール
  mpc_.input_scale.resize(z_rotors_.count());
  mpc_.input_scale.fill(tobas::getMass() * tobas::kGravity / z_rotors_.count());
}

void RotationController::setInputConstraintBase()
{
  const MatrixXd E = MatrixXd::Identity(z_rotors_.count(), z_rotors_.count());
  const VectorXd ones = VectorXd::Ones(z_rotors_.count());

  mpc_.input_constraint.resize(z_rotors_.count(), z_rotors_.count() * 2 + 2);

  mpc_.input_constraint.A.block(0, 0, z_rotors_.count(), z_rotors_.count()) = E;
  mpc_.input_constraint.A.block(z_rotors_.count(), 0, z_rotors_.count(), z_rotors_.count()) = -E;
  mpc_.input_constraint.A.block(z_rotors_.count() * 2, 0, 1, z_rotors_.count()) = ones.transpose();
  mpc_.input_constraint.A.block(z_rotors_.count() * 2 + 1, 0, 1, z_rotors_.count()) =
    -ones.transpose();
}

void RotationController::updateInputConstraint(double battery_voltage, double U)
{
  const auto min_voltage = battery_voltage * tobas::kMotorSpinArm;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    mpc_.input_constraint.b(i) = z_rotors_.thrustFromVoltage(i, battery_voltage);
    mpc_.input_constraint.b(z_rotors_.count() + i) = -z_rotors_.thrustFromVoltage(i, min_voltage);
  }

  mpc_.input_constraint.b(z_rotors_.count() * 2) = U;
  mpc_.input_constraint.b(z_rotors_.count() * 2 + 1) = -U;
}
}  // namespace tobas_mr_rotation_mpc
