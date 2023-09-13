#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_linear_control/util.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../include/tobas_multirotor_controller/rotation_controller.hpp"
#include "../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_multirotor_controller
{
RotationController::RotationController(const tobas::Drone& drone)
  : drone_(drone),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE),
    cont_(drone),
    c2d_(STATE_SIZE, z_rotors_.count())
{
  mpc_.Cz = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  mpc_.decay_time_consts.resize(STATE_SIZE);
  setScales();
  mpc_.input_rate_weight.resize(z_rotors_.count());
  mpc_.input_weight.resize(z_rotors_.count());
  mpc_.control_weight.resize(STATE_SIZE);
  mpc_.input_rate_constraint.resize(z_rotors_.count(), 0);
  setInputConstraintBase();
  mpc_.control_constraint.resize(STATE_SIZE, 0);
  mpc_.current_state.resize(STATE_SIZE);
  mpc_.set_state.resize(STATE_SIZE);
  mpc_.last_input = VectorXd::Zero(z_rotors_.count());
}

void RotationController::updateInternalDataStructures()
{
  z_rotors_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  c2d_.resize(STATE_SIZE, z_rotors_.count());

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
  const Vector& cur_angvel_B,
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
  updateCurrentState(cur_rpy, cur_angvel_B);
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
  mpc_.decay_time_consts(ROLL) = mpc_.decay_time_consts(PITCH) = params.attitude_decay;
  mpc_.decay_time_consts(YAW) = params.heading_decay;
  mpc_.decay_time_consts(ANGVEL_X) = mpc_.decay_time_consts(ANGVEL_Y) =
    mpc_.decay_time_consts(ANGVEL_Z) = params.angvel_decay;

  mpc_.discrete_dynamics.resize(
    params.pred_steps, ctrl::LinearDynamics(STATE_SIZE, z_rotors_.count()));

  // Update weights
  mpc_.control_weight(ROLL) = mpc_.control_weight(PITCH) = params.attitude_weight;
  mpc_.control_weight(YAW) = params.heading_weight;
  mpc_.control_weight(ANGVEL_X) = mpc_.control_weight(ANGVEL_Y) = mpc_.control_weight(ANGVEL_Z) =
    params.angvel_weight;
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

void RotationController::updateCurrentState(const Euler& cur_rpy, const Vector& cur_angvel_B)
{
  mpc_.current_state << cur_rpy.roll, cur_rpy.pitch, cur_rpy.yaw, cur_angvel_B.x(),
    cur_angvel_B.y(), cur_angvel_B.z();
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
    roll_k = ctrl::firstOrderPos(cur_rpy.roll, tar_roll, mpc_.decay_time_consts(ROLL), t);
    pitch_k = ctrl::firstOrderPos(cur_rpy.pitch, tar_pitch, mpc_.decay_time_consts(PITCH), t);

    cont_.update(roll_k, pitch_k, q);
    mpc_.discrete_dynamics[k] = c2d_.convert(cont_, mpc_.time_step);

    // For debug
    // cout << cont_ << endl;
  }
}

void RotationController::setScales()
{
  // 状態変数のスケール
  mpc_.state_scale.resize(STATE_SIZE);
  mpc_.state_scale(ROLL) = mpc_.state_scale(PITCH) = mpc_.state_scale(YAW) = M_PI;
  mpc_.state_scale(ANGVEL_X) = mpc_.state_scale(ANGVEL_Y) = mpc_.state_scale(ANGVEL_Z) = M_PI;

  // 制御変数は状態変数と等しい
  mpc_.control_scale = mpc_.state_scale;

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
}  // namespace tobas_multirotor_controller
