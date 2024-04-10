#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_linear_control/util.hpp>
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
    z_rotors_(drone, tobas::Axis::Z_POSITIVE),
    dynamics_(drone),
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
  mpc_.state_scale.segment<3>(kRotIdx).fill(M_PI);
  mpc_.state_scale.segment<3>(kGyroIdx).fill(M_PI);
  mpc_.state_scale.segment<3>(kHForceIdx).fill(kHMomentScale);

  // 制御変数のスケールは状態変数と等しい
  mpc_.control_scale = mpc_.state_scale.head(kCtrlSize);

  mpc_.control_weight.resize(kCtrlSize);
  mpc_.current_state.resize(kStateSize);
  mpc_.set_state.resize(kCtrlSize);

  updateInternalDataStructures();
}

void OrientationController::updateInternalDataStructures()
{
  z_rotors_.updateInternalDataStructures();
  dynamics_.updateInternalDataStructures();
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
  const double& dt,
  const Rotation& cur_rot,
  const Twist& cur_twist_B,
  const Vector& cur_wind_W,
  const JntArray& cur_q,
  const double& cur_voltage,
  const vector<double>& cur_rot_speeds,
  const double& tar_thrust,
  const Rotation& tar_rot)
{
  assert(cur_voltage > 0);

  // Rotation -> Euler
  // FIXME: wrap_piを行う必要がある
  const Euler cur_rpy(cur_rot);
  const Euler tar_rpy(tar_rot);

  // 現在の姿勢と合計推力から，重力方向の推力を計算
  const auto thrust_z = tar_thrust * cos(cur_rpy.roll) * cos(cur_rpy.pitch);

  // MPCの最適制御問題を構築
  updateCurrentState(cur_rpy, cur_twist_B, cur_wind_W, cur_q, thrust_z);
  updateSetState(tar_rpy);

  const auto max_thrust_sum = dynamics_.maxThrustSum(cur_voltage);
  const auto min_thrust_sum = dynamics_.minThrustSum(cur_voltage);

  // ダイナミクスと制御入力の制約を更新
  // 姿勢や推力の目標値をそのまま使うと追従性能が悪い場合に想定外の動きになるため，
  // 参照起動からダイナミクスや制約を構成する．
  // 初期状態のダイナミクスを保持するため，後ろから順に処理する
  for (int k = mpc_.prediction_steps - 1; k >= 0; --k)
  {
    const double t = mpc_.time_step * k;  // 計画開始時刻 (= 0) からの経過時間

    // ダイナミクスを更新
    const auto roll_k =
      ctrl::firstOrderPos(cur_rpy.roll, tar_rpy.roll, mpc_.decay_time_consts(kRollIdx), t);
    const auto pitch_k =
      ctrl::firstOrderPos(cur_rpy.pitch, tar_rpy.pitch, mpc_.decay_time_consts(kPitchIdx), t);
    cont_.update(roll_k, pitch_k, cur_q);
    mpc_.discrete_dynamics[k] = c2d_.convert(cont_, mpc_.time_step);

    // 個々のプロペラの推力の限界に関する不等式制約
    for (size_t i = 0; i < z_rotors_.count(); ++i)
    {
      mpc_.input_ineqs[k].b(i) = z_rotors_.maxThrust(i, cur_voltage);
      mpc_.input_ineqs[k].b(z_rotors_.count() + i) = -z_rotors_.minThrust(i, cur_voltage);
    }

    // 全てのプロペラの推力の合計に関する等式制約
    const auto thrust_k =
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
  const Vector3d dgyro_mpc = xd.segment<3>(kGyroIdx);

  // 外乱補償用の微分先行型PD
  const Vector error_B = (cur_rot.inverse() * tar_rot).getRot();
  const Vector dgyro_pd = kp_ * error_B - kd_ * cur_twist_B.rot;

  // Mixerで最終的な推力を計算
  const Vector3d dgyro_des = dgyro_mpc + dgyro_pd.data;  // FF + FB (二自由度制御)
  const Vector h_moment_raw =
    dynamics_.horizontalMoment(cur_rot, cur_twist_B.vel, cur_wind_W, cur_q, cur_rot_speeds);
  const Vector h_moment_comp = h_force_comp_rate_ * h_moment_raw;
  return mixer_.solve(
    dt, cur_voltage, cur_q, cur_twist_B.rot.data, h_moment_comp.data, dgyro_des, thrusts_des);
}

void OrientationController::configure(const OrientationControllerConfig& cfg)
{
  CHECK(0 <= cfg.h_force_comp_rate && cfg.h_force_comp_rate <= 1);
  CHECK(cfg.kp >= 0);
  CHECK(cfg.kd >= 0);
  CHECK(cfg.pred_horizon > 0);
  CHECK(cfg.pred_steps > 0);
  CHECK(cfg.attitude_decay >= 0);
  CHECK(cfg.heading_decay >= 0);
  CHECK(cfg.angvel_decay >= 0);
  CHECK(cfg.attitude_weight > 0);
  CHECK(cfg.heading_weight > 0);
  CHECK(cfg.angvel_weight > 0);

  h_force_comp_rate_ = cfg.h_force_comp_rate;
  kp_ = cfg.kp;
  kd_ = cfg.kd;

  mpc_.time_step = cfg.pred_horizon / cfg.pred_steps;
  mpc_.prediction_steps = mpc_.input_steps = cfg.pred_steps;

  mpc_.decay_time_consts(kRollIdx) = mpc_.decay_time_consts(kPitchIdx) = cfg.attitude_decay;
  mpc_.decay_time_consts(kYawIdx) = cfg.heading_decay;
  mpc_.decay_time_consts.segment<3>(kGyroIdx).fill(cfg.angvel_decay);

  mpc_.discrete_dynamics.resize(cfg.pred_steps);

  mpc_.control_weight(kRollIdx) = mpc_.control_weight(kPitchIdx) = cfg.attitude_weight;
  mpc_.control_weight(kYawIdx) = cfg.heading_weight;
  mpc_.control_weight.segment<3>(kGyroIdx).fill(cfg.angvel_weight);
  mpc_.input_rate_weight.fill(exp10(cfg.thrust_rate_weight_log10));

  mpc_.input_rate_eqs.resize(cfg.pred_steps, ctrl::LinearEquation(z_rotors_.count(), 0));
  mpc_.control_eqs.resize(cfg.pred_steps, ctrl::LinearEquation(kCtrlSize, 0));
  mpc_.input_rate_ineqs.resize(cfg.pred_steps, ctrl::LinearEquation(z_rotors_.count(), 0));
  mpc_.control_ineqs.resize(cfg.pred_steps, ctrl::LinearEquation(kCtrlSize, 0));

  mpc_.input_eqs.resize(mpc_.prediction_steps);
  mpc_.input_ineqs.resize(mpc_.prediction_steps);
  fillInputConstraintFixedParts();
}

const VectorXd& OrientationController::mpcThrusts() const
{
  return mpc_.optimalControlInput();
}

void OrientationController::updateCurrentState(
  const Euler& cur_rpy,
  const Twist& cur_twist_B,
  const Vector& cur_wind_W,
  const JntArray& cur_q,
  const double& thrust_z)
{
  // 簡単のため全プロペラの推力が等しいとしてプロペラの回転数を計算
  // TODO: 予測区間での推力の変化を反映し，より真値に近い回転数を用いて計算
  const double thrust_sum = thrust_z / (cos(cur_rpy.roll) * cos(cur_rpy.pitch));  // 合計推力
  const double thrust_mean = thrust_sum / z_rotors_.count();
  vector<double> rot_speeds(drone_.numRotors(), 0);
  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    const double rot_speed = z_rotors_.rotSpeedFromThrust(i, thrust_mean);
    rot_speeds[z_rotors_.rotorIdx(i)] = rot_speed;
  }

  // H-forceによるモーメントを計算
  // TODO: H-momentの時間変化を考慮
  const Vector h_moment_raw = dynamics_.horizontalMoment(
    cur_rpy.toRotation(), cur_twist_B.vel, cur_wind_W, cur_q, rot_speeds);
  const Vector h_moment_comp = h_moment_raw * h_force_comp_rate_;  // H-momentの補償分

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
