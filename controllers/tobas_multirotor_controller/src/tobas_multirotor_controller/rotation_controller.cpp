#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_linear_control/util.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include "../../include/tobas_multirotor_controller/rotation_controller.hpp"

#define WEIGHT_SCALER 1e+6  // TODO: QPの数値エラーを防ぐために重みにかける定数を自動調整
#define ZERO3 Vector3d::Zero()

using namespace std;
using namespace Eigen;
using namespace KDL;

RotationController::RotationController(
  const Tree& tree,
  double gravity,
  double battery_voltage,
  const RotorConfigs& rotor_configs,
  const RotationControllerDynamicParams& params)
  : gravity_(gravity),
    battery_voltage_(battery_voltage),
    num_rotors_(rotor_configs.size()),
    T_refs_(STATE_SIZE),
    cont_(tree, rotor_configs),
    c2d_(STATE_SIZE, num_rotors_),
    Cz_(MatrixXd::Identity(STATE_SIZE, STATE_SIZE)),
    Q_(STATE_SIZE),
    S_(num_rotors_),
    R_(num_rotors_),
    E_e_(ctrl::LinearEquation(num_rotors_, 0)),
    F_f_(makeBaseInputCondition(rotor_configs)),
    G_g_(ctrl::LinearEquation(STATE_SIZE, 0))
{
  assert(tree.getNrOfJoints() > 0);

  TreeJntToInertiaSolver inertia_solver_(tree);
  mass_ = inertia_solver_.JntToMass();

  reconfigure(params);
}

void RotationController::update(
  const Vector3d& cur_rpy,
  const Vector3d& cur_angvel,
  const JntArray& q,
  const double& U,
  const Vector3d& tar_rpy,
  VectorXd& u_opt)
{
  assert(u_opt.rows() == num_rotors_);

  updateDynamics(cur_rpy, tar_rpy, q);
  updateInputCondition(U);

  VectorXd x = eigen_tools::concat(cur_rpy, cur_angvel, 0);
  VectorXd s = eigen_tools::concat(tar_rpy, ZERO3, 0);

  u_opt = ctrl::solveLinearDenseMPC(
    discs_, Cz_, Hp_, Hp_, dt_, T_refs_, R_, S_, Q_, E_e_, F_f_, G_g_, x, s, u_opt);
}

void RotationController::reconfigure(const RotationControllerDynamicParams& params)
{
  assert(params.pred_horizon > 0.);
  assert(params.pred_steps > 0);
  assert(params.rot_decay >= 0.);
  assert(params.angvel_decay >= 0.);
  assert(params.rot_weight > 0.);
  assert(params.angvel_weight > 0.);

  dt_ = params.pred_horizon / params.pred_steps;
  Hp_ = params.pred_steps;
  T_refs_[ROLL] = T_refs_[PITCH] = T_refs_[YAW] = params.rot_decay;
  T_refs_[ANGVEL_X] = T_refs_[ANGVEL_Y] = T_refs_[ANGVEL_Z] = params.angvel_decay;

  discs_.resize(params.pred_steps, ctrl::LinearDynamics(STATE_SIZE, num_rotors_));

  updateWeight_Q(params.rot_weight, params.angvel_weight);
  updateWeight_S(params.thrust_weight);
  updateWeight_R(params.thrust_rate_weight, dt_);
}

void RotationController::updateDynamics(
  const Vector3d& cur_rpy,
  const Vector3d& tar_rpy,
  const JntArray& q)
{
  const auto& cur_roll = cur_rpy.x();
  const auto& cur_pitch = cur_rpy.y();
  const auto& tar_roll = tar_rpy.x();
  const auto& tar_pitch = tar_rpy.y();

  double t;
  double roll_k, pitch_k;

  for (int k = 0; k < Hp_; ++k)
  {
    t = dt_ * k;  // 計画開始時刻(= 0)からの経過時間

    // 時刻tにおけるドローンの姿勢の参照値
    roll_k = ctrl::firstOrderPos(cur_roll, tar_roll, T_refs_[ROLL], t);
    pitch_k = ctrl::firstOrderPos(cur_pitch, tar_pitch, T_refs_[PITCH], t);

    cont_.update(roll_k, pitch_k, q);
    discs_[k] = c2d_.convert(cont_, dt_);
  }
}

void RotationController::updateWeight_Q(double rot_weight, double angvel_weight)
{
  // 姿勢と角速度のスケールは機体によって変化しないため，Qはスケーリングしない
  Q_(ROLL) = Q_(PITCH) = Q_(YAW) = rot_weight * WEIGHT_SCALER;
  Q_(ANGVEL_X) = Q_(ANGVEL_Y) = Q_(ANGVEL_Z) = angvel_weight * WEIGHT_SCALER;
}

void RotationController::updateWeight_S(int thrust_weight)
{
  double u_scale = mass_ * gravity_;
  double S_value = pow(10, thrust_weight) / sqr(u_scale) * WEIGHT_SCALER;

  for (uint32_t i = 0; i < num_rotors_; ++i)
  {
    S_(i) = S_value;
  }
}

void RotationController::updateWeight_R(int thrust_rate_weight, double dt)
{
  double delta_u_scale = mass_ * gravity_ * dt;
  double R_value = pow(10, thrust_rate_weight) / sqr(delta_u_scale) * WEIGHT_SCALER;

  for (uint32_t i = 0; i < num_rotors_; ++i)
  {
    R_(i) = R_value;
  }
}

ctrl::LinearEquation RotationController::makeBaseInputCondition(const RotorConfigs& rotor_configs)
{
  const MatrixXd E = MatrixXd::Identity(num_rotors_, num_rotors_);
  const VectorXd ones = VectorXd::Ones(num_rotors_);

  ctrl::LinearEquation F_f(num_rotors_, num_rotors_ * 2 + 2);

  F_f.A.block(0, 0, num_rotors_, num_rotors_) = E;
  F_f.A.block(num_rotors_, 0, num_rotors_, num_rotors_) = -E;
  F_f.A.block(num_rotors_ * 2, 0, 1, num_rotors_) = ones.transpose();
  F_f.A.block(num_rotors_ * 2 + 1, 0, 1, num_rotors_) = -ones.transpose();

  for (int i = 0; i < num_rotors_; ++i)
  {
    const auto& rotor_config = rotor_configs[i];

    const double max_speed = dh_std::rpmToRadPerSec(battery_voltage_ * rotor_config.kv);
    const double max_thrust = rotor_config.motor_constant * sqr(max_speed);
    const double min_thrust = 0.;

    F_f.b(i) = max_thrust;
    F_f.b(num_rotors_ + i) = -min_thrust;
  }

  return F_f;
}

void RotationController::updateInputCondition(const double& U)
{
  F_f_.b(num_rotors_ * 2) = U;
  F_f_.b(num_rotors_ * 2 + 1) = -U;
}
