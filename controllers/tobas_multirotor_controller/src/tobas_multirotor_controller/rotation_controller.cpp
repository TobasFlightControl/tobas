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

namespace tobas_multirotor_controller
{
RotationController::RotationController(
  const Drone& drone,
  const Tree& tree,
  double gravity,
  const RotationControllerDynamicParams& params)
  : drone_(drone),
    gravity_(gravity),
    u_dim_(drone.numRotorsInAxis(Axis::Z_POSITIVE)),
    cont_(drone, tree),
    c2d_(STATE_SIZE, u_dim_)
{
  assert(tree.getNrOfJoints() > 0);

  TreeJntToInertiaSolver inertia_solver_(tree);
  mass_ = inertia_solver_.JntToMass();

  mpc_.decay_time_consts.resize(STATE_SIZE);
  mpc_.Cz = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  mpc_.input_rate_weight.resize(u_dim_);
  mpc_.input_weight.resize(u_dim_);
  mpc_.control_weight.resize(STATE_SIZE);
  mpc_.input_rate_constraint.resize(u_dim_, 0);
  setInputConstraintBase();
  mpc_.control_constraint.resize(STATE_SIZE, 0);
  mpc_.last_input = VectorXd::Zero(u_dim_);

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
  assert(u_opt.rows() == u_dim_);

  updateDynamics(cur_rpy, tar_rpy, q);
  updateInputConstraint(U);

  mpc_.current_state = eigen_tools::concat(cur_rpy, cur_angvel, 0);
  mpc_.set_state = eigen_tools::concat(tar_rpy, ZERO3, 0);

  u_opt = mpc_.solveMPC();
}

void RotationController::reconfigure(const RotationControllerDynamicParams& params)
{
  assert(params.pred_horizon > 0.);
  assert(params.pred_steps > 0);
  assert(params.rot_decay >= 0.);
  assert(params.angvel_decay >= 0.);
  assert(params.rot_weight > 0.);
  assert(params.angvel_weight > 0.);

  mpc_.time_step = params.pred_horizon / params.pred_steps;
  mpc_.prediction_steps = mpc_.input_steps = params.pred_steps;
  mpc_.decay_time_consts[ROLL] = mpc_.decay_time_consts[PITCH] = mpc_.decay_time_consts[YAW] =
    params.rot_decay;
  mpc_.decay_time_consts[ANGVEL_X] = mpc_.decay_time_consts[ANGVEL_Y] =
    mpc_.decay_time_consts[ANGVEL_Z] = params.angvel_decay;

  mpc_.discrete_dynamics.resize(params.pred_steps, ctrl::LinearDynamics(STATE_SIZE, u_dim_));

  updateWeight_Q(params.rot_weight, params.angvel_weight);
  updateWeight_S(params.thrust_weight);
  updateWeight_R(params.thrust_rate_weight, mpc_.time_step);
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

  for (int k = 0; k < mpc_.prediction_steps; ++k)
  {
    t = mpc_.time_step * k;  // 計画開始時刻(= 0)からの経過時間

    // 時刻tにおけるドローンの姿勢の参照値
    roll_k = ctrl::firstOrderPos(cur_roll, tar_roll, mpc_.decay_time_consts[ROLL], t);
    pitch_k = ctrl::firstOrderPos(cur_pitch, tar_pitch, mpc_.decay_time_consts[PITCH], t);

    cont_.update(roll_k, pitch_k, q);
    mpc_.discrete_dynamics[k] = c2d_.convert(cont_, mpc_.time_step);
  }
}

void RotationController::updateWeight_Q(double rot_weight, double angvel_weight)
{
  constexpr double rot_scale = M_PI;
  constexpr double angvel_scale = M_PI;

  mpc_.control_weight(ROLL) = mpc_.control_weight(PITCH) = mpc_.control_weight(YAW) =
    rot_weight / sqr(rot_scale) * WEIGHT_SCALER;
  mpc_.control_weight(ANGVEL_X) = mpc_.control_weight(ANGVEL_Y) = mpc_.control_weight(ANGVEL_Z) =
    angvel_weight / sqr(angvel_scale) * WEIGHT_SCALER;
}

void RotationController::updateWeight_S(int thrust_weight)
{
  double u_scale = mass_ * gravity_;
  double S_value = pow(10, thrust_weight) / sqr(u_scale) * WEIGHT_SCALER;
  mpc_.input_weight = VectorXd::Constant(u_dim_, S_value);
}

void RotationController::updateWeight_R(int thrust_rate_weight, double dt)
{
  double delta_u_scale = mass_ * gravity_ * dt;
  double R_value = pow(10, thrust_rate_weight) / sqr(delta_u_scale) * WEIGHT_SCALER;
  mpc_.input_rate_weight = VectorXd::Constant(u_dim_, R_value);
}

void RotationController::setInputConstraintBase()
{
  const MatrixXd E = MatrixXd::Identity(u_dim_, u_dim_);
  const VectorXd ones = VectorXd::Ones(u_dim_);

  mpc_.input_constraint.resize(u_dim_, u_dim_ * 2 + 2);

  mpc_.input_constraint.A.block(0, 0, u_dim_, u_dim_) = E;
  mpc_.input_constraint.A.block(u_dim_, 0, u_dim_, u_dim_) = -E;
  mpc_.input_constraint.A.block(u_dim_ * 2, 0, 1, u_dim_) = ones.transpose();
  mpc_.input_constraint.A.block(u_dim_ * 2 + 1, 0, 1, u_dim_) = -ones.transpose();

  const auto rotor_idxes_in_use = drone_.rotorConfigIdxInAxis(Axis::Z_POSITIVE);
  for (int i = 0; i < u_dim_; ++i)
  {
    const auto& rotor_idx = rotor_idxes_in_use[i];
    const double max_thrust = drone_.maxThrust(rotor_idx);
    const double min_thrust = 0.;

    mpc_.input_constraint.b(i) = max_thrust;
    mpc_.input_constraint.b(u_dim_ + i) = -min_thrust;
  }
}

void RotationController::updateInputConstraint(double U)
{
  mpc_.input_constraint.b(u_dim_ * 2) = U;
  mpc_.input_constraint.b(u_dim_ * 2 + 1) = -U;
}
}  // namespace tobas_multirotor_controller
