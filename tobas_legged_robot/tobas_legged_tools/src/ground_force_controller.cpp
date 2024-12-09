#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_linear_control/util.hpp>

#include "../include/tobas_legged_tools/ground_force_controller.hpp"

using namespace std;
using namespace Eigen;

namespace lr_tools
{
GroundForceController::GroundForceController(const kdl::Tree& tree, const vector<string>& foot_names)
  : tree_(tree),
    foot_names_(foot_names),
    nc_(foot_names.size()),
    inertia_solver_(tree),
    bb_solver_(tree),
    cont_(tree, foot_names),
    c2d_(cont_.stateSize(), cont_.inputSize())
{
  initializeMPC();
}

bool GroundForceController::updateInternalDataStructures()
{
  if (!inertia_solver_.updateInternalDataStructures())
    return false;
  if (!bb_solver_.updateInternalDataStructures())
    return false;

  if (!cont_.updateInternalDataStructures())
    return false;

  initializeMPC();

  return true;
}

bool GroundForceController::configure(const GroundForceControllerConfig& cfg)
{
  // TODO: 有効な値かチェック

  friction_coef_ = cfg.friction_coef;
  foot_diameter_ = cfg.foot_diameter;

  normal_force_range_.lower = cfg.min_normal_force;
  normal_force_range_.upper = cfg.max_normal_force;

  mpc_.discrete_dynamics.resize(cfg.prediction_steps);
  mpc_.prediction_steps = cfg.prediction_steps;
  mpc_.input_steps = cfg.prediction_steps;
  mpc_.time_step = cfg.prediction_horizon / cfg.prediction_steps;

  mpc_.decay_time_consts.segment<2>(cont_.kRollIdx).fill(cfg.attitude_error_decay);
  mpc_.decay_time_consts(cont_.kAltIdx) = cfg.height_error_decay;
  mpc_.decay_time_consts.segment<2>(cont_.kGyroXIdx).setZero();
  mpc_.decay_time_consts(cont_.kGyroZIdx) = cfg.yawrate_error_decay;
  mpc_.decay_time_consts.segment<2>(cont_.kVelXIdx).fill(cfg.velocity_error_decay);
  mpc_.decay_time_consts(cont_.kVelZIdx) = 0.;

  mpc_.control_weight.segment<2>(cont_.kRollIdx).fill(cfg.attitude_weight);
  mpc_.control_weight(cont_.kAltIdx) = cfg.height_weight;
  mpc_.control_weight.segment<3>(cont_.kGyroXIdx).fill(cfg.gyro_weight);
  mpc_.control_weight.segment<3>(cont_.kVelXIdx).fill(cfg.velocity_weight);

  mpc_.input_weight.fill(exp10(cfg.force_weight_log10));
  mpc_.input_rate_weight.fill(exp10(cfg.force_rate_weight_log10));

  mpc_.input_rate_eqs.resize(cfg.prediction_steps, ctrl::LinearEquation(cont_.inputSize(), 0));
  mpc_.input_eqs.resize(cfg.prediction_steps, ctrl::LinearEquation(cont_.inputSize(), 0));
  mpc_.control_eqs.resize(cfg.prediction_steps, ctrl::LinearEquation(kCtrlSize, 0));

  mpc_.input_rate_ineqs.resize(cfg.prediction_steps, ctrl::LinearEquation(cont_.inputSize(), 0));
  mpc_.input_ineqs.resize(cfg.prediction_steps, makeInputConstraint());
  mpc_.control_ineqs.resize(cfg.prediction_steps, ctrl::LinearEquation(kCtrlSize, 0));

  return true;
}

bool GroundForceController::solve(
  const double& cur_z,
  const kdl::Vector& cur_vel,
  const kdl::Vector& cur_gyro,
  const double& tar_z,
  const double& tar_yawrate,
  const double& tar_vx,
  const double& tar_vy,
  const vector<double>& roll_pred,
  const vector<double>& pitch_pred,
  const vector<kdl::JntArray>& q_pred,
  const vector<vector<bool>>& is_stand_pred)
{
  assert(q_pred.size() == static_cast<size_t>(mpc_.prediction_steps));
  assert(tobas_std::allOf(q_pred, [this](const kdl::JntArray& q) { return q.size() == tree_.getNrOfJoints(); }));
  assert(roll_pred.size() == static_cast<size_t>(mpc_.prediction_steps));
  assert(pitch_pred.size() == static_cast<size_t>(mpc_.prediction_steps));
  assert(is_stand_pred.size() == static_cast<size_t>(mpc_.prediction_steps));

  // Update dynamics
  // ステップ0の連続時間ダイナミクスを後の予測で使うため，未来から逆順で処理する．
  for (Index k = mpc_.prediction_steps - 1; k >= 0; --k)
  {
    cont_.update(roll_pred[k], pitch_pred[k], q_pred[k], is_stand_pred[k]);
    mpc_.discrete_dynamics[k] = c2d_.convert(cont_, mpc_.time_step);
  }

  // Update current state
  mpc_.current_state << roll_pred[0], pitch_pred[0], cur_z, cur_gyro.x(), cur_gyro.y(), cur_gyro.z(), cur_vel.x(),
    cur_vel.y(), cur_vel.z(), tobas_std::kGravity;

  // Update set state
  mpc_.set_state << 0, 0, tar_z, 0, 0, tar_yawrate, tar_vx, tar_vy, 0;

  // Solve MPC
  if (!mpc_.solve())
    return false;

  // Prediction
  const auto& u = mpc_.optimalControlInput();
  x_rate_ = cont_.A * mpc_.current_state + cont_.B * u;
  x_next_ = mpc_.discrete_dynamics[0].A * mpc_.current_state + mpc_.discrete_dynamics[0].B * u;

  return true;
}

void GroundForceController::initializeMPC()
{
  mpc_.Cz = makeCz();
  mpc_.decay_time_consts.resize(kCtrlSize);
  mpc_.control_weight.resize(kCtrlSize);
  mpc_.input_weight.resize(cont_.inputSize());
  mpc_.input_rate_weight.resize(cont_.inputSize());

  const double rpy_sc = M_PI;
  const double size_sc = calcSizeScale();
  const double gyro_sc = M_PI;
  const double vel_sc = sqrt(tobas_std::kGravity * size_sc);  // フルード数の定義を元に決定
  mpc_.state_scale.resize(cont_.stateSize());
  mpc_.input_scale.resize(cont_.inputSize());
  mpc_.control_scale.resize(kCtrlSize);
  mpc_.state_scale << rpy_sc, rpy_sc, size_sc, gyro_sc, gyro_sc, gyro_sc, vel_sc, vel_sc, vel_sc, tobas_std::kGravity;
  mpc_.input_scale.fill(calcMass() * tobas_std::kGravity / nc_);  // TODO: 力とトルクでスケールを分ける
  mpc_.control_scale << rpy_sc, rpy_sc, size_sc, gyro_sc, gyro_sc, gyro_sc, vel_sc, vel_sc, vel_sc;

  mpc_.current_state.resize(cont_.stateSize());
  mpc_.set_state.resize(kCtrlSize);
}

double GroundForceController::calcMass()
{
  inertia_solver_.JntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints()));
  return inertia_solver_.getInertia().getMass();
}

double GroundForceController::calcSizeScale()
{
  bb_solver_.solve(kdl::JntArray::Zero(tree_.getNrOfJoints()));
  return bb_solver_.diagonalLength();
}

MatrixXd GroundForceController::makeCz()
{
  MatrixXd res = MatrixXd::Zero(kCtrlSize, cont_.stateSize());
  res.diagonal().head<kCtrlSize>().setOnes();
  return res;
}

ctrl::LinearEquation GroundForceController::makeInputConstraint()
{
  const auto& u = friction_coef_;
  const auto& d = foot_diameter_;
  const auto& f_min = normal_force_range_.lower;
  const auto& f_max = normal_force_range_.upper;

  const auto ud = u * d;

  // 足1本について
  MatrixXd F1(kNumConstraintsPerLeg, LinearDynamics::kInputSizePerLeg);
  F1 << 0, 0, -1, 0, 0, 0, 1, 0, -1, 0, -u, 0, 1, 0, -u, 0, 0, -1, -u, 0, 0, 1, -u, 0, 0, 0, -ud, -1, 0, 0, -ud, 1;
  VectorXd f1(kNumConstraintsPerLeg);
  f1 << -f_min, f_max, 0, 0, 0, 0, 0, 0;

  // 全ての足について
  const auto F = eigen::blockDiag(F1, nc_);
  const auto f = eigen::tile(f1, nc_, 0);

  return ctrl::LinearEquation(F, f);
}
}  // namespace lr_tools
