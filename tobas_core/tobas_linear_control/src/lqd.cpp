#include <iostream>
#include <Eigen/LU>

#include <tobas_eigen_tools/core.hpp>

#include "../include/tobas_linear_control/lqd.hpp"
#include "../include/tobas_linear_control/care.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
LQD::LQD()
{
}

VectorXd LQD::solve(const double& dt, const bool& update_gain)
{
  assert(dt >= 0);
  checkProblemValidity();

  if (update_gain)
    updateGain();

  // スケーリング
  const VectorXd x_scaled = current_state.array() / state_scale.array();
  const VectorXd s_scaled = target_state.array() / state_scale.array();
  const VectorXd last_u_scaled = last_input.array() / input_scale.array();

  // 拡大状態を作成
  const VectorXd x_tilde = eigen_tools::concat(x_scaled, last_u_scaled, 0);
  const VectorXd s_tilde = eigen_tools::concat(s_scaled, VectorXd::Zero(input_weight.rows()), 0);

  const auto ud_scaled = K_ * (s_tilde - x_tilde);

  // 最新の制御入力を更新
  const auto ud = ud_scaled.cwiseProduct(input_scale);
  last_input += ud * dt;

  return last_input;
}

void LQD::resize(const Index& state_size, const Index& input_size)
{
  dynamics.resize(state_size, input_size);

  state_scale.conservativeResize(state_size);
  input_scale.conservativeResize(input_size);

  state_weight.conservativeResize(state_size);
  input_weight.conservativeResize(input_size);
  input_rate_weight.conservativeResize(input_size);

  current_state.conservativeResize(state_size);
  target_state.conservativeResize(state_size);
  last_input.conservativeResize(input_size);
}

void LQD::updateGain()
{
  const auto x_size = current_state.rows();
  const auto u_size = input_weight.rows();
  const auto x_tilde_size = x_size + u_size;

  // スケーリング
  const auto dyn_scaled = dynamics.scale(state_scale, input_scale);

  // 拡大状態に対応するダイナミクスを作成
  MatrixXd A_tilde(x_tilde_size, x_tilde_size);
  A_tilde.topLeftCorner(x_size, x_size) = dyn_scaled.A;
  A_tilde.topRightCorner(x_size, u_size) = dyn_scaled.B;
  A_tilde.bottomRows(u_size).setZero();

  MatrixXd B_tilde(x_tilde_size, u_size);
  B_tilde.topRows(x_size).setZero();
  B_tilde.bottomRows(u_size).diagonal().setOnes();

  // 重み行列を作成
  const MatrixXd Q_tilde = eigen_tools::concat(state_weight, input_weight, 0).asDiagonal();
  const MatrixXd R_tilde = input_rate_weight.asDiagonal();

  // CAREを解く
  P_inf_ = care_ArimotoPotter(A_tilde, B_tilde, Q_tilde, R_tilde);

  // LQRの解を計算
  K_ = R_tilde.diagonal().cwiseInverse().asDiagonal() * B_tilde.transpose() * P_inf_;
}

void LQD::checkProblemValidity()
{
  [[maybe_unused]] const auto x_size = current_state.rows();
  [[maybe_unused]] const auto u_size = last_input.rows();

  assert(x_size > 0);
  assert(u_size > 0);

  assert(dynamics.stateSize() == x_size && dynamics.inputSize() == u_size);
  assert(dynamics.isFinite());

  assert(state_scale.rows() == x_size);
  assert(input_scale.rows() == u_size);
  assert(state_weight.rows() == x_size);
  assert(input_weight.rows() == u_size);
  assert(input_rate_weight.rows() == u_size);

  assert(eigen_tools::isFinite(state_scale));
  assert(eigen_tools::isFinite(input_scale));
  assert(eigen_tools::isFinite(state_weight));
  assert(eigen_tools::isFinite(input_weight));
  assert(eigen_tools::isFinite(input_rate_weight));

  assert((state_scale.array() > 0.).all());
  assert((input_scale.array() > 0.).all());
  assert((state_weight.array() >= 0.).all());
  assert((input_weight.array() >= 0.).all());
  assert((input_rate_weight.array() > 0.).all());
}

ostream& operator<<(ostream& os, const LQD& arg)
{
  os << "Dynamics:\n" << arg.dynamics << endl;
  os << "Current state:\n" << arg.current_state << endl;
  os << "Target state:\n" << arg.target_state << endl;
  os << "State error:\n" << arg.target_state - arg.current_state << endl;
  os << "Covariance matrix:\n" << arg.P_inf_ << endl;
  os << "Gain:\n" << arg.K_ << endl;

  return os;
}
}  // namespace ctrl
