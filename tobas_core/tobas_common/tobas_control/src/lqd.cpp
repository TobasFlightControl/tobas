// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/lqd.hpp"

#include <tobas_eigen_tools/core.hpp>

#include "tobas_control/care.hpp"

namespace tobas
{
namespace ctrl
{
LQD::LQD()
{
}

std::expected<Eigen::VectorXd, std::string> LQD::solve(double _dt, bool _update_gain)
{
  assert(_dt >= 0);
  checkProblemValidity();

  if (_update_gain) {
    if (const auto result = updateGainMatrix(); !result) {
      return std::unexpected("Failed to update gain matrix: " + result.error());
    }
  }

  // Scaling.
  const Eigen::VectorXd x_scaled = current_state.array() / state_scale.array();
  const Eigen::VectorXd s_scaled = target_state.array() / state_scale.array();
  const Eigen::VectorXd last_u_scaled = last_input.array() / input_scale.array();

  // Create the augmented state.
  const Eigen::VectorXd x_tilde = eigen::concat(x_scaled, last_u_scaled, Eigen::Vertical);
  const Eigen::VectorXd s_tilde = eigen::concat(s_scaled, Eigen::VectorXd::Zero(input_weight.rows()), Eigen::Vertical);

  const auto ud_scaled = K_ * (s_tilde - x_tilde);

  // Update the latest control input.
  const auto ud = ud_scaled.cwiseProduct(input_scale);
  last_input += ud * _dt;

  return last_input;
}

void LQD::resize(Eigen::Index _state_size, Eigen::Index _input_size)
{
  dynamics.resize(_state_size, _input_size);

  state_scale.conservativeResize(_state_size);
  input_scale.conservativeResize(_input_size);

  state_weight.conservativeResize(_state_size);
  input_weight.conservativeResize(_input_size);
  input_rate_weight.conservativeResize(_input_size);

  current_state.conservativeResize(_state_size);
  target_state.conservativeResize(_state_size);
  last_input.conservativeResize(_input_size);
}

std::expected<void, std::string> LQD::updateGainMatrix()
{
  const auto x_size = current_state.rows();
  const auto u_size = input_weight.rows();
  const auto x_tilde_size = x_size + u_size;

  // Scaling.
  const auto dyn_scaled = dynamics.scale(state_scale, input_scale);

  // Create dynamics for the augmented state.
  Eigen::MatrixXd A_tilde(x_tilde_size, x_tilde_size);
  A_tilde.topLeftCorner(x_size, x_size) = dyn_scaled.A;
  A_tilde.topRightCorner(x_size, u_size) = dyn_scaled.B;
  A_tilde.bottomRows(u_size).setZero();

  Eigen::MatrixXd B_tilde(x_tilde_size, u_size);
  B_tilde.topRows(x_size).setZero();
  B_tilde.bottomRows(u_size).setIdentity();

  // Create weight matrices.
  const Eigen::MatrixXd Q_tilde = eigen::concat(state_weight, input_weight, Eigen::Vertical).asDiagonal();
  const Eigen::MatrixXd R_tilde = input_rate_weight.asDiagonal();

  // Solve CARE.
  const auto P_inf = care_ArimotoPotter(A_tilde, B_tilde, Q_tilde, R_tilde);
  if (!P_inf) {
    return std::unexpected("Failed to solve CARE: " + P_inf.error());
  }

  // Compute the LQR solution.
  K_ = R_tilde.diagonal().cwiseInverse().asDiagonal() * B_tilde.transpose() * *P_inf;

  return {};
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

  assert(eigen::isFinite(state_scale));
  assert(eigen::isFinite(input_scale));
  assert(eigen::isFinite(state_weight));
  assert(eigen::isFinite(input_weight));
  assert(eigen::isFinite(input_rate_weight));

  assert((state_scale.array() > 0.0).all());
  assert((input_scale.array() > 0.0).all());
  assert((state_weight.array() >= 0.0).all());
  assert((input_weight.array() >= 0.0).all());
  assert((input_rate_weight.array() > 0.0).all());
}

std::ostream& operator<<(std::ostream& _os, const LQD& _arg)
{
  _os << "Dynamics:\n" << _arg.dynamics << std::endl;
  _os << "Current state:\n" << _arg.current_state << std::endl;
  _os << "Target state:\n" << _arg.target_state << std::endl;
  _os << "State error:\n" << _arg.target_state - _arg.current_state << std::endl;
  _os << "Gain:\n" << _arg.K_ << std::endl;

  return _os;
}
}  // namespace ctrl
}  // namespace tobas
