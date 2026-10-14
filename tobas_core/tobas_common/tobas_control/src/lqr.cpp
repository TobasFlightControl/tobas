// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/lqr.hpp"

#include "tobas_control/care.hpp"

namespace tobas
{
namespace ctrl
{
LQR::LQR()
{
}

std::expected<Eigen::VectorXd, std::string> LQR::solve(bool _update_gain) noexcept
{
  checkProblemValidity();

  if (_update_gain) {
    if (const auto result = updateGainMatrix(); !result) {
      return std::unexpected("Failed to update gain matrix: " + result.error());
    }
  }

  // Scaling.
  const Eigen::VectorXd x_scaled = current_state.array() / state_scale.array();
  const Eigen::VectorXd s_scaled = target_state.array() / state_scale.array();

  const auto u_scaled = K_ * (s_scaled - x_scaled);
  return u_scaled.cwiseProduct(input_scale).eval();
}

void LQR::resize(Eigen::Index _state_size, Eigen::Index _input_size)
{
  dynamics.resize(_state_size, _input_size);

  state_scale.conservativeResize(_state_size);
  input_scale.conservativeResize(_input_size);

  state_weight.conservativeResize(_state_size);
  input_weight.conservativeResize(_input_size);

  current_state.conservativeResize(_state_size);
  target_state.conservativeResize(_state_size);
}

std::expected<void, std::string> LQR::updateGainMatrix() noexcept
{
  const auto dyn_scaled = dynamics.scale(state_scale, input_scale);
  const auto Q = state_weight.asDiagonal();
  const auto R = input_weight.asDiagonal();

  const auto P_inf = care_ArimotoPotter(dyn_scaled.A, dyn_scaled.B, Q, R);
  if (!P_inf) {
    return std::unexpected("Failed to solve CARE: " + P_inf.error());
  }

  K_ = R.inverse() * dyn_scaled.B.transpose() * *P_inf;

  return {};
}

void LQR::checkProblemValidity()
{
  [[maybe_unused]] const auto x_size = current_state.rows();
  [[maybe_unused]] const auto u_size = input_weight.rows();

  assert(x_size > 0);
  assert(u_size > 0);

  assert(dynamics.stateSize() == x_size && dynamics.inputSize() == u_size);
  assert(dynamics.isFinite());

  assert(state_scale.rows() == x_size);
  assert((state_scale.array() > 0.0).all());
  assert(input_scale.rows() == u_size);
  assert((input_scale.array() > 0.0).all());

  assert(state_weight.rows() == x_size);
  assert((state_weight.array() >= 0.0).all());
  assert(input_weight.rows() == u_size);
  assert((input_weight.array() > 0.0).all());
}

std::ostream& operator<<(std::ostream& _os, const LQR& _arg)
{
  _os << "Dynamics:\n" << _arg.dynamics << '\n';
  _os << "Current state:\n" << _arg.current_state << '\n';
  _os << "Target state:\n" << _arg.target_state << '\n';
  _os << "State error:\n" << _arg.target_state - _arg.current_state << '\n';
  _os << "Gain:\n" << _arg.K_ << '\n';

  return _os;
}
}  // namespace ctrl
}  // namespace tobas
