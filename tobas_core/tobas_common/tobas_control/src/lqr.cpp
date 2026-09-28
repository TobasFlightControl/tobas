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

Eigen::VectorXd LQR::solve(const bool& update_gain)
{
  checkProblemValidity();

  if (update_gain) {
    updateGain();
  }

  // Scaling.
  const Eigen::VectorXd x_scaled = current_state.array() / state_scale.array();
  const Eigen::VectorXd s_scaled = target_state.array() / state_scale.array();

  const auto u_scaled = K_ * (s_scaled - x_scaled);
  return u_scaled.cwiseProduct(input_scale);
}

void LQR::resize(const Eigen::Index& state_size, const Eigen::Index& input_size)
{
  dynamics.resize(state_size, input_size);

  state_scale.conservativeResize(state_size);
  input_scale.conservativeResize(input_size);

  state_weight.conservativeResize(state_size);
  input_weight.conservativeResize(input_size);

  current_state.conservativeResize(state_size);
  target_state.conservativeResize(state_size);
}

void LQR::updateGain()
{
  const auto dyn_scaled = dynamics.scale(state_scale, input_scale);
  P_inf_ = care_ArimotoPotter(dyn_scaled.A, dyn_scaled.B, state_weight.asDiagonal(), input_weight.asDiagonal());
  K_ = input_weight.asDiagonal().inverse() * dyn_scaled.B.transpose() * P_inf_;
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

std::ostream& operator<<(std::ostream& os, const LQR& arg)
{
  os << "Dynamics:\n" << arg.dynamics << std::endl;
  os << "Current state:\n" << arg.current_state << std::endl;
  os << "Target state:\n" << arg.target_state << std::endl;
  os << "State error:\n" << arg.target_state - arg.current_state << std::endl;
  os << "Covariance matrix:\n" << arg.P_inf_ << std::endl;
  os << "Gain:\n" << arg.K_ << std::endl;

  return os;
}
}  // namespace ctrl
}  // namespace tobas
