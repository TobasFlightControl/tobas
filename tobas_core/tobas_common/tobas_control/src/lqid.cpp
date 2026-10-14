// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/lqid.hpp"

#include <tobas_eigen_tools/core.hpp>

#include "tobas_control/care.hpp"

namespace tobas
{
namespace ctrl
{
LQID::LQID(Eigen::Index _state_size, Eigen::Index _input_size, Eigen::Index _integrate_size)
  : dynamics(_state_size, _input_size)
  , C(_integrate_size, _state_size)
  , state_weight(_state_size)
  , integrated_error_weight(_integrate_size)
  , input_weight(_input_size)
  , input_rate_weight(_input_size)
  , current_state(_state_size)
  , target_state(_state_size)
  , max_integrated_error(_integrate_size)
  ,

  x_size_(_state_size)
  , u_size_(_input_size)
  , r_size_(_integrate_size)
  , x_tilde_size_(_state_size + _input_size + _integrate_size)
  , x_idx_(0)
  , u_idx_(x_idx_ + x_size_)
  , eps_idx_(u_idx_ + u_size_)
  ,

  last_u_(u_size_)
  , eps_(r_size_)
  , x_tilde_(x_tilde_size_)
  , s_tilde_(x_tilde_size_)
  , A_tilde_(x_tilde_size_, x_tilde_size_)
  , B_tilde_(x_tilde_size_, u_size_)
  , Q_tilde_(x_tilde_size_, x_tilde_size_)
  , R_tilde_(u_size_, u_size_)
{
  C.setZero();
  state_weight.setZero();
  integrated_error_weight.setZero();
  input_weight.setZero();
  input_rate_weight.setZero();
  current_state.setZero();
  target_state.setZero();
  max_integrated_error.setZero();

  last_u_.setZero();
  eps_.setZero();
  x_tilde_.setZero();
  s_tilde_.setZero();
  A_tilde_.setZero();
  B_tilde_.setZero();
  Q_tilde_.setZero();
  R_tilde_.setZero();
}

std::expected<Eigen::VectorXd, std::string> LQID::solve(double _dt, bool _update_gain)
{
  assert(_dt >= 0);

  assert(current_state.rows() == x_size_);
  assert(eigen::isFinite(current_state));

  assert(target_state.rows() == x_size_);
  assert(eigen::isFinite(target_state));

  assert(max_integrated_error.rows() == r_size_);
  assert(eigen::isFinite(max_integrated_error));
  assert((max_integrated_error.array() >= 0).all());

  if (_update_gain) {
    if (const auto result = updateGainMatrix(); !result) {
      return std::unexpected("Failed to update gain matrix: " + result.error());
    }
  }

  // Update the integrated error.
  const Eigen::VectorXd y = C * current_state;
  const Eigen::VectorXd r = C * target_state;
  eps_ += (r - y) * _dt;
  eps_ = eps_.cwiseMax(-max_integrated_error).cwiseMin(max_integrated_error);

  // Create the augmented state.
  x_tilde_ << current_state, last_u_, eps_;
  s_tilde_.block(0, 0, x_size_, 1) = target_state;

  // Compute the optimal control input rate.
  const auto ud_scaled = K_ * (s_tilde_ - x_tilde_);

  // Update the latest control input.
  last_u_ += ud_scaled * _dt;

  return last_u_;
}

std::expected<void, std::string> LQID::updateGainMatrix()
{
  assert(dynamics.stateSize() == x_size_ && dynamics.inputSize() == u_size_);
  assert(dynamics.isFinite());

  assert(C.rows() == r_size_ && C.cols() == x_size_);
  assert(eigen::isFinite(C));

  assert(state_weight.rows() == x_size_);
  assert(eigen::isFinite(state_weight));
  assert((state_weight.array() >= 0).all());

  assert(integrated_error_weight.rows() == r_size_);
  assert(eigen::isFinite(integrated_error_weight));
  assert((integrated_error_weight.array() >= 0).all());

  assert(input_weight.rows() == u_size_);
  assert(eigen::isFinite(input_weight));
  assert((input_weight.array() >= 0).all());

  assert(input_rate_weight.rows() == u_size_);
  assert(eigen::isFinite(input_rate_weight));
  assert((input_rate_weight.array() > 0).all());

  // Update the augmented-state dynamics.
  A_tilde_.block(x_idx_, x_idx_, x_size_, x_size_) = dynamics.A;
  A_tilde_.block(x_idx_, u_idx_, x_size_, u_size_) = dynamics.B;
  A_tilde_.block(eps_idx_, x_idx_, r_size_, x_size_) = -C;
  B_tilde_.block(u_idx_, 0, u_size_, u_size_).diagonal().setOnes();

  // Update the weight matrices.
  Q_tilde_.block(x_idx_, x_idx_, x_size_, x_size_).diagonal() = state_weight;
  Q_tilde_.block(u_idx_, u_idx_, u_size_, u_size_).diagonal() = input_weight;
  Q_tilde_.block(eps_idx_, eps_idx_, r_size_, r_size_).diagonal() = integrated_error_weight;
  R_tilde_.diagonal() = input_rate_weight;

  // Solve CARE.
  const auto P_inf = care_ArimotoPotter(A_tilde_, B_tilde_, Q_tilde_, R_tilde_);
  if (!P_inf) {
    return std::unexpected("Failed to solve CARE: " + P_inf.error());
  }

  // Compute the LQR solution.
  K_ = R_tilde_.diagonal().cwiseInverse().asDiagonal() * B_tilde_.transpose() * *P_inf;

  return {};
}

std::ostream& operator<<(std::ostream& _os, const LQID& _arg)
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
