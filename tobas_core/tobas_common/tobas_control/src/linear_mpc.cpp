// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/linear_mpc.hpp"

#include <iostream>

#include <tobas_eigen_tools/core.hpp>

using namespace Eigen;

namespace tobas
{
namespace ctrl
{
LinearMPC::LinearMPC()
{
}

bool LinearMPC::solve()
{
  // Initialization.
  if (is_first_solve_) {
    x_size_ = state_scale.rows();
    u_size_ = input_scale.rows();
    z_size_ = control_scale.rows();
    last_input_ = VectorXd::Zero(input_scale.rows());
    is_first_solve_ = false;
  }

  checkProblemValidity();

  // Scale the output matrix.
  MatrixXd Cz_scaled = Cz;
  for (Index c = 0; c < x_size_; ++c) {
    Cz_scaled.col(c) *= state_scale(c);
  }
  for (Index r = 0; r < z_size_; ++r) {
    Cz_scaled.row(r) /= control_scale(r);
  }

  // Scale dynamics and constraints.
  std::vector<LinearDynamics> dyns_scaled;
  std::vector<LinearEquation> du_eqs_scaled, u_eqs_scaled, z_eqs_scaled;
  std::vector<LinearEquation> du_ineqs_scaled, u_ineqs_scaled, z_ineqs_scaled;
  for (Index k = 0; k < prediction_steps; ++k) {
    dyns_scaled.emplace_back(discrete_dynamics[k].scale(state_scale, input_scale));

    const auto du_eq = input_rate_eqs[k].discretise(time_step);
    du_eqs_scaled.emplace_back(du_eq.scale(input_scale));
    u_eqs_scaled.emplace_back(input_eqs[k].scale(input_scale));
    z_eqs_scaled.emplace_back(control_eqs[k].scale(control_scale));

    const auto du_ineq = input_rate_ineqs[k].discretise(time_step);
    du_ineqs_scaled.emplace_back(du_ineq.scale(input_scale));
    u_ineqs_scaled.emplace_back(input_ineqs[k].scale(input_scale));
    z_ineqs_scaled.emplace_back(control_ineqs[k].scale(control_scale));
  }

  // Scale the state vectors and related values.
  const VectorXd x_scaled = current_state.array() / state_scale.array();
  const VectorXd s_scaled = set_state.array() / control_scale.array();
  const VectorXd last_u_scaled = last_input_.array() / input_scale.array();

  // Weight matrices.
  const DiagonalMatrix<double, Dynamic> Q = eigen::tile(control_weight, prediction_steps, 0).asDiagonal();
  const MatrixXd R = eigen::tile(input_rate_weight, input_steps, 0).asDiagonal().toDenseMatrix();
  const MatrixXd Sa = makeSa();
  const MatrixXd Sb = makeSb(last_u_scaled);

  // (2.67)
  const MatrixXd Psi = makePsi(dyns_scaled, Cz_scaled);
  const MatrixXd Upsilon = makeUpsilon(dyns_scaled, Cz_scaled);
  const MatrixXd Theta = makeTheta(dyns_scaled, Cz_scaled);

  // Precompute repeated calculations to reduce computation cost.
  const VectorXd Psi_x = Psi * x_scaled;
  const VectorXd Upsilon_u = Upsilon * last_u_scaled;
  const MatrixXd Theta_Q = Theta.transpose() * Q;

  const VectorXd Tau = makeTau(x_scaled, s_scaled, Cz_scaled);
  const VectorXd Epsilon = Tau - Psi_x - Upsilon_u;  // (3.6)

  // Objective function.
  qpsolver_.problem.P = Theta_Q * Theta + R + Sa;  // Phi (= Eta)
  qpsolver_.problem.q = Sb - Theta_Q * Epsilon;    // phi: (3.11), (3.43)

  // Equality constraints.
  updateQpConstraint(
    last_u_scaled,
    Psi_x,
    Upsilon_u,
    Theta,
    du_eqs_scaled,
    u_eqs_scaled,
    z_eqs_scaled,
    qpsolver_.problem.G,
    qpsolver_.problem.h);

  // Inequality constraints.
  updateQpConstraint(
    last_u_scaled,
    Psi_x,
    Upsilon_u,
    Theta,
    du_ineqs_scaled,
    u_ineqs_scaled,
    z_ineqs_scaled,
    qpsolver_.problem.A,
    qpsolver_.problem.b);

  // Decision variable scale.
  // Prioritize accuracy when the rate is small,
  // and assume the control input rate changes from the maximum to the minimum value over the input horizon.
  // Since the control input is scaled to 1, the control input increment scale is (1/Tu)*dt = 1/Hu.
  qpsolver_.x_scale.conservativeResize(u_size_ * input_steps);
  qpsolver_.x_scale.fill(1. / static_cast<double>(input_steps));

  // Solve the QP.
  if (!qpsolver_.solve()) {
    return false;
  }

  // Update the latest control input.
  const auto& dU = qpsolver_.solution();
  const auto du_scaled = dU.head(u_size_);
  last_input_ += du_scaled.cwiseProduct(input_scale);

  return true;
}

std::ostream& operator<<(std::ostream& os, const LinearMPC& arg)
{
  os << "QuadProgSolver:" << std::endl;
  os << arg.qpsolver_ << std::endl;

  return os;
}

void LinearMPC::checkProblemValidity()
{
  // Dynamics
  assert(static_cast<Index>(discrete_dynamics.size()) == prediction_steps);
  for ([[maybe_unused]] const auto& dyn : discrete_dynamics) {
    assert(dyn.stateSize() == x_size_ && dyn.inputSize() == u_size_);
    assert(dyn.isFinite());
    // TODO: Check stabilizability of control variables, not state variables.
  }
  assert(Cz.rows() == z_size_ && Cz.cols() == x_size_);
  assert(eigen::isFinite(Cz));

  assert(1 <= input_steps && input_steps <= prediction_steps);
  assert(time_step > 0);

  // Tracking error decay time constants
  assert(decay_time_consts.size() == z_size_);
  assert(eigen::isFinite(decay_time_consts));
  assert((decay_time_consts.array() >= 0).all());

  // Scales
  assert(state_scale.rows() == x_size_);
  assert(input_scale.rows() == u_size_);
  assert(control_scale.rows() == z_size_);
  assert(eigen::isFinite(state_scale));
  assert(eigen::isFinite(input_scale));
  assert(eigen::isFinite(control_scale));
  assert((state_scale.array() > 0).all());
  assert((input_scale.array() > 0).all());
  assert((control_scale.array() > 0).all());

  // Weights
  assert(input_rate_weight.rows() == u_size_);
  assert(input_weight.rows() == u_size_);
  assert(control_weight.rows() == z_size_);
  assert(eigen::isFinite(input_rate_weight));
  assert(eigen::isFinite(input_weight));
  assert(eigen::isFinite(control_weight));
  assert((input_rate_weight.array() >= 0).all());
  assert((input_weight.array() >= 0).all());
  assert((control_weight.array() >= 0).all());

  // Constraints
  assert(static_cast<Index>(input_rate_eqs.size()) == prediction_steps);
  assert(static_cast<Index>(input_eqs.size()) == prediction_steps);
  assert(static_cast<Index>(control_eqs.size()) == prediction_steps);
  assert(static_cast<Index>(input_rate_ineqs.size()) == prediction_steps);
  assert(static_cast<Index>(input_ineqs.size()) == prediction_steps);
  assert(static_cast<Index>(control_ineqs.size()) == prediction_steps);

  for (Index k = 0; k < prediction_steps; ++k) {
    assert(input_rate_eqs[k].variableSize() == u_size_);
    assert(input_eqs[k].variableSize() == u_size_);
    assert(control_eqs[k].variableSize() == z_size_);
    assert(input_rate_ineqs[k].variableSize() == u_size_);
    assert(input_ineqs[k].variableSize() == u_size_);
    assert(control_ineqs[k].variableSize() == z_size_);

    assert(input_rate_eqs[k].isFinite());
    assert(input_eqs[k].isFinite());
    assert(control_eqs[k].isFinite());
    assert(input_rate_ineqs[k].isFinite());
    assert(input_ineqs[k].isFinite());
    assert(control_ineqs[k].isFinite());
  }

  // States
  assert(current_state.rows() == x_size_);
  assert(set_state.rows() == z_size_);
  assert(eigen::isFinite(current_state));
  assert(eigen::isFinite(set_state));
}

void LinearMPC::updateQpConstraint(
  const VectorXd& last_u,
  const VectorXd& Psi_x,
  const VectorXd& Upsilon_u,
  const MatrixXd& Theta,
  const std::vector<LinearEquation>& du_consts,
  const std::vector<LinearEquation>& u_consts,
  const std::vector<LinearEquation>& z_consts,
  MatrixXd& A,
  VectorXd& b)
{
  // p.53
  const MatrixXd E = makeConstraintMatrix(du_consts, input_steps);
  const MatrixXd F = makeConstraintMatrix(u_consts, input_steps);
  const MatrixXd G = makeConstraintMatrix(z_consts, prediction_steps);

  // p.100
  const MatrixXd W = E.leftCols(E.cols() - 1);
  const VectorXd w = -E.col(E.cols() - 1);

  // p.99
  const MatrixXd F_gothic = makeFGothic(F);
  const MatrixXd F_1 = F_gothic.leftCols(u_size_);
  const VectorXd f = F.col(F.cols() - 1);

  // p.100
  const MatrixXd Gamma = G.leftCols(G.cols() - 1);
  const VectorXd g = G.col(G.cols() - 1);

  // (3.41)
  A = eigen::concat(F_gothic, Gamma * Theta, W, 0);
  b = eigen::concat(-F_1 * last_u - f, -Gamma * Psi_x - Gamma * Upsilon_u - g, w, 0);
}

MatrixXd LinearMPC::makeSa()
{
  const MatrixXd S_diag = input_weight.asDiagonal();

  // Compute the cumulative sum of S. This is left over from the old implementation.
  std::vector<MatrixXd> S_cumsum(input_steps + 1);
  S_cumsum[0] = MatrixXd::Zero(u_size_, u_size_);
  for (Index i = 0; i < input_steps; ++i) {
    S_cumsum[i + 1] = S_cumsum[i] + S_diag;
  }

  // Fill the blocks.
  MatrixXd Sa(u_size_ * input_steps, u_size_ * input_steps);
  for (Index i = 0; i < input_steps; ++i) {
    for (Index j = 0; j < input_steps; ++j) {
      Sa.block(u_size_ * i, u_size_ * j, u_size_, u_size_) = S_cumsum[input_steps] - S_cumsum[std::max(i, j)];
    }
  }

  return Sa;
}

VectorXd LinearMPC::makeSb(const VectorXd& last_u_scaled)
{
  // Exercise 3-5.
  const VectorXd Sb_elem = input_weight.cwiseProduct(last_u_scaled);

  // Simplified using the facts that S is constant over the prediction horizon and u_ref is zero.
  VectorXd Sb(u_size_ * input_steps);
  for (Index i = 0; i < input_steps; ++i) {
    Sb.segment(u_size_ * i, u_size_) = (input_steps - i) * Sb_elem;
  }

  return Sb;
}

MatrixXd LinearMPC::makeFGothic(const MatrixXd& F)
{
  const auto n_cond_u = F.rows();  // Number of conditions in (3.35)

  // Compute cumulative sums of F elements.
  std::vector<MatrixXd> F_cumsum(input_steps + 1);
  F_cumsum[0] = MatrixXd::Zero(n_cond_u, u_size_);
  for (Index i = 0; i < input_steps; ++i) {
    F_cumsum[i + 1] = F_cumsum[i] + F.block(0, u_size_ * i, n_cond_u, u_size_);
  }

  // Create F_gothic.
  MatrixXd F_gothic(n_cond_u, u_size_ * input_steps);
  for (Index i = 0; i < input_steps; ++i) {
    F_gothic.block(0, u_size_ * i, n_cond_u, u_size_) = F_cumsum[input_steps] - F_cumsum[i];
  }

  return F_gothic;
}

MatrixXd LinearMPC::makePsi(const std::vector<LinearDynamics>& dyns_scaled, const MatrixXd& Cz_scaled)
{
  MatrixXd Psi(z_size_ * prediction_steps, x_size_);
  MatrixXd tmp = MatrixXd::Identity(x_size_, x_size_);
  for (Index i = 0; i < prediction_steps; ++i) {
    tmp = dyns_scaled[i].A * tmp;
    Psi.block(z_size_ * i, 0, z_size_, x_size_) = Cz_scaled * tmp;
  }

  return Psi;
}

MatrixXd LinearMPC::makeUpsilon(const std::vector<LinearDynamics>& dyns_scaled, const MatrixXd& Cz_scaled)
{
  MatrixXd Upsilon(z_size_ * prediction_steps, u_size_);
  MatrixXd tmp = MatrixXd::Zero(x_size_, u_size_);
  for (Index i = 0; i < prediction_steps; ++i) {
    tmp = dyns_scaled[i].A * tmp + dyns_scaled[i].B;
    Upsilon.block(z_size_ * i, 0, z_size_, u_size_) = Cz_scaled * tmp;
  }

  return Upsilon;
}

MatrixXd LinearMPC::makeTheta(const std::vector<LinearDynamics>& dyns_scaled, const MatrixXd& Cz_scaled)
{
  MatrixXd Theta(z_size_ * prediction_steps, u_size_ * input_steps);
  std::vector<MatrixXd> tmp;
  for (Index i = 0; i < prediction_steps; ++i) {
    tmp.push_back(MatrixXd::Zero(x_size_, u_size_));
    const auto max_j = std::min(input_steps, i + 1);
    for (Index j = 0; j < max_j; ++j) {
      tmp[j] = dyns_scaled[i].A * tmp[j] + dyns_scaled[i].B;
      Theta.block(z_size_ * i, u_size_ * j, z_size_, u_size_) = Cz_scaled * tmp[j];
    }
    for (Index j = max_j; j < input_steps; ++j) {
      Theta.block(z_size_ * i, u_size_ * j, z_size_, u_size_).setZero();
    }
  }

  return Theta;
}

VectorXd LinearMPC::makeTau(const VectorXd& x_scaled, const VectorXd& s_scaled, const MatrixXd& Cz_scaled)
{
  const VectorXd error = s_scaled - Cz_scaled * x_scaled;
  const auto decays = makeDecays();

  VectorXd Tau(z_size_ * prediction_steps);
  for (Index i = 0; i < prediction_steps; ++i) {
    Tau.segment(z_size_ * i, z_size_) = s_scaled - decays[i].cwiseProduct(error);
  }

  return Tau;
}

std::vector<VectorXd> LinearMPC::makeDecays()
{
  std::vector<VectorXd> decays(prediction_steps, VectorXd(z_size_));

  for (Index i = 0; i < prediction_steps; ++i) {
    const auto coin_time = time_step * static_cast<double>(i + 1);
    for (Index j = 0; j < z_size_; ++j) {
      const auto& T_ref = decay_time_consts(j);
      decays[i](j) = T_ref > 0 ? std::exp(-coin_time / T_ref) : 0;
    }
  }

  return decays;
}

MatrixXd LinearMPC::makeConstraintMatrix(const std::vector<LinearEquation>& consts, const Index& H)
{
  const auto const_size = consts[0].equationSize();
  const auto var_size = consts[0].variableSize();

  MatrixXd res(const_size * H, var_size * H + 1);
  res.setZero();

  for (Index k = 0; k < H; ++k) {
    res.block(const_size * k, var_size * k, const_size, var_size) = consts[k].A;
    res.block(const_size * k, var_size * H, const_size, 1) = -consts[k].b;
  }

  return res;
}
}  // namespace ctrl
}  // namespace tobas
