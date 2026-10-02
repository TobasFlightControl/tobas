// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/linear_mpc.hpp"

#include <tobas_eigen_tools/core.hpp>

namespace tobas
{
namespace ctrl
{
LinearMPC::LinearMPC()
{
}

std::expected<Eigen::VectorXd, std::string> LinearMPC::solve()
{
  // Initialization.
  if (is_first_solve_) {
    x_size_ = state_scale.rows();
    u_size_ = input_scale.rows();
    z_size_ = control_scale.rows();
    last_input_ = Eigen::VectorXd::Zero(input_scale.rows());
    is_first_solve_ = false;
  }

  checkProblemValidity();

  // Scale the output matrix.
  Eigen::MatrixXd Cz_scaled = Cz;
  for (Eigen::Index c = 0; c < x_size_; ++c) {
    Cz_scaled.col(c) *= state_scale(c);
  }
  for (Eigen::Index r = 0; r < z_size_; ++r) {
    Cz_scaled.row(r) /= control_scale(r);
  }

  // Scale dynamics and constraints.
  std::vector<LinearDynamics> dyns_scaled;
  std::vector<LinearEquation> du_eqs_scaled, u_eqs_scaled, z_eqs_scaled;
  std::vector<LinearEquation> du_ineqs_scaled, u_ineqs_scaled, z_ineqs_scaled;
  for (Eigen::Index k = 0; k < prediction_steps; ++k) {
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
  const Eigen::VectorXd x_scaled = current_state.array() / state_scale.array();
  const Eigen::VectorXd s_scaled = set_state.array() / control_scale.array();
  const Eigen::VectorXd last_u_scaled = last_input_.array() / input_scale.array();

  // Weight matrices.
  const Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q =
    eigen::tile(control_weight, prediction_steps, Eigen::Vertical).asDiagonal();
  const Eigen::MatrixXd R = eigen::tile(input_rate_weight, input_steps, Eigen::Vertical).asDiagonal().toDenseMatrix();
  const Eigen::MatrixXd Sa = makeSa();
  const Eigen::MatrixXd Sb = makeSb(last_u_scaled);

  // (2.67)
  const Eigen::MatrixXd Psi = makePsi(dyns_scaled, Cz_scaled);
  const Eigen::MatrixXd Upsilon = makeUpsilon(dyns_scaled, Cz_scaled);
  const Eigen::MatrixXd Theta = makeTheta(dyns_scaled, Cz_scaled);

  // Precompute repeated calculations to reduce computation cost.
  const Eigen::VectorXd Psi_x = Psi * x_scaled;
  const Eigen::VectorXd Upsilon_u = Upsilon * last_u_scaled;
  const Eigen::MatrixXd Theta_Q = Theta.transpose() * Q;

  const Eigen::VectorXd Tau = makeTau(x_scaled, s_scaled, Cz_scaled);
  const Eigen::VectorXd Epsilon = Tau - Psi_x - Upsilon_u;  // (3.6)

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
  qpsolver_.x_scale.fill(1.0 / static_cast<double>(input_steps));

  // Solve the QP.
  const auto dU = qpsolver_.solve();
  if (!dU) {
    return dU;
  }

  // Update the latest control input.
  const auto du_scaled = dU->head(u_size_);
  last_input_ += du_scaled.cwiseProduct(input_scale);

  return last_input_;
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
  assert(static_cast<Eigen::Index>(discrete_dynamics.size()) == prediction_steps);
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
  assert(static_cast<Eigen::Index>(input_rate_eqs.size()) == prediction_steps);
  assert(static_cast<Eigen::Index>(input_eqs.size()) == prediction_steps);
  assert(static_cast<Eigen::Index>(control_eqs.size()) == prediction_steps);
  assert(static_cast<Eigen::Index>(input_rate_ineqs.size()) == prediction_steps);
  assert(static_cast<Eigen::Index>(input_ineqs.size()) == prediction_steps);
  assert(static_cast<Eigen::Index>(control_ineqs.size()) == prediction_steps);

  for (Eigen::Index k = 0; k < prediction_steps; ++k) {
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
  const Eigen::VectorXd& last_u,
  const Eigen::VectorXd& Psi_x,
  const Eigen::VectorXd& Upsilon_u,
  const Eigen::MatrixXd& Theta,
  const std::vector<LinearEquation>& du_consts,
  const std::vector<LinearEquation>& u_consts,
  const std::vector<LinearEquation>& z_consts,
  Eigen::MatrixXd& A,
  Eigen::VectorXd& b)
{
  // p.53
  const Eigen::MatrixXd E = makeConstraintMatrix(du_consts, input_steps);
  const Eigen::MatrixXd F = makeConstraintMatrix(u_consts, input_steps);
  const Eigen::MatrixXd G = makeConstraintMatrix(z_consts, prediction_steps);

  // p.100
  const Eigen::MatrixXd W = E.leftCols(E.cols() - 1);
  const Eigen::VectorXd w = -E.col(E.cols() - 1);

  // p.99
  const Eigen::MatrixXd F_gothic = makeFGothic(F);
  const Eigen::MatrixXd F_1 = F_gothic.leftCols(u_size_);
  const Eigen::VectorXd f = F.col(F.cols() - 1);

  // p.100
  const Eigen::MatrixXd Gamma = G.leftCols(G.cols() - 1);
  const Eigen::VectorXd g = G.col(G.cols() - 1);

  // (3.41)
  A = eigen::concat(F_gothic, Gamma * Theta, W, Eigen::Vertical);
  b = eigen::concat(-F_1 * last_u - f, -Gamma * Psi_x - Gamma * Upsilon_u - g, w, Eigen::Vertical);
}

Eigen::MatrixXd LinearMPC::makeSa()
{
  const Eigen::MatrixXd S_diag = input_weight.asDiagonal();

  // Compute the cumulative sum of S. This is left over from the old implementation.
  std::vector<Eigen::MatrixXd> S_cumsum(input_steps + 1);
  S_cumsum[0] = Eigen::MatrixXd::Zero(u_size_, u_size_);
  for (Eigen::Index i = 0; i < input_steps; ++i) {
    S_cumsum[i + 1] = S_cumsum[i] + S_diag;
  }

  // Fill the blocks.
  Eigen::MatrixXd Sa(u_size_ * input_steps, u_size_ * input_steps);
  for (Eigen::Index i = 0; i < input_steps; ++i) {
    for (Eigen::Index j = 0; j < input_steps; ++j) {
      Sa.block(u_size_ * i, u_size_ * j, u_size_, u_size_) = S_cumsum[input_steps] - S_cumsum[std::max(i, j)];
    }
  }

  return Sa;
}

Eigen::VectorXd LinearMPC::makeSb(const Eigen::VectorXd& last_u_scaled)
{
  // Exercise 3-5.
  const Eigen::VectorXd Sb_elem = input_weight.cwiseProduct(last_u_scaled);

  // Simplified using the facts that S is constant over the prediction horizon and u_ref is zero.
  Eigen::VectorXd Sb(u_size_ * input_steps);
  for (Eigen::Index i = 0; i < input_steps; ++i) {
    Sb.segment(u_size_ * i, u_size_) = (input_steps - i) * Sb_elem;
  }

  return Sb;
}

Eigen::MatrixXd LinearMPC::makeFGothic(const Eigen::MatrixXd& F)
{
  const auto n_cond_u = F.rows();  // Number of conditions in (3.35)

  // Compute cumulative sums of F elements.
  std::vector<Eigen::MatrixXd> F_cumsum(input_steps + 1);
  F_cumsum[0] = Eigen::MatrixXd::Zero(n_cond_u, u_size_);
  for (Eigen::Index i = 0; i < input_steps; ++i) {
    F_cumsum[i + 1] = F_cumsum[i] + F.block(0, u_size_ * i, n_cond_u, u_size_);
  }

  // Create F_gothic.
  Eigen::MatrixXd F_gothic(n_cond_u, u_size_ * input_steps);
  for (Eigen::Index i = 0; i < input_steps; ++i) {
    F_gothic.block(0, u_size_ * i, n_cond_u, u_size_) = F_cumsum[input_steps] - F_cumsum[i];
  }

  return F_gothic;
}

Eigen::MatrixXd LinearMPC::makePsi(const std::vector<LinearDynamics>& dyns_scaled, const Eigen::MatrixXd& Cz_scaled)
{
  Eigen::MatrixXd Psi(z_size_ * prediction_steps, x_size_);
  Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(x_size_, x_size_);
  for (Eigen::Index i = 0; i < prediction_steps; ++i) {
    tmp = dyns_scaled[i].A * tmp;
    Psi.block(z_size_ * i, 0, z_size_, x_size_) = Cz_scaled * tmp;
  }

  return Psi;
}

Eigen::MatrixXd LinearMPC::makeUpsilon(const std::vector<LinearDynamics>& dyns_scaled, const Eigen::MatrixXd& Cz_scaled)
{
  Eigen::MatrixXd Upsilon(z_size_ * prediction_steps, u_size_);
  Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(x_size_, u_size_);
  for (Eigen::Index i = 0; i < prediction_steps; ++i) {
    tmp = dyns_scaled[i].A * tmp + dyns_scaled[i].B;
    Upsilon.block(z_size_ * i, 0, z_size_, u_size_) = Cz_scaled * tmp;
  }

  return Upsilon;
}

Eigen::MatrixXd LinearMPC::makeTheta(const std::vector<LinearDynamics>& dyns_scaled, const Eigen::MatrixXd& Cz_scaled)
{
  Eigen::MatrixXd Theta(z_size_ * prediction_steps, u_size_ * input_steps);
  std::vector<Eigen::MatrixXd> tmp;
  for (Eigen::Index i = 0; i < prediction_steps; ++i) {
    tmp.push_back(Eigen::MatrixXd::Zero(x_size_, u_size_));
    const auto max_j = std::min(input_steps, i + 1);
    for (Eigen::Index j = 0; j < max_j; ++j) {
      tmp[j] = dyns_scaled[i].A * tmp[j] + dyns_scaled[i].B;
      Theta.block(z_size_ * i, u_size_ * j, z_size_, u_size_) = Cz_scaled * tmp[j];
    }
    for (Eigen::Index j = max_j; j < input_steps; ++j) {
      Theta.block(z_size_ * i, u_size_ * j, z_size_, u_size_).setZero();
    }
  }

  return Theta;
}

Eigen::VectorXd
LinearMPC::makeTau(const Eigen::VectorXd& x_scaled, const Eigen::VectorXd& s_scaled, const Eigen::MatrixXd& Cz_scaled)
{
  const Eigen::VectorXd error = s_scaled - Cz_scaled * x_scaled;
  const auto decays = makeDecays();

  Eigen::VectorXd Tau(z_size_ * prediction_steps);
  for (Eigen::Index i = 0; i < prediction_steps; ++i) {
    Tau.segment(z_size_ * i, z_size_) = s_scaled - decays[i].cwiseProduct(error);
  }

  return Tau;
}

std::vector<Eigen::VectorXd> LinearMPC::makeDecays()
{
  std::vector<Eigen::VectorXd> decays(prediction_steps, Eigen::VectorXd(z_size_));

  for (Eigen::Index i = 0; i < prediction_steps; ++i) {
    const auto coin_time = time_step * static_cast<double>(i + 1);
    for (Eigen::Index j = 0; j < z_size_; ++j) {
      const auto& T_ref = decay_time_consts(j);
      decays[i](j) = T_ref > 0 ? std::exp(-coin_time / T_ref) : 0;
    }
  }

  return decays;
}

Eigen::MatrixXd LinearMPC::makeConstraintMatrix(const std::vector<LinearEquation>& consts, Eigen::Index H)
{
  const auto const_size = consts[0].equationSize();
  const auto var_size = consts[0].variableSize();

  Eigen::MatrixXd res(const_size * H, var_size * H + 1);
  res.setZero();

  for (Eigen::Index k = 0; k < H; ++k) {
    res.block(const_size * k, var_size * k, const_size, var_size) = consts[k].A;
    res.block(const_size * k, var_size * H, const_size, 1) = -consts[k].b;
  }

  return res;
}
}  // namespace ctrl
}  // namespace tobas
