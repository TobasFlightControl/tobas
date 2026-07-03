// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

// #include <tobas_quadprog/quadprogpp.hpp>
// #include <tobas_quadprog/qpoases.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
// #include <tobas_quadprog/primal_dual_interior_point.hpp>

#include "./equations.hpp"
#include "./state_spaces.hpp"

namespace tobas
{
namespace ctrl
{
/**
 * @brief Linear model predictive control.
 * cf. https://www.tdupress.jp/book/b349347.html
 */
class LinearMPC
{
public:
  // Dynamics
  std::vector<LinearDynamics> discrete_dynamics;  // x[k+1] = A[k]x[k] + B[k]u[k]: discrete state equation
  Eigen::MatrixXd Cz;                             // z = Cz x: control variable equation

  // Time horizon
  Eigen::Index prediction_steps;  // H_p: number of state prediction steps
  Eigen::Index input_steps;       // H_u: number of control input prediction steps
  double time_step;               // dt: discretization interval [s]

  // tau: decay time constant for the error between the control variable setpoint and reference trajectory [s].
  // When control variable oscillation is large, adjusting the reference trajectory often preserves tracking
  // performance better than increasing the weight on its time derivative.
  // One advantage of MPC is that the reference trajectory can be time-varying.
  Eigen::VectorXd decay_time_consts;

  // Scales
  Eigen::VectorXd state_scale;    // State variable scale
  Eigen::VectorXd input_scale;    // Control input scale
  Eigen::VectorXd control_scale;  // Control variable scale

  // Weights
  Eigen::VectorXd input_rate_weight;  // R: weight for control input rates (dimensionless)
  Eigen::VectorXd input_weight;       // S: weight for control inputs (dimensionless)
  Eigen::VectorXd control_weight;     // Q: weight for control variables (dimensionless)

  // Equality constraints
  std::vector<LinearEquation> input_rate_eqs;  // Ee du <= ee: equality constraints on control input rates
  std::vector<LinearEquation> input_eqs;       // Fe u <= fe: equality constraints on control inputs
  std::vector<LinearEquation> control_eqs;     // Ge z <= ge: equality constraints on control variables

  // Inequality constraints
  std::vector<LinearEquation> input_rate_ineqs;  // Ei du <= ei: inequality constraints on control input rates
  std::vector<LinearEquation> input_ineqs;       // Fi u <= fi: inequality constraints on control inputs
  std::vector<LinearEquation> control_ineqs;     // Gi z <= gi: inequality constraints on control variables

  // States
  Eigen::VectorXd current_state;  // x: current state
  Eigen::VectorXd set_state;      // s: setpoint

  explicit LinearMPC();

  bool solve();

  inline const Eigen::VectorXd& optimalControlInput() const;
  inline const std::string& errorMessage() const;

  friend std::ostream& operator<<(std::ostream& os, const LinearMPC& arg);

private:
  // quadprog::QuadProgppSolver qpsolver_;
  // quadprog::QpOasesSolver qpsolver_;
  quadprog::DualActiveSetSolver qpsolver_;
  // quadprog::PrimalDualInteriorPointSolver qpsolver_;

  bool is_first_solve_ = true;
  Eigen::Index x_size_, u_size_, z_size_;
  Eigen::VectorXd last_input_;  // u: latest control input

  void checkProblemValidity();

  void updateQpConstraint(
    const Eigen::VectorXd& last_u,
    const Eigen::VectorXd& Psi_x,
    const Eigen::VectorXd& Upsilon_u,
    const Eigen::MatrixXd& Theta,
    const std::vector<LinearEquation>& du_consts,
    const std::vector<LinearEquation>& u_consts,
    const std::vector<LinearEquation>& z_consts,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b);

  Eigen::MatrixXd makeSa();
  Eigen::VectorXd makeSb(const Eigen::VectorXd& last_u);
  Eigen::MatrixXd makeFGothic(const Eigen::MatrixXd& F);
  Eigen::MatrixXd makePsi(const std::vector<LinearDynamics>& dyn, const Eigen::MatrixXd& Cz);
  Eigen::MatrixXd makeUpsilon(const std::vector<LinearDynamics>& dyn, const Eigen::MatrixXd& Cz);
  Eigen::MatrixXd makeTheta(const std::vector<LinearDynamics>& dyn, const Eigen::MatrixXd& Cz);

  /* Create Tau on p.90 based on Example 1.3 on p.12. */
  Eigen::VectorXd makeTau(const Eigen::VectorXd& x, const Eigen::VectorXd& z, const Eigen::MatrixXd& Cz);
  std::vector<Eigen::VectorXd> makeDecays();

  /* Create the overall inequality constraint matrix from the time series of inequality conditions A x <= b. */
  static Eigen::MatrixXd makeConstraintMatrix(const std::vector<LinearEquation>& ineqs, const Eigen::Index& H);
};

inline const Eigen::VectorXd& LinearMPC::optimalControlInput() const
{
  return last_input_;
}

inline const std::string& LinearMPC::errorMessage() const
{
  return qpsolver_.errorMessage();
}
}  // namespace ctrl
}  // namespace tobas
