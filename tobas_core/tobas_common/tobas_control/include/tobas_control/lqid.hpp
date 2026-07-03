// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./state_spaces.hpp"

namespace tobas
{
namespace ctrl
{
/**
 * @brief Linear quadratic integral control. (Introduction to Drone Engineering: p.213)
 * Uses the control input rate of the given dynamics as the input.
 */
class LQID
{
public:
  LinearDynamics dynamics;  // xd = Ax + Bu: continuous-time state equation
  Eigen::MatrixXd C;        // y = C x: matrix that extracts variables to integrate

  Eigen::VectorXd state_weight;             // Q: weight for state variables
  Eigen::VectorXd integrated_error_weight;  // Qi: weight for integrated errors
  Eigen::VectorXd input_weight;             // R: weight for control inputs
  Eigen::VectorXd input_rate_weight;        // S: weight for control input rates

  Eigen::VectorXd current_state;  // x: current state
  Eigen::VectorXd target_state;   // s: setpoint

  // Maximum integrated error.
  // A limit is theoretically unnecessary when assuming a steady disturbance.
  // Set a safe value if the disturbance may suddenly be removed.
  Eigen::VectorXd max_integrated_error;

  explicit LQID(const Eigen::Index& state_size, const Eigen::Index& input_size, const Eigen::Index& integrate_size);

  Eigen::VectorXd solve(const double& dt, const bool& update_gain = true);
  void updateGain();

  inline const Eigen::VectorXd& getIntegralError() const;

  friend std::ostream& operator<<(std::ostream& os, const LQID& arg);

private:
  const Eigen::Index x_size_;        // State size
  const Eigen::Index u_size_;        // Control input size
  const Eigen::Index r_size_;        // Size of variables to integrate
  const Eigen::Index x_tilde_size_;  // Augmented state size
  const Eigen::Index x_idx_;
  const Eigen::Index u_idx_;
  const Eigen::Index eps_idx_;

  Eigen::VectorXd last_u_;   // Latest control input
  Eigen::VectorXd eps_;      // Integrated error
  Eigen::VectorXd x_tilde_;  // Augmented state
  Eigen::VectorXd s_tilde_;  // Augmented target state
  Eigen::MatrixXd A_tilde_;
  Eigen::MatrixXd B_tilde_;
  Eigen::MatrixXd Q_tilde_;
  Eigen::MatrixXd R_tilde_;

  Eigen::MatrixXd P_inf_;
  Eigen::MatrixXd K_;
};

inline const Eigen::VectorXd& LQID::getIntegralError() const
{
  return eps_;
}
}  // namespace ctrl
}  // namespace tobas
