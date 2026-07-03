// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./state_spaces.hpp"

namespace tobas
{
namespace ctrl
{
/* Infinite-horizon optimal regulator. */
class LQR
{
public:
  LinearDynamics dynamics;  // xd = Ax + Bu: continuous-time state equation

  Eigen::VectorXd state_scale;  // State variable scale
  Eigen::VectorXd input_scale;  // Control input scale

  Eigen::VectorXd state_weight;  // Q: weight for state variables (dimensionless)
  Eigen::VectorXd input_weight;  // R: weight for control inputs (dimensionless)

  Eigen::VectorXd current_state;  // x: current state
  Eigen::VectorXd target_state;   // s: setpoint

  explicit LQR();

  Eigen::VectorXd solve(const bool& update_gain = true);
  void resize(const Eigen::Index& state_size, const Eigen::Index& input_size);
  void updateGain();

  friend std::ostream& operator<<(std::ostream& os, const LQR& arg);

private:
  Eigen::MatrixXd P_inf_;
  Eigen::MatrixXd K_;

  void checkProblemValidity();
};
}  // namespace ctrl
}  // namespace tobas
