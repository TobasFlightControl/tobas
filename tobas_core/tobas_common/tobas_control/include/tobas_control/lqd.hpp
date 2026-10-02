// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./state_spaces.hpp"

namespace tobas
{
namespace ctrl
{
/** Linear quadratic derivative control. (memo: 2-22) */
class LQD
{
public:
  LinearDynamics dynamics;  ///< xd = Ax + Bu: continuous-time state equation

  Eigen::VectorXd state_scale;  ///< State variable scale
  Eigen::VectorXd input_scale;  ///< Control input scale

  Eigen::VectorXd state_weight;       ///< Q: weight for state variables (dimensionless)
  Eigen::VectorXd input_weight;       ///< R: weight for control inputs (dimensionless)
  Eigen::VectorXd input_rate_weight;  ///< S: weight for control input rates (dimensionless)

  Eigen::VectorXd current_state;  ///< x: current state
  Eigen::VectorXd target_state;   ///< s: setpoint
  Eigen::VectorXd last_input;     ///< u: latest control input

  explicit LQD();

  std::expected<Eigen::VectorXd, std::string> solve(double dt, bool update_gain = true);
  void resize(Eigen::Index state_size, Eigen::Index input_size);
  std::expected<void, std::string> updateGainMatrix();

  friend std::ostream& operator<<(std::ostream& os, const LQD& arg);

private:
  Eigen::MatrixXd K_;

  void checkProblemValidity();
};
}  // namespace ctrl
}  // namespace tobas
