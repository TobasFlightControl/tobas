// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas::ctrl
{
/**
 * Bilinear transform.
 * https://www.dropbox.com/s/ijfnlkvcep1w0f2/%E5%A7%BF%E5%8B%A2%E6%8E%A8%E5%AE%9A%E3%81%AE%E5%9F%BA%E7%A4%8E.pdf
 */
class C2D_Tustin : BaseC2D
{
public:
  explicit C2D_Tustin(Eigen::Index _x_size, Eigen::Index _u_size);
  explicit C2D_Tustin();

  LinearDynamics convert(const LinearDynamics& _cont, double _dt) override;

  void resize(Eigen::Index _x_size, Eigen::Index _u_size);

private:
  Eigen::Index x_size_, u_size_;
  Eigen::MatrixXd I_;  ///< Identity matrix
};
}  // namespace tobas::ctrl
