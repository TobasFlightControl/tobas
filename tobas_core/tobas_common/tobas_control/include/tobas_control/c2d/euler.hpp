// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas::ctrl
{
class C2D_Euler : BaseC2D
{
public:
  explicit C2D_Euler(Eigen::Index _x_size, Eigen::Index _u_size);

  LinearDynamics convert(const LinearDynamics& _cont, double _dt) override;

private:
  const Eigen::Index x_size_, u_size_;
  const Eigen::MatrixXd I_;  ///< Identity matrix
};
}  // namespace tobas::ctrl
