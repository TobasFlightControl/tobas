// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace ctrl
{
class C2D_Euler : BaseC2D
{
public:
  explicit C2D_Euler(Eigen::Index x_size, Eigen::Index u_size);

  LinearDynamics convert(const LinearDynamics& cont, double dt) override;

private:
  const Eigen::Index x_size_, u_size_;
  const Eigen::MatrixXd I_;  ///< Identity matrix
};
}  // namespace ctrl
}  // namespace tobas
