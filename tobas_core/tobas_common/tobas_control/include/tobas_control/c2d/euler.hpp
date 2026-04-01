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
  explicit C2D_Euler(const Eigen::Index& x_size, const Eigen::Index& u_size);

  LinearDynamics convert(const LinearDynamics& cont, const double& dt) override;

private:
  const Eigen::Index x_size_, u_size_;
  const Eigen::MatrixXd I_;  // 単位行列
};
}  // namespace ctrl
}  // namespace tobas
