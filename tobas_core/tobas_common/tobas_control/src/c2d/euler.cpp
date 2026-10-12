// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/c2d/euler.hpp"

namespace tobas
{
namespace ctrl
{
C2D_Euler::C2D_Euler(Eigen::Index x_size, Eigen::Index u_size)
  : x_size_(x_size), u_size_(u_size), I_(Eigen::MatrixXd::Identity(x_size, x_size))
{
}

LinearDynamics C2D_Euler::convert(const LinearDynamics& cont, double dt)
{
  assert(cont.stateSize() == x_size_ && cont.inputSize() == u_size_);
  assert(cont.isFinite());
  assert(dt > 0);

  return LinearDynamics(I_ + cont.A * dt, cont.B * dt);
}
}  // namespace ctrl
}  // namespace tobas
