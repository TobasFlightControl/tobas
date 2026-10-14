// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/c2d/euler.hpp"

namespace tobas::ctrl
{
C2D_Euler::C2D_Euler(Eigen::Index _x_size, Eigen::Index _u_size)
  : x_size_(_x_size), u_size_(_u_size), I_(Eigen::MatrixXd::Identity(_x_size, _x_size))
{
}

LinearDynamics C2D_Euler::convert(const LinearDynamics& _cont, double _dt)
{
  assert(_cont.stateSize() == x_size_ && _cont.inputSize() == u_size_);
  assert(_cont.isFinite());
  assert(_dt > 0);

  return LinearDynamics(I_ + _cont.A * _dt, _cont.B * _dt);
}
}  // namespace tobas::ctrl
