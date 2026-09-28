// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/c2d/tustin.hpp"

#include <eigen3/Eigen/LU>  // Required for `inverse()`.

namespace tobas
{
namespace ctrl
{
C2D_Tustin::C2D_Tustin(const Eigen::Index& x_size, const Eigen::Index& u_size)
{
  resize(x_size, u_size);
}

C2D_Tustin::C2D_Tustin()
{
}

LinearDynamics C2D_Tustin::convert(const LinearDynamics& cont, const double& dt)
{
  assert(cont.stateSize() == x_size_ && cont.inputSize() == u_size_);
  assert(cont.isFinite());
  assert(dt > 0);

  const Eigen::MatrixXd dt2A = (dt / 2) * cont.A;
  const Eigen::MatrixXd I_plus_dt2A = I_ + dt2A;
  const Eigen::MatrixXd I_minus_dt2A = I_ - dt2A;
  const Eigen::PartialPivLU<Eigen::MatrixXd> lu(I_minus_dt2A);

  LinearDynamics res;
  res.A = lu.solve(I_plus_dt2A);
  res.B = lu.solve(cont.B * dt);

  return res;
}

void C2D_Tustin::resize(const Eigen::Index& x_size, const Eigen::Index& u_size)
{
  x_size_ = x_size;
  u_size_ = u_size;
  I_ = Eigen::MatrixXd::Identity(x_size, x_size);
}
}  // namespace ctrl
}  // namespace tobas
