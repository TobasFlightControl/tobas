// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/c2d/tustin.hpp"

#include <eigen3/Eigen/LU>  // Required for `inverse()`.

namespace tobas::ctrl
{
C2D_Tustin::C2D_Tustin(Eigen::Index _x_size, Eigen::Index _u_size)
{
  resize(_x_size, _u_size);
}

C2D_Tustin::C2D_Tustin()
{
}

LinearDynamics C2D_Tustin::convert(const LinearDynamics& _cont, double _dt)
{
  assert(_cont.stateSize() == x_size_ && _cont.inputSize() == u_size_);
  assert(_cont.isFinite());
  assert(_dt > 0);

  const Eigen::MatrixXd dt2A = (_dt / 2) * _cont.A;
  const Eigen::MatrixXd I_plus_dt2A = I_ + dt2A;
  const Eigen::MatrixXd I_minus_dt2A = I_ - dt2A;
  const Eigen::PartialPivLU<Eigen::MatrixXd> lu(I_minus_dt2A);

  LinearDynamics res;
  res.A = lu.solve(I_plus_dt2A);
  res.B = lu.solve(_cont.B * _dt);

  return res;
}

void C2D_Tustin::resize(Eigen::Index _x_size, Eigen::Index _u_size)
{
  x_size_ = _x_size;
  u_size_ = _u_size;
  I_ = Eigen::MatrixXd::Identity(_x_size, _x_size);
}
}  // namespace tobas::ctrl
