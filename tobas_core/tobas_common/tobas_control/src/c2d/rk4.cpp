// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/c2d/rk4.hpp"

namespace tobas::ctrl
{
C2D_RK4::C2D_RK4(Eigen::Index _x_size, Eigen::Index _u_size)
{
  resize(_x_size, _u_size);
}

C2D_RK4::C2D_RK4()
{
}

LinearDynamics C2D_RK4::convert(const LinearDynamics& _cont, double _dt)
{
  assert(_cont.stateSize() == x_size_ && _cont.inputSize() == u_size_);
  assert(_cont.isFinite());
  assert(_dt >= 0);

  // Compute powers of Ac_dt.
  const Eigen::MatrixXd Ac_dt = _cont.A * _dt;
  for (size_t i = 1; i <= 4; ++i) {
    Ac_dt_pows_[i] = Ac_dt_pows_[i - 1] * Ac_dt;
  }

  // Compute Ad.
  Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(x_size_, x_size_);
  for (size_t i = 1; i <= 4; ++i) {
    Ad += Ac_dt_pows_[i] / factorials_[i];
  }

  // Compute Bd.
  Eigen::MatrixXd Bd = Eigen::MatrixXd::Identity(x_size_, x_size_);
  for (size_t i = 1; i <= 3; ++i) {
    Bd += Ac_dt_pows_[i] / factorials_[i + 1];
  }
  Bd *= (_cont.B * _dt);

  return LinearDynamics(Ad, Bd);
}

void C2D_RK4::resize(Eigen::Index _x_size, Eigen::Index _u_size)
{
  x_size_ = _x_size;
  u_size_ = _u_size;

  Ac_dt_pows_[0] = Eigen::MatrixXd::Identity(x_size_, x_size_);

  factorials_[0] = 1;
  for (size_t i = 1; i <= 4; ++i) {
    factorials_[i] = factorials_[i - 1] * static_cast<double>(i);
  }
}
}  // namespace tobas::ctrl
