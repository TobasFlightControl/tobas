// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/state_spaces.hpp"

namespace tobas
{
namespace ctrl
{
LinearDynamics LinearDynamics::scale(const Eigen::VectorXd& x_scale, const Eigen::VectorXd& u_scale) const
{
  assert(x_scale.rows() == stateSize());
  assert(u_scale.rows() == inputSize());
  assert(eigen::isFinite(x_scale));
  assert(eigen::isFinite(u_scale));
  assert((x_scale.array() > 0).all());
  assert((u_scale.array() > 0).all());

  auto res = *this;

  // memo: 2-20
  for (Eigen::Index c = 0; c < stateSize(); ++c) {
    res.A.col(c) *= x_scale(c);
  }
  for (Eigen::Index r = 0; r < stateSize(); ++r) {
    res.A.row(r) /= x_scale(r);
  }
  for (Eigen::Index c = 0; c < inputSize(); ++c) {
    res.B.col(c) *= u_scale(c);
  }
  for (Eigen::Index r = 0; r < stateSize(); ++r) {
    res.B.row(r) /= x_scale(r);
  }

  return res;
}

std::ostream& operator<<(std::ostream& os, const LinearDynamics& arg)
{
  os << "A:" << std::endl;
  os << arg.A << std::endl;
  os << "B:" << std::endl;
  os << arg.B << std::endl;

  return os;
}

std::ostream& operator<<(std::ostream& os, const LinearStateSpace& arg)
{
  os << "A:" << std::endl;
  os << arg.A << std::endl;
  os << "B:" << std::endl;
  os << arg.B << std::endl;
  os << "C:" << std::endl;
  os << arg.C << std::endl;

  return os;
}
}  // namespace ctrl
}  // namespace tobas
