// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/equations.hpp"

namespace tobas
{
namespace ctrl
{
LinearEquation LinearEquation::scale(const Eigen::VectorXd& scale) const
{
  assert(scale.rows() == variableSize());
  assert((scale.array() > 0).all());

  auto res = *this;
  for (Eigen::Index c = 0; c < variableSize(); ++c) {
    res.A.col(c) *= scale(c);
  }

  return res;
}

LinearEquation LinearEquation::discretise(const double& dt) const
{
  assert(dt >= 0);

  auto res = *this;
  res.b *= dt;  // Simply multiply the right-hand side of the equation by dt.
  return res;
}

std::ostream& operator<<(std::ostream& os, const LinearEquation& arg)
{
  os << "A:" << std::endl;
  os << arg.A << std::endl;
  os << "b:" << std::endl;
  os << arg.b << std::endl;

  return os;
}
}  // namespace ctrl
}  // namespace tobas
