// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/equations.hpp"

namespace tobas
{
namespace ctrl
{
LinearEquation LinearEquation::scale(const Eigen::VectorXd& _scale) const
{
  assert(_scale.rows() == variableSize());
  assert((_scale.array() > 0).all());

  auto res = *this;
  for (Eigen::Index c = 0; c < variableSize(); ++c) {
    res.A.col(c) *= _scale(c);
  }

  return res;
}

LinearEquation LinearEquation::discretise(double _dt) const
{
  assert(_dt >= 0);

  auto res = *this;
  res.b *= _dt;  // Simply multiply the right-hand side of the equation by dt.
  return res;
}

std::ostream& operator<<(std::ostream& _os, const LinearEquation& _arg)
{
  _os << "A:" << std::endl;
  _os << _arg.A << std::endl;
  _os << "b:" << std::endl;
  _os << _arg.b << std::endl;

  return _os;
}
}  // namespace ctrl
}  // namespace tobas
