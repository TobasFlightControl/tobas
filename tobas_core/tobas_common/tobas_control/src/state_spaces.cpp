// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/state_spaces.hpp"

namespace tobas
{
namespace ctrl
{
LinearDynamics LinearDynamics::scale(const Eigen::VectorXd& _x_scale, const Eigen::VectorXd& _u_scale) const
{
  assert(_x_scale.rows() == stateSize());
  assert(_u_scale.rows() == inputSize());
  assert(eigen::isFinite(_x_scale));
  assert(eigen::isFinite(_u_scale));
  assert((_x_scale.array() > 0).all());
  assert((_u_scale.array() > 0).all());

  auto res = *this;

  // memo: 2-20
  for (Eigen::Index c = 0; c < stateSize(); ++c) {
    res.A.col(c) *= _x_scale(c);
  }
  for (Eigen::Index r = 0; r < stateSize(); ++r) {
    res.A.row(r) /= _x_scale(r);
  }
  for (Eigen::Index c = 0; c < inputSize(); ++c) {
    res.B.col(c) *= _u_scale(c);
  }
  for (Eigen::Index r = 0; r < stateSize(); ++r) {
    res.B.row(r) /= _x_scale(r);
  }

  return res;
}

std::ostream& operator<<(std::ostream& _os, const LinearDynamics& _arg)
{
  _os << "A:" << std::endl;
  _os << _arg.A << std::endl;
  _os << "B:" << std::endl;
  _os << _arg.B << std::endl;

  return _os;
}

std::ostream& operator<<(std::ostream& _os, const LinearStateSpace& _arg)
{
  _os << "A:" << std::endl;
  _os << _arg.A << std::endl;
  _os << "B:" << std::endl;
  _os << _arg.B << std::endl;
  _os << "C:" << std::endl;
  _os << _arg.C << std::endl;

  return _os;
}
}  // namespace ctrl
}  // namespace tobas
