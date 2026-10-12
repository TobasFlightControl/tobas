// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_quadprog/utils.hpp"

#include <tobas_eigen_tools/core.hpp>

namespace tobas
{
namespace quadprog
{
void matIneqFromRange(
  const Eigen::VectorXd& _lb,
  const Eigen::VectorXd& _ub,
  Eigen::MatrixXd& _a,
  Eigen::VectorXd& _b,
  double _inf)
{
  assert(_lb.rows() == _ub.rows());
  assert(((_ub - _lb).array() >= 0.0).all());

  const auto size = _lb.rows();

  const Eigen::MatrixXd E = Eigen::MatrixXd::Identity(size, size);
  const auto left = eigen::concat(-E, E, Eigen::Vertical);
  const auto right = eigen::concat(-_lb, _ub, Eigen::Vertical);
  const auto is_valid = (right.array().abs() < _inf).eval();
  const auto num_valid = is_valid.count();

  _a.conservativeResize(num_valid, size);
  _b.conservativeResize(num_valid);

  int row = 0;  // Matrix inequality row index.
  for (int i = 0; i < size * 2; ++i) {
    if (!is_valid(i)) {
      continue;
    }
    _a.block(row, 0, 1, size) = left.row(i);
    _b(row) = right(i);
    ++row;
  }
}
}  // namespace quadprog
}  // namespace tobas
