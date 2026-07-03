// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/core.hpp>

namespace tobas
{
namespace ctrl
{
class LinearEquation
{
public:
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  inline explicit LinearEquation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);
  inline explicit LinearEquation(const Eigen::Index& var_size, const Eigen::Index& eq_size);
  inline explicit LinearEquation();

  static inline LinearEquation Zero(const Eigen::Index& var_size, const Eigen::Index& eq_size);

  inline void resize(const Eigen::Index& var_size, const Eigen::Index& eq_size);
  inline void setZero();

  inline bool isFinite() const;

  /* Variable dimension. */
  inline Eigen::Index variableSize() const;

  /* Equation or inequality dimension. */
  inline Eigen::Index equationSize() const;

  /* Create a matrix equation for scaled variables. */
  LinearEquation scale(const Eigen::VectorXd& scale) const;

  /* Discretize an equation for rates into an equation for increments. */
  LinearEquation discretise(const double& dt) const;

  friend std::ostream& operator<<(std::ostream& os, const LinearEquation& arg);
};

inline LinearEquation::LinearEquation(const Eigen::MatrixXd& _A, const Eigen::MatrixXd& _b) : A(_A), b(_b)
{
  assert(A.rows() == b.rows());
}

inline LinearEquation::LinearEquation(const Eigen::Index& var_size, const Eigen::Index& eq_size)
  : A(eq_size, var_size), b(eq_size)
{
}

inline LinearEquation::LinearEquation()
{
}

inline LinearEquation LinearEquation::Zero(const Eigen::Index& var_size, const Eigen::Index& eq_size)
{
  LinearEquation res(var_size, eq_size);
  res.setZero();
  return res;
}

inline void LinearEquation::resize(const Eigen::Index& var_size, const Eigen::Index& eq_size)
{
  A.conservativeResize(eq_size, var_size);
  b.conservativeResize(eq_size);
}

inline void LinearEquation::setZero()
{
  A.setZero();
  b.setZero();
}

inline bool LinearEquation::isFinite() const
{
  return eigen::isFinite(A) && eigen::isFinite(b);
}

inline Eigen::Index LinearEquation::variableSize() const
{
  return A.cols();
}

inline Eigen::Index LinearEquation::equationSize() const
{
  return A.rows();
}
}  // namespace ctrl
}  // namespace tobas
