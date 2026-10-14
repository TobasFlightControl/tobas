// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/core.hpp>

namespace tobas::ctrl
{
class LinearEquation
{
public:
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  inline explicit LinearEquation(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b);
  inline explicit LinearEquation(Eigen::Index _var_size, Eigen::Index _eq_size);
  inline explicit LinearEquation();

  static inline LinearEquation Zero(Eigen::Index _var_size, Eigen::Index _eq_size);

  inline void resize(Eigen::Index _var_size, Eigen::Index _eq_size);
  inline void setZero();

  inline bool isFinite() const;

  /** Variable dimension. */
  inline Eigen::Index variableSize() const;

  /** Equation or inequality dimension. */
  inline Eigen::Index equationSize() const;

  /** Create a matrix equation for scaled variables. */
  LinearEquation scale(const Eigen::VectorXd& _scale) const;

  /** Discretize an equation for rates into an equation for increments. */
  LinearEquation discretise(double _dt) const;

  friend std::ostream& operator<<(std::ostream& _os, const LinearEquation& _arg);
};

inline LinearEquation::LinearEquation(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b) : A(_a), b(_b)
{
  assert(A.rows() == b.rows());
}

inline LinearEquation::LinearEquation(Eigen::Index _var_size, Eigen::Index _eq_size)
  : A(_eq_size, _var_size), b(_eq_size)
{
}

inline LinearEquation::LinearEquation()
{
}

inline LinearEquation LinearEquation::Zero(Eigen::Index _var_size, Eigen::Index _eq_size)
{
  LinearEquation res(_var_size, _eq_size);
  res.setZero();
  return res;
}

inline void LinearEquation::resize(Eigen::Index _var_size, Eigen::Index _eq_size)
{
  A.conservativeResize(_eq_size, _var_size);
  b.conservativeResize(_eq_size);
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
}  // namespace tobas::ctrl
