// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace quadprog
{
/** Q
 * uadratic problem.
 * minimize 0.5 x^T P x + q^T x s.t. G x = h & A x <= b
 */
class QuadProgProblem
{
public:
  Eigen::MatrixXd P;
  Eigen::VectorXd q;
  Eigen::MatrixXd G;
  Eigen::VectorXd h;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  explicit QuadProgProblem(const Eigen::Index& var_size, const Eigen::Index& eq_size, const Eigen::Index& ineq_size);
  explicit QuadProgProblem();

  void resize(const Eigen::Index& var_size, const Eigen::Index& eq_size, const Eigen::Index& ineq_size);
  void setZero();

  bool isSizeMatch() const;
  bool isFinite() const;

  inline Eigen::Index varSize() const;
  inline Eigen::Index eqSize() const;
  inline Eigen::Index ineqSize() const;

  friend std::ostream& operator<<(std::ostream& os, const QuadProgProblem& arg);
};

/**
 * A base class of quadratic problem solver.
 * minimize 0.5 x^T P x + q^T x s.t. G x = h & A x <= b
 */
class QuadProgSolver
{
public:
  QuadProgProblem problem;
  Eigen::VectorXd x_scale;  ///< Decision variable scale.

  explicit QuadProgSolver();

  virtual std::expected<Eigen::VectorXd, std::string> solve() = 0;

  void resize(const Eigen::Index& var_size, const Eigen::Index& eq_size, const Eigen::Index& ineq_size);
  void setZero();

  friend std::ostream& operator<<(std::ostream& os, const QuadProgSolver& arg);

protected:
  QuadProgProblem scaleProblem() const;
  void checkProblemValidity() const;
};

inline Eigen::Index QuadProgProblem::varSize() const
{
  return q.rows();
}

inline Eigen::Index QuadProgProblem::eqSize() const
{
  return h.rows();
}

inline Eigen::Index QuadProgProblem::ineqSize() const
{
  return b.rows();
}
}  // namespace quadprog
}  // namespace tobas
