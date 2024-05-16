#pragma once

#include <Eigen/Core>

namespace quadprog
{
/**
 * @brief Quadratic problem.
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

  explicit QuadProgProblem(const size_t& var_size, const size_t& eq_size, const size_t& ineq_size);
  explicit QuadProgProblem();

  void resize(const size_t& var_size, const size_t& eq_size, const size_t& ineq_size);
  void setZero();

  bool isSizeMatch() const;
  bool isFinite() const;

  inline size_t varSize() const;
  inline size_t eqSize() const;
  inline size_t ineqSize() const;

  friend std::ostream& operator<<(std::ostream& os, const QuadProgProblem& arg);
};

/**
 * @brief A base class of quadratic problem solver.
 * minimize 0.5 x^T P x + q^T x s.t. G x = h & A x <= b
 */
class QuadProgSolver
{
public:
  QuadProgProblem problem;
  Eigen::VectorXd x_scale;  // 決定変数のスケール

  explicit QuadProgSolver();

  virtual Eigen::VectorXd solve() = 0;

  void resize(const size_t& var_size, const size_t& eq_size, const size_t& ineq_size);
  void setZero();

  friend std::ostream& operator<<(std::ostream& os, const QuadProgSolver& arg);

protected:
  QuadProgProblem scaleProblem() const;
  void checkProblemValidity() const;
};

inline size_t QuadProgProblem::varSize() const
{
  return q.rows();
}

inline size_t QuadProgProblem::eqSize() const
{
  return h.rows();
}

inline size_t QuadProgProblem::ineqSize() const
{
  return b.rows();
}
}  // namespace quadprog
