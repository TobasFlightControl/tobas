#pragma once

#include "./qpsolver.hpp"

namespace quadprog
{
/**
 * @brief A Condensed and Sparse QP Formulation for Predictive Control [Jerez+, 2011]
 */
class PrimalDualInteriorPointSolver : public QuadProgSolver
{
  using super = QuadProgSolver;

public:
  explicit PrimalDualInteriorPointSolver();

  Eigen::VectorXd solve() override;

  bool setNumberOfIterations(const size_t& num_iter);
  bool setSigma(const double& sigma);
  bool setAlphaTolerance(const double& alpha_tol);

private:
  size_t num_iter_ = 10;  // TODO
  double sigma_ = 0.1;    // TODO
  double alpha_tol_ = 1e-3;

  bool is_first_solve_ = true;

  size_t var_dim_;
  size_t eq_dim_;
  size_t ineq_dim_;

  Eigen::VectorXd theta_;
  // Eigen::VectorXd nu_;
  Eigen::VectorXd lam_;
  Eigen::VectorXd s_;

  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;

  void initialize(const QuadProgProblem& scaled);
  double findAlpha(const Eigen::VectorXd& dlam, const Eigen::VectorXd& ds) const;
};
}  // namespace quadprog
