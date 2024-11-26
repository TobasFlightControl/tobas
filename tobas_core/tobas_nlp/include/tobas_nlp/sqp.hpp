#pragma once

#include <tobas_quadprog/dual_active_set.hpp>

namespace nlp
{
class SQP
{
public:
  enum error_t
  {
    E_NO_ERROR = 0,
    E_MAX_ITERATION_EXCEEDED = -1,
    E_QP_FAILED = -2,
  };

  explicit SQP();

  bool initialize(
    const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& H0,
    const Eigen::VectorXd& x_scale,
    std::function<double(const Eigen::VectorXd&)> f,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> g,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
    std::function<Eigen::RowVectorXd(const Eigen::VectorXd&)> dfdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dgdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dhdx);

  error_t solve();

  const Eigen::VectorXd& optimal() const;
  size_t iterations() const;

  error_t errorCode() const;
  const char* errorMessage() const;

  bool setMaximumIterations(size_t max_iter);
  bool setRelativeTolerance(double rel_tol);

private:
  error_t error_code_;

  Eigen::VectorXd x_;
  Eigen::MatrixXd H_;
  Eigen::Index n_;
  size_t iter_;

  std::function<double(const Eigen::VectorXd&)> f_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> g_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h_;
  std::function<Eigen::RowVectorXd(const Eigen::VectorXd&)> dfdx_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dgdx_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dhdx_;

  quadprog::DualActiveSetSolver qp_;

  // Configurations
  size_t max_iter_ = 0;
  double rel_tol_ = 1e-6;

  Eigen::RowVectorXd dLdx(const Eigen::VectorXd& x, const Eigen::VectorXd& lam, const Eigen::VectorXd& mu);
};
}  // namespace nlp
