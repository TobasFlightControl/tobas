#pragma once

#include <tobas_eigen_tools/tensor.hpp>
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

  void initialize(
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& x_scale,
    std::function<double(const Eigen::VectorXd&)> f,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> g,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
    std::function<Eigen::RowVectorXd(const Eigen::VectorXd&)> dfdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dgdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dhdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dFdx,
    std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> dGdx,
    std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> dHdx);

  error_t solve();

  const Eigen::VectorXd& optimal() const;
  size_t iterations() const;

  error_t errorCode() const;
  const char* errorMessage() const;

  bool setMaximumIterations(size_t max_iter);
  bool setRelativeTolerance(double rel_tol);

private:
  error_t error_code_;

  Eigen::Index n_;  // The number of optimization variables
  Eigen::Index m_;  // The number of inequality constraints
  Eigen::Index p_;  // The number of equality constraints

  Eigen::VectorXd x_;
  Eigen::VectorXd lam_;
  Eigen::VectorXd mu_;

  std::function<double(const Eigen::VectorXd&)> f_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> g_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h_;
  std::function<Eigen::RowVectorXd(const Eigen::VectorXd&)> dfdx_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dgdx_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dhdx_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> dFdx_;
  std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> dGdx_;
  std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> dHdx_;

  size_t iter_;
  quadprog::DualActiveSetSolver qp_;

  // Configurations
  size_t max_iter_ = 0;
  double rel_tol_ = 1e-6;
};
}  // namespace nlp
