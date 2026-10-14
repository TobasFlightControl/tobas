// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <tobas_eigen_tools/tensor.hpp>
#include <tobas_quadprog/dual_active_set.hpp>

namespace tobas
{
namespace nlp
{
class SQP
{
public:
  explicit SQP();

  void initialize(
    const Eigen::VectorXd& _x0,
    std::function<double(const Eigen::VectorXd&)> _f,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> _g,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> _h,
    std::function<Eigen::RowVectorXd(const Eigen::VectorXd&)> _dfdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> _dgdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> _dhdx,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> _d_fdx,
    std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> _d_gdx,
    std::function<Eigen::Tensor3Xd(const Eigen::VectorXd&)> _d_hdx);

  std::expected<Eigen::VectorXd, std::string> solve();

  size_t iterations() const;

  void setMaximumIterations(size_t _max_iter);
  void setRelativeTolerance(double _rel_tol);
  void setVariableScales(const Eigen::VectorXd& _x_scale);

private:
  Eigen::Index n_;  ///< The number of optimization variables
  Eigen::Index m_;  ///< The number of inequality constraints
  Eigen::Index p_;  ///< The number of equality constraints

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
  size_t max_iter_ = 100;
  double rel_tol_ = 1e-3;
};
}  // namespace nlp
}  // namespace tobas
