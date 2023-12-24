#include <QuadProg++/QuadProg++.hh>

#include "../include/tobas_quadprog/quadprogpp.hpp"
#include "../include/tobas_quadprog/utilities/qp_eigen.hpp"

#define F_VALUE_THRESHOLD 1e+10

using namespace std;
using namespace Eigen;

namespace quadprog
{
QuadProgppSolver::QuadProgppSolver() : super()
{
}

Eigen::VectorXd QuadProgppSolver::solve()
{
  checkProblemValidity();

  // スケーリング
  const auto scaled = scaleProblem();

  // QuadProg++の行列に変換
  quadprogpp::matrixEigenToQp(scaled.P, G_);
  quadprogpp::vectorEigenToQp(scaled.q, g0_);
  quadprogpp::matrixEigenToQp(-scaled.G.transpose(), CE_);
  quadprogpp::vectorEigenToQp(scaled.h, ce0_);
  quadprogpp::matrixEigenToQp(-scaled.A.transpose(), CI_);
  quadprogpp::vectorEigenToQp(scaled.b, ci0_);

  // QPを解く
  const double f_value = quadprogpp::solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x_);
  if (f_value > F_VALUE_THRESHOLD)
  {
    throw runtime_error("Failed to solve QP.");
  }

  VectorXd x_scaled(x_.size());
  quadprogpp::vectorQpToEigen(x_, x_scaled);

  // 解を元のスケールに戻して返す
  return x_scaled.cwiseProduct(x_scale);
}
}  // namespace quadprog
