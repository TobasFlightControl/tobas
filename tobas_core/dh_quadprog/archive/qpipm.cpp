#include <dh_eigen_tools/core.hpp>
#include <qpipm/qpipm.hpp>

#include "../../include/dh_linear_control/quadprog/qpipm.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
QpIpmSolver::QpIpmSolver() : super()
{
}

VectorXd QpIpmSolver::solveQp()
{
  checkProblemValidity();

  // スケーリング
  const auto scaled = scaleProblem();

  // 内点法の標準形を作る (memo: 2-31)
  // https://qiita.com/taka_horibe/items/0c9b0993e0bd1c0135fa
  const auto var_size = scaled.varSize();
  const auto eq_size = scaled.eqSize();
  const auto ineq_size = scaled.ineqSize();
  const auto var_size_2 = var_size * 2 + ineq_size;
  const auto eq_size_2 = eq_size + ineq_size;

  MatrixXd Q(var_size_2, var_size_2);
  Q.setZero();
  Q.block(0, 0, var_size, var_size) = scaled.P;
  Q.block(0, var_size, var_size, var_size) = -scaled.P;
  Q.block(var_size, 0, var_size, var_size) = -scaled.P;
  Q.block(var_size, var_size, var_size, var_size) = scaled.P;

  VectorXd c(var_size_2);
  c.block(0, 0, var_size, 1) = scaled.q;
  c.block(var_size, 0, var_size, 1) = -scaled.q;
  c.block(var_size * 2, 0, ineq_size, 1).setZero();

  MatrixXd A(eq_size_2, var_size_2);
  A.block(0, 0, eq_size, var_size) = scaled.G;
  A.block(0, var_size, eq_size, var_size) = -scaled.G;
  A.block(0, var_size * 2, eq_size, ineq_size).setZero();
  A.block(eq_size, 0, ineq_size, var_size) = scaled.A;
  A.block(eq_size, var_size, ineq_size, var_size) = -scaled.A;
  A.block(eq_size, var_size * 2, ineq_size, ineq_size).diagonal().setOnes();

  VectorXd b(eq_size_2);
  b << scaled.h, scaled.b;

  // QPを解く
  qpipm::QuadProgSolver solver(Q, A, b, c);
  if (solver.solve().getExitFlag() != qpipm::Status::OPTIMAL)
  {
    throw runtime_error("Failed to solve QP.");
  }

  // 解を元のスケールに戻して返す
  const auto& x_tilde = solver.getState();
  const VectorXd x_plus = x_tilde.block(0, 0, var_size, 1);
  const VectorXd x_minus = x_tilde.block(var_size, 0, var_size, 1);
  const VectorXd x_scaled = x_plus - x_minus;
  return x_scaled.cwiseProduct(x_scale);
}
}  // namespace ctrl
