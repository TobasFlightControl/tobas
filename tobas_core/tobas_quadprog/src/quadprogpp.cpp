#include <QuadProg++/QuadProg++.hh>

#include "../include/tobas_quadprog/quadprogpp.hpp"

#define F_VALUE_THRESHOLD 1e+10

using namespace std;
using namespace Eigen;

namespace quadprogpp
{
void matrixEigenToQp(const MatrixXd& e, Matrix<double>& q)
{
  if (q.nrows() != e.rows() || q.ncols() != e.cols())
    q.resize(e.rows(), e.cols());

  for (size_t i = 0; i < e.rows(); ++i)
    for (size_t j = 0; j < e.cols(); ++j)
      q[i][j] = e(i, j);
}

void matrixQpToEigen(const Matrix<double>& q, MatrixXd& e)
{
  // Eigenは安易にresizeできないため，引数の時点でサイズが合っていることを確認する
  assert(e.rows() == q.nrows() && e.cols() == q.ncols());

  for (size_t i = 0; i < e.rows(); ++i)
    for (size_t j = 0; j < e.cols(); ++j)
      e(i, j) = q[i][j];
}

void vectorEigenToQp(const VectorXd& e, Vector<double>& q)
{
  assert(e.cols() == 1);

  if (q.size() != e.size())
    q.resize(e.rows());

  for (size_t i = 0; i < e.rows(); ++i)
    q[i] = e(i);
}

void vectorQpToEigen(const Vector<double>& q, VectorXd& e)
{
  assert(e.rows() == q.size());
  assert(e.cols() == 1);

  for (size_t i = 0; i < e.rows(); ++i)
    e(i) = q[i];
}
}  // namespace quadprogpp

namespace quadprog
{
QuadProgppSolver::QuadProgppSolver() : super()
{
}

VectorXd QuadProgppSolver::solve()
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
