#include <iostream>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/linalg.hpp>

#include "../include/tobas_linear_control/util.hpp"

#define EPS numeric_limits<double>::epsilon()

using namespace std;
using namespace Eigen;

namespace ctrl
{
MatrixXd ctrb(const MatrixXd& A, const MatrixXd& B)
{
  const auto x_size = A.rows();
  const auto u_size = B.cols();

  assert(A.rows() == x_size && A.cols() == x_size);
  assert(B.rows() == x_size && B.cols() == u_size);

  MatrixXd Mc(x_size, u_size * x_size);
  MatrixXd tmp = B;
  for (int i = 0; i < x_size; ++i)
  {
    Mc.block(0, u_size * i, x_size, u_size) = tmp;
    tmp = A * tmp;
  }

  return Mc;
}

MatrixXd obsv(const MatrixXd& A, const MatrixXd& C)
{
  const auto x_size = A.rows();
  const auto y_size = C.rows();

  assert(A.rows() == x_size && A.cols() == x_size);
  assert(C.rows() == y_size && C.cols() == x_size);

  MatrixXd Mo(y_size * x_size, x_size);
  MatrixXd tmp = C;
  for (int i = 0; i < x_size; ++i)
  {
    Mo.block(y_size * i, 0, y_size, x_size) = tmp;
    tmp = tmp * A;
  }

  return Mo;
}

bool isControllable(const MatrixXd& A, const MatrixXd& B)
{
  const auto x_size = A.rows();
  const auto Mc = ctrb(A, B);
  const auto rank = eigen_tools::matrixRank(Mc);
  return rank == x_size;
}

bool isObservable(const MatrixXd& A, const MatrixXd& C)
{
  const auto x_size = A.rows();
  const auto Mo = obsv(A, C);
  const auto rank = eigen_tools::matrixRank(Mo);
  return rank == x_size;
}

LinearEquation matIneqFromRange(const VectorXd& lb, const VectorXd& ub, const double& inf)
{
  assert(lb.rows() == ub.rows());
  assert(((ub - lb).array() >= 0.).all());

  const auto size = lb.rows();

  const MatrixXd E = MatrixXd::Identity(size, size);
  const auto left = eigen_tools::concat(-E, E, 0);
  const auto right = eigen_tools::concat(-lb, ub, 0);
  const auto is_valid = (right.array().abs() < inf).eval();
  const auto num_valid = is_valid.count();

  LinearEquation res(size, num_valid);
  int row = 0;  // 行列不等式の行番号
  for (int i = 0; i < size * 2; ++i)
  {
    if (!is_valid(i))
    {
      continue;
    }
    res.A.block(row, 0, 1, size) = left.row(i);
    res.b(row) = right(i);
    ++row;
  }

  return res;
}

double firstOrderPos(const double& x0, const double& xd, const double& tau, const double& t)
{
  return xd - exp(-t / (tau + EPS)) * (xd - x0);
}

double firstOrderVel(
  const double& x0,
  const double& v0,
  const double& vd,
  const double& tau,
  const double& t)
{
  return x0 + vd * t - tau * (1 - exp(-t / (tau + EPS))) * (vd - v0);
}
}  // namespace ctrl
