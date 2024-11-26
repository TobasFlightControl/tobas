#include <iostream>

#include <tobas_eigen_tools/linalg.hpp>

#include "../include/tobas_nlp/sqp.hpp"

#define EPS numeric_limits<double>::epsilon()
#define INF numeric_limits<double>::infinity()
#define TOL_FACTOR 100.

using namespace std;
using namespace Eigen;

namespace nlp
{
SQP::SQP()
{
}

bool SQP::initialize(
  const VectorXd& x0,
  const MatrixXd& H0,
  const VectorXd& x_scale,
  function<double(const VectorXd&)> f,
  function<VectorXd(const VectorXd&)> g,
  function<VectorXd(const VectorXd&)> h,
  function<RowVectorXd(const VectorXd&)> dfdx,
  function<MatrixXd(const VectorXd&)> dgdx,
  function<MatrixXd(const VectorXd&)> dhdx)
{
  if (x0.size() != H0.rows() || x0.size() != H0.cols())
  {
    cerr << "The size of the initial hessian matrix does not match that of the initial variables." << endl;
    return false;
  }

  if (!eigen_tools::isSymmetricPositiveDefinite(H0))
  {
    cerr << "The initial hessian matrix must be symmetric positive definite." << endl;
    return false;
  }

  if (x_scale.size() != x0.size())
  {
    cerr << "The size of the scale vector does not match that of the initial variables." << endl;
    return false;
  }

  if ((x_scale.array() <= 0.).any())
  {
    cerr << "The scale of variables must be positive." << endl;
    return false;
  }

  if (dfdx(x0).size() != x0.size())
  {
    cerr << "The return size of df/dx(x) does not match that of the initial variables." << endl;
    return false;
  }

  if (dgdx(x0).rows() != g(x0).size())
  {
    cerr << "The number of rows of dg/dx(x) does not match the size of g(x)." << endl;
    return false;
  }

  if (dgdx(x0).cols() != x0.size())
  {
    cerr << "The number of columns of dg/dx(x) does not match the size of the initial variables." << endl;
    return false;
  }

  if (dhdx(x0).rows() != h(x0).size())
  {
    cerr << "The number of rows of dh/dx(x) does not match the size of h(x)." << endl;
    return false;
  }

  if (dhdx(x0).cols() != x0.size())
  {
    cerr << "The number of columns of dh/dx(x) does not match the size of the initial variables." << endl;
    return false;
  }

  x_ = x0;
  H_ = H0;
  n_ = x0.size();

  qp_.x_scale = x_scale;

  f_ = f;
  g_ = g;
  h_ = h;
  dfdx_ = dfdx;
  dgdx_ = dgdx;
  dhdx_ = dhdx;

  return true;
}

SQP::error_t SQP::solve()
{
  iter_ = 0;

  while (true)
  {
    // 局所的なQPを解く
    qp_.problem.P = H_;
    qp_.problem.q = dfdx_(x_).transpose();
    qp_.problem.A = dgdx_(x_);
    qp_.problem.b = -g_(x_);
    qp_.problem.G = dhdx_(x_);
    qp_.problem.h = -h_(x_);

    if (!qp_.solve())
      return error_code_ = E_QP_FAILED;

    const auto& dx = qp_.solution();
    const auto lam = qp_.getLagrangeMultipliersIneq();
    const auto mu = qp_.getLagrangeMultipliersEq();

    // 終了判定
    // cf. https://kotakku.github.io/cpp_robotics/tech_note/optimize/tolerances_and_stopping/
    if ((dx.cwiseAbs().array() < (rel_tol_ * qp_.x_scale).array()).all())
      return error_code_ = E_NO_ERROR;

    // ヘッセ行列を更新
    const auto next_x = x_ + dx;
    const auto& s = dx;
    const VectorXd y = (dLdx(next_x, lam, mu) - dLdx(x_, lam, mu)).transpose();
    const VectorXd Hs = H_ * s;
    H_ += y * (y.transpose() / y.dot(s)) - Hs * (Hs.transpose() / (s.transpose() * Hs).value());  // BFGS method
    eigen_tools::symmetrise(H_);

    // 最適化変数を更新
    x_ = next_x;

    // 繰り返し回数の上限チェック
    ++iter_;
    if (max_iter_ > 0 && iter_ > max_iter_)
      return error_code_ = E_MAX_ITERATION_EXCEEDED;
  }
}

const VectorXd& SQP::optimal() const
{
  return x_;
}

size_t SQP::iterations() const
{
  return iter_;
}

SQP::error_t SQP::errorCode() const
{
  return error_code_;
}

const char* SQP::errorMessage() const
{
  switch (error_code_)
  {
    case E_NO_ERROR:
      return "No error.";
    case E_MAX_ITERATION_EXCEEDED:
      return "The number of iterations exceeded the limit.";
    case E_QP_FAILED:
      return qp_.errorMessage().c_str();
    default:
      return "Unknown error.";
  }
}

bool SQP::setMaximumIterations(size_t max_iter)
{
  max_iter_ = max_iter;
  return true;
}

bool SQP::setRelativeTolerance(double rel_tol)
{
  if (rel_tol <= 0.)
  {
    cerr << "Relative tolerance must be positive." << endl;
    return false;
  }

  rel_tol_ = rel_tol;
  return true;
}

RowVectorXd SQP::dLdx(const VectorXd& x, const VectorXd& lam, const VectorXd& mu)
{
  return dfdx_(x) + lam.transpose() * dgdx_(x) + mu.transpose() * dhdx_(x);
}
}  // namespace nlp
