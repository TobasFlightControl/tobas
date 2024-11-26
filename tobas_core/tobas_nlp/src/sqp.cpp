#include <iostream>

#include <tobas_eigen_tools/linalg.hpp>

#include "../include/tobas_nlp/sqp.hpp"

#define EPS 1e-6

using namespace std;
using namespace Eigen;

namespace nlp
{
SQP::SQP()
{
}

void SQP::initialize(
  const VectorXd& x0,
  const VectorXd& x_scale,
  function<double(const VectorXd&)> f,
  function<VectorXd(const VectorXd&)> g,
  function<VectorXd(const VectorXd&)> h,
  function<RowVectorXd(const VectorXd&)> dfdx,
  function<MatrixXd(const VectorXd&)> dgdx,
  function<MatrixXd(const VectorXd&)> dhdx,
  function<MatrixXd(const VectorXd&)> dFdx,
  function<Tensor3Xd(const VectorXd&)> dGdx,
  function<Tensor3Xd(const VectorXd&)> dHdx)
{
  n_ = x0.size();
  m_ = dgdx(x0).size();
  p_ = dhdx(x0).size();

  x_ = x0;
  lam_ = VectorXd::Zero(m_);
  mu_ = VectorXd::Zero(p_);

  f_ = f;
  g_ = g;
  h_ = h;
  dfdx_ = dfdx;
  dgdx_ = dgdx;
  dhdx_ = dhdx;
  dFdx_ = dFdx;
  dGdx_ = dGdx;
  dHdx_ = dHdx;

  qp_.x_scale = x_scale;
}

SQP::error_t SQP::solve()
{
  iter_ = 0;

  while (true)
  {
    // 繰り返し回数の上限チェック
    ++iter_;
    if (max_iter_ > 0 && iter_ > max_iter_)
      return error_code_ = E_MAX_ITERATION_EXCEEDED;

    // ラグランジュ関数のヘッセ行列を計算
    const MatrixXd H = dFdx_(x_) + lam_.transpose() * dGdx_(x_) + mu_.transpose() * dHdx_(x_);

    // 局所的なQPを解く
    qp_.problem.P = eigen_tools::nearestPositiveDefinite(H, EPS);
    qp_.problem.q = dfdx_(x_).transpose();
    qp_.problem.A = dgdx_(x_);
    qp_.problem.b = -g_(x_);
    qp_.problem.G = dhdx_(x_);
    qp_.problem.h = -h_(x_);

    if (!qp_.solve())
      return error_code_ = E_QP_FAILED;

    const auto& dx = qp_.solution();

    // 最適化変数を更新
    x_ += dx;
    lam_ = qp_.getLagrangeMultipliersIneq();
    mu_ = qp_.getLagrangeMultipliersEq();

    // 終了判定
    // cf. https://kotakku.github.io/cpp_robotics/tech_note/optimize/tolerances_and_stopping/
    if ((dx.cwiseAbs().array() < (rel_tol_ * qp_.x_scale).array()).all())
      return error_code_ = E_NO_ERROR;
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
}  // namespace nlp
