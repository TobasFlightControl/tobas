#include <Eigen/LU>
#include <Eigen/Cholesky>

#include "../include/tobas_quadprog/primal_dual_interior_point.hpp"
#include "../include/tobas_quadprog/dual_active_set.hpp"

using namespace std;
using namespace Eigen;

namespace quadprog
{
PrimalDualInteriorPointSolver::PrimalDualInteriorPointSolver()
{
}

bool PrimalDualInteriorPointSolver::solve()
{
  checkProblemValidity();

  // Scaling
  const auto scaled = scaleProblem();

  if (is_first_solve_)
  {
    if (!initialize(scaled))
    {
      error_msg_ = "Failed to initialize decision variables.";
      return false;
    }
    is_first_solve_ = false;
  }

  // 制約がない場合は停留点を求めて終わり
  if (eq_dim_ == 0 && ineq_dim_ == 0)
  {
    const LLT<MatrixXd> llt(scaled.P);
    if (llt.info() == NumericalIssue)
    {
      error_msg_ = "Cholesky decomposition failed.";
      return false;
    }

    theta_ = -llt.solve(scaled.q);
    x_opt_ = theta_.cwiseProduct(x_scale);
    return true;
  }

  // Iteration
  // Real-time rquirements will impose a hard bound on the number of interior-point iterations,
  // hense it is assumed fixed a priori.
  for (size_t _ = 0; _ < num_iter_; ++_)
  {
    const DiagonalMatrix<double, Dynamic> W = lam_.cwiseProduct(s_.cwiseInverse()).asDiagonal();
    const double mu = lam_.dot(s_) / static_cast<double>(eq_dim_ + ineq_dim_);  // 制約なしだとNaN
    const VectorXd sigma_mu_sinv = sigma_ * mu * s_.cwiseInverse();

    // 1
    A_.topLeftCorner(var_dim_, var_dim_) = scaled.P + scaled.A.transpose() * W * scaled.A;
    A_.topRightCorner(var_dim_, eq_dim_) = scaled.G.transpose();
    A_.bottomLeftCorner(eq_dim_, var_dim_) = scaled.G;

    // 2
    b_.head(var_dim_) = -scaled.q - scaled.A.transpose() * (lam_ - W * scaled.b + sigma_mu_sinv);
    b_.bottomRows(eq_dim_) = -scaled.G * theta_ + scaled.h;

    // 3
    const PartialPivLU<MatrixXd> lu(A_);
    const VectorXd z = lu.solve(b_);  // TODO: 正則かどうかを確かめる
    const VectorXd theta_dtheta = z.head(var_dim_);
    // const VectorXd dnu = z.bottomRows(eq_dim_);
    const VectorXd dtheta = theta_dtheta - theta_;
    const VectorXd G_theta_g = scaled.A * theta_dtheta - scaled.b;

    // 4
    const VectorXd dlam = W * G_theta_g + sigma_mu_sinv;

    // 5
    const VectorXd ds = -s_ - G_theta_g;

    // 6
    const double alpha = findAlpha(dlam, ds);

    // 7
    theta_ += alpha * dtheta;
    // nu_ += alpha * dnu;
    lam_ += alpha * dlam;
    s_ += alpha * ds;
  }

  // TODO: 解の収束と実行可能性をチェック

  // 解を元のスケールに戻す
  x_opt_ = theta_.cwiseProduct(x_scale);

  return true;
}

bool PrimalDualInteriorPointSolver::setNumberOfIterations(const size_t& num_iter)
{
  if (num_iter == 0)
    return false;

  num_iter_ = num_iter;
  return true;
}

bool PrimalDualInteriorPointSolver::setSigma(const double& sigma)
{
  if (sigma <= 0. || 1. <= sigma)
    return false;

  sigma_ = sigma;
  return true;
}

bool PrimalDualInteriorPointSolver::setAlphaTolerance(const double& alpha_tol)
{
  if (alpha_tol <= 0. || 1. <= alpha_tol)
    return false;

  alpha_tol_ = alpha_tol;
  return true;
}

bool PrimalDualInteriorPointSolver::initialize(const QuadProgProblem& scaled)
{
  var_dim_ = scaled.q.rows();
  eq_dim_ = scaled.h.rows();
  ineq_dim_ = scaled.b.rows();

  // 実行可能な初期解をアクティブセット法で求める
  DualActiveSetSolver active_set_solver_;
  active_set_solver_.problem = scaled;
  active_set_solver_.x_scale = VectorXd::Ones(problem.varSize());
  if (!active_set_solver_.solve())
    return false;
  theta_ = active_set_solver_.solution();

  // 不等式制約のラグランジュ乗数とスラック変数の初期値を1に設定
  lam_ = VectorXd::Ones(ineq_dim_);
  s_ = VectorXd::Ones(ineq_dim_);

  A_ = MatrixXd::Zero(var_dim_ + eq_dim_, var_dim_ + eq_dim_);
  b_ = VectorXd::Zero(var_dim_ + eq_dim_);

  return true;
}

double PrimalDualInteriorPointSolver::findAlpha(const VectorXd& dlam, const VectorXd& ds) const
{
  double lb = 0.;
  double ub = 1.;

  while (ub - lb > alpha_tol_)
  {
    const auto mid = (lb + ub) / 2;
    const VectorXd lam = lam_ + mid * dlam;
    const VectorXd s = s_ + mid * ds;
    if ((lam.array() > 0).all() && (s.array() > 0).all())
      lb = mid;
    else
      ub = mid;
  }

  return lb;
}
}  // namespace quadprog
