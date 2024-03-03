#include <tobas_eigen_tools/core.hpp>

#include "../include/tobas_quadprog/qpsolver.hpp"

using namespace std;
using namespace Eigen;

namespace quadprog
{
QuadProgProblem::QuadProgProblem(
  const size_t& var_size,
  const size_t& eq_size,
  const size_t& ineq_size)
{
  resize(var_size, eq_size, ineq_size);
}

QuadProgProblem::QuadProgProblem()
{
}

void QuadProgProblem::resize(const size_t& var_size, const size_t& eq_size, const size_t& ineq_size)
{
  eigen_tools::resizeIfNecessary(P, var_size, var_size);
  eigen_tools::resizeIfNecessary(q, var_size);
  eigen_tools::resizeIfNecessary(G, eq_size, var_size);
  eigen_tools::resizeIfNecessary(h, eq_size);
  eigen_tools::resizeIfNecessary(A, ineq_size, var_size);
  eigen_tools::resizeIfNecessary(b, ineq_size);
}

void QuadProgProblem::setZero()
{
  P.setZero();
  q.setZero();
  G.setZero();
  h.setZero();
  A.setZero();
  b.setZero();
}

bool QuadProgProblem::isSizeMatch() const
{
  bool res = true;

  res &= static_cast<size_t>(P.rows()) == varSize();
  res &= static_cast<size_t>(P.cols()) == varSize();
  res &= static_cast<size_t>(q.size()) == varSize();
  res &= static_cast<size_t>(G.rows()) == eqSize();
  res &= static_cast<size_t>(G.cols()) == varSize();
  res &= static_cast<size_t>(h.size()) == eqSize();
  res &= static_cast<size_t>(A.rows()) == ineqSize();
  res &= static_cast<size_t>(A.cols()) == varSize();
  res &= static_cast<size_t>(b.size()) == ineqSize();

  return res;
}

bool QuadProgProblem::isFinite() const
{
  return eigen_tools::isFinite(P) && eigen_tools::isFinite(q) && eigen_tools::isFinite(G)
         && eigen_tools::isFinite(h) && eigen_tools::isFinite(A) && eigen_tools::isFinite(b);
}

ostream& operator<<(ostream& os, const QuadProgProblem& arg)
{
  os << "P:\n" << arg.P << endl;
  os << "q:\n" << arg.q << endl;
  os << "G:\n" << arg.G << endl;
  os << "h:\n" << arg.h << endl;
  os << "A:\n" << arg.A << endl;
  os << "b:\n" << arg.b << endl;
  return os;
}

QuadProgSolver::QuadProgSolver()
{
}

void QuadProgSolver::resize(const size_t& var_size, const size_t& eq_size, const size_t& ineq_size)
{
  problem.resize(var_size, eq_size, ineq_size);
  eigen_tools::resizeIfNecessary(x_scale, var_size);
}

void QuadProgSolver::setZero()
{
  problem.setZero();
  x_scale.setZero();
}

ostream& operator<<(ostream& os, const QuadProgSolver& arg)
{
  os << "problem:\n" << arg.problem << endl;
  os << "x_scale:\n" << arg.x_scale.transpose() << endl;
  return os;
}

QuadProgProblem QuadProgSolver::scaleProblem() const
{
  QuadProgProblem scaled;

  // xが[-1, 1]の範囲に収まるように全体をスケーリング (ChatGPT)
  const DiagonalMatrix<double, Dynamic> x_scale_diag = x_scale.asDiagonal();
  scaled.P = x_scale_diag * problem.P * x_scale_diag;
  scaled.q = x_scale_diag * problem.q;
  scaled.G = problem.G * x_scale_diag;
  scaled.h = problem.h;
  scaled.A = problem.A * x_scale_diag;
  scaled.b = problem.b;

  // 目的関数をPの要素和でスケーリング (ChatGPT)
  // 理論上結果には影響しない
  const double P_norm = scaled.P.sum();
  assert(P_norm > 0);  // Pは正定行列
  scaled.P /= P_norm;
  scaled.q /= P_norm;

  return scaled;
}

void QuadProgSolver::checkProblemValidity() const
{
  assert(problem.isSizeMatch());
  assert(problem.isFinite());
  assert(static_cast<size_t>(x_scale.size()) == problem.varSize());
  assert((x_scale.array() > 0).all());
}
}  // namespace quadprog
