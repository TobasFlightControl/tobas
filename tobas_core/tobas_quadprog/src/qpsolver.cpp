#include "../include/tobas_quadprog/qpsolver.hpp"

#include <tobas_eigen_tools/core.hpp>

using namespace std;
using namespace Eigen;

namespace quadprog
{
QuadProgProblem::QuadProgProblem(const Index& var_size, const Index& eq_size, const Index& ineq_size)
{
  resize(var_size, eq_size, ineq_size);
}

QuadProgProblem::QuadProgProblem()
{
}

void QuadProgProblem::resize(const Index& var_size, const Index& eq_size, const Index& ineq_size)
{
  P.conservativeResize(var_size, var_size);
  q.conservativeResize(var_size);
  G.conservativeResize(eq_size, var_size);
  h.conservativeResize(eq_size);
  A.conservativeResize(ineq_size, var_size);
  b.conservativeResize(ineq_size);
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

  res &= P.rows() == varSize();
  res &= P.cols() == varSize();
  res &= q.size() == varSize();
  res &= G.rows() == eqSize();
  res &= G.cols() == varSize();
  res &= h.size() == eqSize();
  res &= A.rows() == ineqSize();
  res &= A.cols() == varSize();
  res &= b.size() == ineqSize();

  return res;
}

bool QuadProgProblem::isFinite() const
{
  return eigen::isFinite(P) && eigen::isFinite(q) && eigen::isFinite(G) && eigen::isFinite(h) && eigen::isFinite(A) &&
         eigen::isFinite(b);
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

void QuadProgSolver::resize(const Index& var_size, const Index& eq_size, const Index& ineq_size)
{
  problem.resize(var_size, eq_size, ineq_size);
  x_scale.conservativeResize(var_size);
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

  // xの各要素が同程度の絶対値になるようにスケーリング (memo: 2-21)
  // 最適化変数は変化するが随伴変数は変化しない
  const DiagonalMatrix<double, Dynamic> x_scale_diag = x_scale.asDiagonal();
  scaled.P = x_scale_diag * problem.P * x_scale_diag;
  scaled.q = x_scale_diag * problem.q;
  scaled.G = problem.G * x_scale_diag;
  scaled.h = problem.h;
  scaled.A = problem.A * x_scale_diag;
  scaled.b = problem.b;

  // Pの対角成分の最大値が1になるようにスケーリング
  // ラグランジュ関数を定数倍しているだけなので最適化変数及び随伴変数は変化しない
  const auto P_diag_max = scaled.P.diagonal().maxCoeff();
  assert(P_diag_max > 0);
  scaled.P /= P_diag_max;
  scaled.q /= P_diag_max;
  scaled.G /= P_diag_max;
  scaled.h /= P_diag_max;
  scaled.A /= P_diag_max;
  scaled.b /= P_diag_max;

  // TODO: 各制約条件について，係数の絶対値の最大値が1になるようにスケーリング
  // XXX: これを行う場合は随伴変数が変化することに注意

  return scaled;
}

void QuadProgSolver::checkProblemValidity() const
{
  assert(problem.isSizeMatch());
  assert(problem.isFinite());
  assert(x_scale.size() == problem.varSize());
  assert((x_scale.array() > 0).all());
}
}  // namespace quadprog
