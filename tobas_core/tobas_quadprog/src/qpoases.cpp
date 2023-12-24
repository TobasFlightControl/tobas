#include <qpOASES.hpp>

#include <tobas_std_tools/math.hpp>
#include <tobas_eigen_tools/core.hpp>

#include "../include/tobas_quadprog/qpoases.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace quadprog
{
QpOasesSolver::QpOasesSolver() : super()
{
}

VectorXd QpOasesSolver::solve()
{
  checkProblemValidity();

  // スケーリング
  const auto scaled = scaleProblem();

  // qpOASES用の行列を作成
  const size_t var_size = scaled.varSize();
  const size_t con_size = scaled.eqSize() + scaled.ineqSize();

  double H[sqr(var_size)];
  double g[var_size];
  double A[con_size * var_size];
  double lb[var_size];
  double ub[var_size];
  double lbA[con_size];
  double ubA[con_size];

  memcpy(H, scaled.P.data(), sizeof(H));  // Hは対称行列だから列優先でも行優先でもコピーできる
  memcpy(g, scaled.q.data(), sizeof(g));

  // 列優先の場合を考慮し，要素を1つずつコピー
  const MatrixXd A_eigen = eigen_tools::concat(scaled.G, scaled.A, 0);
  for (size_t r = 0; r < con_size; ++r)
  {
    for (size_t c = 0; c < var_size; ++c)
    {
      A[r * var_size + c] = A_eigen(r, c);
    }
  }

  for (size_t i = 0; i < var_size; ++i)
  {
    lb[i] = -qpOASES::INFTY;
    ub[i] = qpOASES::INFTY;
  }

  const VectorXd inf = VectorXd::Constant(scaled.ineqSize(), -qpOASES::INFTY);
  const VectorXd lbA_eigen = eigen_tools::concat(scaled.h, inf, 0);
  memcpy(lbA, lbA_eigen.data(), sizeof(lbA));

  const VectorXd ubA_eigen = eigen_tools::concat(scaled.h, scaled.b, 0);
  memcpy(ubA, ubA_eigen.data(), sizeof(ubA));

  // QPソルバを作成
  qpOASES::QProblem solver(var_size, con_size);

  // QPソルバの設定
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = scaled.eqSize() > 0 ? qpOASES::BT_TRUE : qpOASES::BT_FALSE;
  solver.setOptions(options);

  // QPを解く
  solver.init(H, g, A, lb, ub, lbA, ubA, nWSR_);

  double x_opt[var_size];
  if (solver.getPrimalSolution(x_opt) != qpOASES::SUCCESSFUL_RETURN)
  {
    throw runtime_error("Failed to solve QP.");
  }

  // 解を元のスケールに戻して返す
  VectorXd x_scaled = Map<VectorXd>(x_opt, var_size);
  return x_scaled.cwiseProduct(x_scale);
}
}  // namespace quadprog
