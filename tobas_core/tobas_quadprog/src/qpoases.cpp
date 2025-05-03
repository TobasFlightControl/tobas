#include <qpOASES.hpp>

#include <tobas_math/core.hpp>
#include <tobas_eigen_tools/core.hpp>

#include "../include/tobas_quadprog/qpoases.hpp"

using namespace std;
using namespace Eigen;

namespace quadprog
{
QpOasesSolver::QpOasesSolver() : super()
{
}

bool QpOasesSolver::solve()
{
  checkProblemValidity();

  // スケーリング
  const auto scaled = scaleProblem();

  // qpOASES用の行列を作成
  const auto var_size = scaled.varSize();
  const auto con_size = scaled.eqSize() + scaled.ineqSize();

  double H[math::sqr(var_size)];
  double g[var_size];
  double A[con_size * var_size];
  double lb[var_size];
  double ub[var_size];
  double lbA[con_size];
  double ubA[con_size];

  memcpy(H, scaled.P.data(), sizeof(H));  // Hは対称行列だから列優先でも行優先でもコピーできる
  memcpy(g, scaled.q.data(), sizeof(g));

  // 列優先の場合を考慮し，要素を1つずつコピー
  const MatrixXd A_eigen = eigen::concat(scaled.G, scaled.A, 0);
  for (Index r = 0; r < con_size; ++r) {
    for (Index c = 0; c < var_size; ++c) {
      A[r * var_size + c] = A_eigen(r, c);
    }
  }

  for (Index i = 0; i < var_size; ++i) {
    lb[i] = -qpOASES::INFTY;
    ub[i] = qpOASES::INFTY;
  }

  const VectorXd inf = VectorXd::Constant(scaled.ineqSize(), -qpOASES::INFTY);
  const VectorXd lbA_eigen = eigen::concat(scaled.h, inf, 0);
  memcpy(lbA, lbA_eigen.data(), sizeof(lbA));

  const VectorXd ubA_eigen = eigen::concat(scaled.h, scaled.b, 0);
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
  const auto ret = solver.getPrimalSolution(x_opt);
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    error_msg_ = "qpOASES finished with error code " + to_string(ret);
    return false;
  }

  // 解を元のスケールに戻す
  VectorXd x_scaled = Map<VectorXd>(x_opt, var_size);
  x_opt_ = x_scaled.cwiseProduct(x_scale);

  return true;
}
}  // namespace quadprog
