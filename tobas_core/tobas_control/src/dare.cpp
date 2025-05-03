#include <iostream>

#include <tobas_eigen_tools/linalg.hpp>

#include "../include/tobas_control/dare.hpp"
#include "../include/tobas_control/util.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
MatrixXd
dare(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R, const double& tol, size_t max_iter)
{
  const auto n = A.rows();
  [[maybe_unused]] const auto l = B.cols();

  assert(A.cols() == n);
  assert(B.rows() == n);
  assert(Q.rows() == n && Q.cols() == n);
  assert(R.rows() == l && R.rows() == l);
  assert(isControllable(A, B));
  assert(eigen::isSymmetricSemiPositiveDefinite(Q));
  assert(eigen::isSymmetricPositiveDefinite(R));
  assert(tol > 0.);

  const MatrixXd I = MatrixXd::Identity(n, n);
  MatrixXd X_prev = MatrixXd::Zero(n, n);
  MatrixXd X_next = MatrixXd::Identity(n, n);
  size_t iter = 0;

  while ((X_next - X_prev).norm() / X_next.norm() > tol) {
    X_prev = X_next;

    // 事前推定
    const MatrixXd X_mid = A.transpose() * X_prev.selfadjointView<Lower>() * A + Q;

    // 事後推定
    const MatrixXd XB = X_mid.selfadjointView<Lower>() * B;
    const auto G = XB * (B.transpose() * XB + R).inverse();
    const auto I_GBt = I - G * B.transpose();
    X_next = I_GBt * X_mid.selfadjointView<Lower>();

    if (iter++ > max_iter) {
      throw runtime_error("DARE failed to converge in " + to_string(max_iter) + " iterations.");
    }
  }

  cout << "DARE has successfully converged in " << iter << " iterations." << endl;
  return X_next;
}
}  // namespace ctrl
