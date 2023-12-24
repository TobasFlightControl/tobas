#include <iostream>

#include <dh_eigen_tools/linalg.hpp>

#include "../include/dh_linear_control/dare.hpp"
#include "../include/dh_linear_control/util.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
MatrixXd dare(
  const MatrixXd& A,
  const MatrixXd& B,
  const MatrixXd& Q,
  const MatrixXd& R,
  DareMethod method,
  const double& tol,
  size_t max_iter)
{
  const size_t n = A.rows();
  const size_t m = B.cols();

  assert(A.cols() == n);
  assert(B.rows() == n);
  assert(Q.rows() == n && Q.cols() == n);
  assert(R.rows() == m && R.rows() == m);
  assert(isControllable(A, B));
  assert(eigen_tools::isSymmetricSemiPositiveDefinite(Q));
  assert(eigen_tools::isSymmetricPositiveDefinite(R));
  assert(tol > 0.);

  const MatrixXd I = MatrixXd::Identity(n, n);
  MatrixXd X_prev = MatrixXd::Zero(n, n);
  MatrixXd X_next = MatrixXd::Identity(n, n);
  size_t iter = 0;

  while ((X_next - X_prev).norm() / X_next.norm() > tol)
  {
    X_prev = X_next;

    // 事前推定
    MatrixXd X_mid = A.transpose() * X_prev * A + Q;

    eigen_tools::symmetrise(X_mid);  // 対称性を保存

    // 事後推定
    auto XB = X_mid * B;
    auto G = XB * (B.transpose() * XB + R).inverse();
    auto I_GBt = I - G * B.transpose();

    switch (method)
    {
      case DareMethod::Normal:
      {
        X_next = I_GBt * X_mid;
        break;
      }
      case DareMethod::Joseph:
      {
        X_next = I_GBt * X_mid * I_GBt.transpose() + G * R * G.transpose();
        break;
      }
      default:
      {
        throw runtime_error("Unknown method ID: " + to_string(method));
      }
    }

    eigen_tools::symmetrise(X_next);  // 対称性を保存

    if (iter++ > max_iter)
    {
      throw runtime_error("Failed to converge");
    }
  }

  // cout << eigen_tools::matrixRank(X_next) << endl;
  assert(eigen_tools::isPositiveDefinite(X_next));

  cout << "DARE has successfully converged in " << iter << " iterations." << endl;
  return X_next;
}
}  // namespace ctrl
