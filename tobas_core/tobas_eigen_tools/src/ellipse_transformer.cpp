#include "tobas_eigen_tools/ellipse_transformer.hpp"

#include <eigen3/Eigen/Eigen>

using namespace Eigen;

namespace eigen
{
EllipseTransformer::EllipseTransformer()
{
  setIdentity();
}

bool EllipseTransformer::initialize(const EllipseCoefficients& coefs)
{
  // 楕円の方程式: x^T A x + b^T x + c = 0
  Matrix3d A;
  A << coefs.a_xx, coefs.a_xy, coefs.a_zx, coefs.a_xy, coefs.a_yy, coefs.a_yz, coefs.a_zx, coefs.a_yz, coefs.a_zz;
  Vector3d b;
  b << coefs.b_x, coefs.b_y, coefs.b_z;

  // Aを対角化
  const SelfAdjointEigenSolver<Matrix3d> eigen_solver(A);
  const Vector3d Lam = eigen_solver.eigenvalues();
  const Matrix3d P = eigen_solver.eigenvectors();

  // 変換行列を計算
  const Vector3d Lam_inv = Lam.cwiseInverse();
  const double W = (P.transpose() * b).cwiseAbs2().cwiseProduct(Lam_inv).sum() / 4 - coefs.c;

  // 楕円体であるための条件チェック
  if (!((Lam * W).array() > 0.).all()) {
    return false;
  }

  const Matrix3d A_inv = P * Lam_inv.asDiagonal() * P.transpose();
  b_ = -0.5 * A_inv * b;

  const Vector3d S = (Lam / W).cwiseSqrt();
  T_inv_ = P * S.asDiagonal() * P.transpose();

  return true;
}

void EllipseTransformer::setIdentity()
{
  b_.setZero();
  T_inv_.setIdentity();
}

const Vector3d& EllipseTransformer::getHardBias() const
{
  return b_;
}

void EllipseTransformer::setHardBias(const Vector3d& b)
{
  b_ = b;
}

Vector6d EllipseTransformer::getSoftBias() const
{
  const Matrix3d T = T_inv_.inverse();
  return (Vector6d() << T(0, 0), T(1, 1), T(2, 2), T(0, 1), T(1, 2), T(2, 0)).finished();
}

void EllipseTransformer::setSoftBias(const Vector6d& t)
{
  const auto& txx = t(0);
  const auto& tyy = t(1);
  const auto& tzz = t(2);
  const auto& txy = t(3);
  const auto& tyz = t(4);
  const auto& tzx = t(5);
  const auto T = (Matrix3d() << txx, txy, tzx, txy, tyy, tyz, tzx, tyz, tzz).finished();
  T_inv_ = T.inverse();
}
}  // namespace eigen
