#include <iostream>
#include <Eigen/Eigen>

#include "../include/tobas_real/ellipse_transformer.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
EllipseTransformer::EllipseTransformer()
{
}

void EllipseTransformer::initialize()
{
  // 楕円の方程式: x^T A x + b^T x + c = 0
  Matrix3d A;
  A << a_xx, a_xy, a_zx, a_xy, a_yy, a_yz, a_zx, a_yz, a_zz;
  Vector3d b;
  b << b_x, b_y, b_z;

  // Aを対角化
  const SelfAdjointEigenSolver<Matrix3d> eigen_solver(A);
  const Vector3d Lam = eigen_solver.eigenvalues();
  const Matrix3d P = eigen_solver.eigenvectors();
  cout << "Eigenvalues:\n" << Lam << endl;
  cout << "Eigenvectors:\n" << P << endl;

  // 変換行列を計算
  const Vector3d Lam_inv = Lam.cwiseInverse();
  const double W = (P.transpose() * b).cwiseAbs2().cwiseProduct(Lam_inv).sum() / 4 - c;

  // 楕円体であるための条件チェック
  if (!((Lam * W).array() > 0.).all())
  {
    throw runtime_error("It cannot be an ellipsoid with the given coefficients.");
  }

  xc_ = -0.5 * P * Lam_inv.asDiagonal() * P.transpose() * b;
  S_ = (Lam / W).cwiseSqrt();
  PSPt_ = P * S_.asDiagonal() * P.transpose();
}

Vector3d EllipseTransformer::transform(const Vector3d& x) const
{
  return PSPt_ * (x - xc_);
}

const Vector3d& EllipseTransformer::getCenter() const
{
  return xc_;
}

const Vector3d& EllipseTransformer::getRadius() const
{
  return S_;
}

ostream& operator<<(ostream& os, const EllipseTransformer& arg)
{
  os << "a_xx: " << arg.a_xx << endl;
  os << "a_yy: " << arg.a_yy << endl;
  os << "a_zz: " << arg.a_zz << endl;
  os << "a_xy: " << arg.a_xy << endl;
  os << "a_yz: " << arg.a_yz << endl;
  os << "a_zx: " << arg.a_zx << endl;
  os << "b_x : " << arg.b_x << endl;
  os << "b_y : " << arg.b_y << endl;
  os << "b_z : " << arg.b_z << endl;
  os << "c   : " << arg.c << endl;

  return os;
}
}  // namespace tobas_real
