#include <Eigen/Eigen>

#include "../../include/tobas_real/ellipse_transformer.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
EllipseTransformer::EllipseTransformer()
{
}

void EllipseTransformer::initialize()
{
  // 楕円の方程式: x^T A x + b^T x + c = 0 (c = 1)
  Matrix3f A;
  A << a_xx, a_xy, a_zx, a_xy, a_yy, a_yz, a_zx, a_yz, a_zz;
  Vector3f b;
  b << b_x, b_y, b_z;

  // Aを対角化
  const SelfAdjointEigenSolver<Matrix3f> eigen_solver(A);
  const Vector3f Lam = eigen_solver.eigenvalues();
  const Matrix3f P = eigen_solver.eigenvectors();

  // 変換行列を計算
  constexpr float c = 1.;
  const Vector3f Lam_inv = Lam.cwiseInverse();
  const float W = (P.transpose() * b).cwiseAbs2().cwiseProduct(Lam_inv).sum() / 4 - c;

  // 楕円体であるための条件チェック
  if (!((Lam * W).array() > 0.).all())
  {
    throw runtime_error("It cannot be an ellipsoid with the given coefficients.");
  }

  const Vector3f S = (Lam / W).cwiseSqrt();
  const Matrix3f PS = P * S.asDiagonal();
  A_ = PS * P.transpose();
  b_ = 0.5 * PS * Lam_inv.asDiagonal() * P.transpose() * b;
}

Vector3f EllipseTransformer::transform(const Vector3f& mag_raw)
{
  return A_ * mag_raw + b_;
}
}  // namespace tobas_real
