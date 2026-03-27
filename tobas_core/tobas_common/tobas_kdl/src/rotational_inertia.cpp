#include "tobas_kdl/rotational_inertia.hpp"

#include <tobas_eigen_tools/linalg.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace kdl
{
bool RotationalInertia::isValid(string& error_msg) const
{
  // 対称行列であることを確認
  if (!eigen::isSymmetric(data)) {
    error_msg = "Inertia matrix must be symmetric.";
    return false;
  }

  // 慣性主軸を求める
  const EigenSolver<Matrix3d> es(data);
  if (es.info() != Success) {
    error_msg = "Failed to get the eigenvalues of the inertia matrix.";
    return false;
  }
  const auto eigvals = es.eigenvalues().real().eval();
  const auto& i1 = eigvals.x();
  const auto& i2 = eigvals.y();
  const auto& i3 = eigvals.z();

  // 正定行列であることを確認
  if (i1 <= 0. || i2 <= 0. || i3 <= 0.) {
    error_msg = "Inertia matrix must be positive-definite.";
    return false;
  }

  // 対角成分の三角不等式を満たすことを確認
  if (i1 + i2 <= i3 || i2 + i3 <= i1 || i3 + i1 <= i2) {
    error_msg = "Inertia matrix is unrealistic.";
    return false;
  }

  return true;
}
}  // namespace kdl
}  // namespace tobas
