// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/rotational_inertia.hpp"

#include <tobas_eigen_tools/linalg.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace kdl
{
std::expected<void, std::string> RotationalInertia::isValid() const
{
  // Check that the matrix is symmetric.
  if (!eigen::isSymmetric(data)) {
    return std::unexpected("Inertia matrix must be symmetric.");
  }

  // Compute the principal moments of inertia.
  const EigenSolver<Matrix3d> es(data);
  if (es.info() != Success) {
    return std::unexpected("Failed to get the eigenvalues of the inertia matrix.");
  }
  const auto eigvals = es.eigenvalues().real().eval();
  const auto& i1 = eigvals.x();
  const auto& i2 = eigvals.y();
  const auto& i3 = eigvals.z();

  // Check that the matrix is positive-semidefinite.
  if (i1 < 0.0 || i2 < 0.0 || i3 < 0.0) {
    return std::unexpected("Inertia matrix must be positive-semidefinite.");
  }

  // Check that the principal moments satisfy the triangle inequality.
  if (i1 + i2 < i3 || i2 + i3 < i1 || i3 + i1 < i2) {
    return std::unexpected("Inertia matrix is unrealistic.");
  }

  return {};
}
}  // namespace kdl
}  // namespace tobas
