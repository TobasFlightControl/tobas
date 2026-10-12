// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/vector.hpp"

#include <tobas_math/core.hpp>

namespace tobas
{
namespace kdl
{
bool Vector::isParallel(const Vector& rhs, bool same_direction_only, double angle_tol_rad) const
{
  assert(angle_tol_rad > 0.0);

  const auto na2 = squaredNorm();
  const auto nb2 = rhs.squaredNorm();

  // Parallelism is undefined for zero vectors.
  [[maybe_unused]] constexpr double kZeroTol = 1e-12;
  assert(na2 > kZeroTol);
  assert(nb2 > kZeroTol);

  const auto dot = this->dot(rhs);
  const auto cos2 = math::sqr(dot) / (na2 * nb2);
  const auto thresh = math::sqr(std::cos(angle_tol_rad));

  if (same_direction_only) {
    return cos2 > thresh && dot > 0.0;
  }
  else {
    return cos2 > thresh;
  }
}
}  // namespace kdl
}  // namespace tobas
