// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eskf/util.hpp"

#include <tobas_math/core.hpp>

namespace tobas
{
namespace eskf
{
double headingVarianceFromMag(const Eigen::Vector3d& mag, const Eigen::Matrix3d& cov)
{
  const auto mx = mag.x();
  const auto my = mag.y();
  const auto mx_std = std::sqrt(cov(0, 0));
  const auto my_std = std::sqrt(cov(1, 1));
  const auto head_std = (std::abs(mx) * my_std + std::abs(my) * mx_std) / (math::sqr(mx) + math::sqr(my));
  return math::sqr(head_std);
}
}  // namespace eskf
}  // namespace tobas
