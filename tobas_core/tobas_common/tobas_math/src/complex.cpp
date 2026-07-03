// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_math/complex.hpp"

namespace tobas
{
namespace math
{
std::complex<double> cbrt(const std::complex<double>& z)
{
  const auto r = std::abs(z);      // Absolute value of `z`, or `r` in polar coordinates.
  const auto theta = std::arg(z);  // Argument of `z`, or `theta` in polar coordinates.

  const auto cbrt_r = std::cbrt(r);   // Cube root of `r`.
  const auto cbrt_theta = theta / 3;  // Divide `theta` by 3.

  return std::polar(cbrt_r, cbrt_theta);  // Convert from polar coordinates to a complex number.
}
}  // namespace math
}  // namespace tobas
