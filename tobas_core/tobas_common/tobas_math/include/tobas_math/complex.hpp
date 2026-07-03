// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <complex>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::complex<T>& z)
{
  os << z.real();
  if (z.imag() >= 0) {
    os << " + " << z.imag() << "i";
  }
  else {
    os << " - " << -z.imag() << "i";
  }
  return os;
}

namespace tobas
{
namespace math
{
/* Calculate the cube root of a complex number. */
std::complex<double> cbrt(const std::complex<double>& z);
}  // namespace math
}  // namespace tobas
