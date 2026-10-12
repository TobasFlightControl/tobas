// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <complex>

template <typename T>
std::ostream& operator<<(std::ostream& _os, const std::complex<T>& _z)
{
  _os << _z.real();
  if (_z.imag() >= 0) {
    _os << " + " << _z.imag() << "i";
  }
  else {
    _os << " - " << -_z.imag() << "i";
  }
  return _os;
}

namespace tobas
{
namespace math
{
/** Calculate the cube root of a complex number. */
std::complex<double> cbrt(const std::complex<double>& _z);
}  // namespace math
}  // namespace tobas
