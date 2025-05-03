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

namespace math
{
/* 複素数の3乗根を求める． */
std::complex<double> cbrt(const std::complex<double>& z);
}  // namespace math
