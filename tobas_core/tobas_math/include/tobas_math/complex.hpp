#pragma once

#include <complex>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::complex<T>& c)
{
  os << c.real();
  if (c.imag() >= 0)
    os << " + " << c.imag() << "i";
  else
    os << " - " << -c.imag() << "i";
  return os;
}
