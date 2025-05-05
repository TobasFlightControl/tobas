#include "tobas_math/complex.hpp"

namespace math
{
std::complex<double> cbrt(const std::complex<double>& z)
{
  const auto r = std::abs(z);      // z の絶対値（極座標の r）
  const auto theta = std::arg(z);  // z の偏角（極座標の θ）

  const auto cbrt_r = std::cbrt(r);   // r の立方根
  const auto cbrt_theta = theta / 3;  // θ を 3 で割る

  return std::polar(cbrt_r, cbrt_theta);  // 極座標から複素数に変換
}
}  // namespace math
