#include "tobas_qt_tools/rgb_color.hpp"

namespace qt
{
RGBColor RGBColor::mean(const RGBColor& other) const
{
  // オーバーフローを避けるために一度型変換してから平均を計算する
  const auto r_mean = (uint8_t)(((int)r + (int)other.r) / 2);
  const auto g_mean = (uint8_t)(((int)g + (int)other.g) / 2);
  const auto b_mean = (uint8_t)(((int)b + (int)other.b) / 2);
  return RGBColor(r_mean, g_mean, b_mean);
}

bool RGBColor::operator==(const RGBColor& rhs) const
{
  return r == rhs.r && g == rhs.g && b == rhs.b;
}
}  // namespace qt
