#pragma once

#include <cstdint>

namespace qt
{
class RGBColor
{
public:
  uint8_t r, g, b;

  constexpr explicit RGBColor(uint8_t _r, uint8_t _g, uint8_t _b);

  constexpr static RGBColor Black();
  constexpr static RGBColor White();
  constexpr static RGBColor Gray();
  constexpr static RGBColor Blue();
  constexpr static RGBColor Green();
  constexpr static RGBColor Orange();
  constexpr static RGBColor Purple();
  constexpr static RGBColor Red();
  constexpr static RGBColor Yellow();

  RGBColor mean(const RGBColor& other) const;

  bool operator==(const RGBColor& rhs) const;
};

constexpr RGBColor::RGBColor(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b)
{
}

constexpr RGBColor RGBColor::Black()
{
  return RGBColor(0, 0, 0);
}

constexpr RGBColor RGBColor::Gray()
{
  return RGBColor(128, 128, 128);
}

constexpr RGBColor RGBColor::White()
{
  return RGBColor(255, 255, 255);
}

constexpr RGBColor RGBColor::Blue()
{
  return RGBColor(115, 206, 244);
}

constexpr RGBColor RGBColor::Green()
{
  return RGBColor(173, 255, 47);
}

constexpr RGBColor RGBColor::Orange()
{
  return RGBColor(255, 165, 0);
}

constexpr RGBColor RGBColor::Purple()
{
  return RGBColor(175, 0, 255);
}

constexpr RGBColor RGBColor::Red()
{
  return RGBColor(244, 55, 83);
}

constexpr RGBColor RGBColor::Yellow()
{
  return RGBColor(255, 255, 0);
}
}  // namespace qt
