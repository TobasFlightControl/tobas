#pragma once

#include <cinttypes>

namespace qt
{
class RGBColor
{
public:
  uint8_t r, g, b;

  constexpr explicit RGBColor(uint8_t _r, uint8_t _g, uint8_t _b);

  constexpr static RGBColor Black();
  constexpr static RGBColor White();
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
  return RGBColor(0x00, 0x00, 0x00);
}

constexpr RGBColor RGBColor::White()
{
  return RGBColor(0xFF, 0xFF, 0xFF);
}

constexpr RGBColor RGBColor::Blue()
{
  return RGBColor(0x73, 0xCE, 0xF4);
}

constexpr RGBColor RGBColor::Green()
{
  return RGBColor(0xAD, 0xFF, 0x2F);
}

constexpr RGBColor RGBColor::Orange()
{
  return RGBColor(0xFF, 0xA5, 0x00);
}

constexpr RGBColor RGBColor::Purple()
{
  return RGBColor(0xAF, 0x00, 0xFF);
}

constexpr RGBColor RGBColor::Red()
{
  return RGBColor(0xF4, 0x37, 0x53);
}

constexpr RGBColor RGBColor::Yellow()
{
  return RGBColor(0xFF, 0xFF, 0x00);
}
}  // namespace qt
