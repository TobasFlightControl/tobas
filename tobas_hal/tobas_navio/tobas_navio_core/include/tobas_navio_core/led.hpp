#pragma once

#include "./gpio.hpp"

namespace navio
{
enum class Colors
{
  Black,
  Red,
  Green,
  Blue,
  Cyan,
  Magenta,
  Yellow,
  White
};

class Led
{
  static constexpr uint8_t kOnValue = 0;
  static constexpr uint8_t kOffValue = 1;

public:
  explicit Led();

  bool initialize();
  void setColor(Colors color);

private:
  Pin pin_R_;
  Pin pin_G_;
  Pin pin_B_;
};
}  // namespace navio
