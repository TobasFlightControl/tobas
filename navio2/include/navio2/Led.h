#pragma once

#include "./gpio.h"

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
public:
  explicit Led();

  bool initialize();
  void setColor(Colors color);

private:
  Pin* pin_R_;
  Pin* pin_G_;
  Pin* pin_B_;
};
}  // namespace navio
