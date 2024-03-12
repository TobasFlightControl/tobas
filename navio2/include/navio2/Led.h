#pragma once

#include "./gpio.h"

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
  Navio::Pin* pinR;
  Navio::Pin* pinG;
  Navio::Pin* pinB;
};
