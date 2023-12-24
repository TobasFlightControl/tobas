#pragma once

#include "../Common/gpio.h"
#include "../Common/Led.h"

class RGBled
{
public:
  explicit RGBled();

  bool initialize();
  void setColor(Colors color);

private:
  Navio::Pin* pinR;
  Navio::Pin* pinG;
  Navio::Pin* pinB;
};
