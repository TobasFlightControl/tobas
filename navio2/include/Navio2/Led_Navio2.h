#pragma once

#include "../Common/Led.h"
#include "./RGBled.h"

class Led_Navio2 : public Led
{
public:
  explicit Led_Navio2();
  bool initialize() override;
  void setColor(Colors color) override;

private:
  RGBled led;
};
