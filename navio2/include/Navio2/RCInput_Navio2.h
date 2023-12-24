#pragma once

#include <cstddef>

#include "../Common/RCInput.h"

class RCInput_Navio2 : public RCInput
{
public:
  explicit RCInput_Navio2();
  void initialize() override;
  int read(int ch) override;

private:
  int open_channel(int ch);

  static const size_t CHANNEL_COUNT = 14;
  int channels[CHANNEL_COUNT];
};
