#pragma once

#include <cstddef>

#include "../Common/ADC.h"

class ADC_Navio2 : public ADC
{
public:
  explicit ADC_Navio2();
  void initialize() override;
  int get_channel_count(void) override;
  int read(int ch) override;

private:
  int open_channel(int ch);

  static const size_t CHANNEL_COUNT = 6;
  int channels[CHANNEL_COUNT];
};
