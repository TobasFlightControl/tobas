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
  int openChannel(int ch);

  static const size_t kChannelCount = 6;
  int channels[kChannelCount];
};
