#pragma once

#include <cstddef>

class RCInput_Navio2
{
public:
  static constexpr size_t kChannelCount = 14;

  explicit RCInput_Navio2();

  int initialize();
  int read(const size_t& ch);

private:
  int channels_[kChannelCount];

  int openChannel(const size_t& ch);
};
