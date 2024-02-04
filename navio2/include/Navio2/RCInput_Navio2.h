#pragma once

#include <cstddef>

class RCInput_Navio2
{
  static constexpr size_t kChannelCount = 14;

public:
  explicit RCInput_Navio2();

  void initialize();
  int read(const size_t& ch);

private:
  int channels_[kChannelCount];

  int openChannel(const size_t& ch);
};
