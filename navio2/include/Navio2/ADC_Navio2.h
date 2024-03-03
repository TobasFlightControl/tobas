#pragma once

#include <cstddef>

class ADC_Navio2
{
  static constexpr size_t kChannelCount = 6;

public:
  explicit ADC_Navio2();

  int initialize();
  int read(const size_t& ch);

  inline const size_t& channelCount() const;

private:
  int channels_[kChannelCount];

  int openChannel(const size_t& ch);
};

inline const size_t& ADC_Navio2::channelCount() const
{
  return kChannelCount;
}
