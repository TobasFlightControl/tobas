#pragma once

#include <cstddef>

namespace navio
{
class ADC
{
  static constexpr size_t kChannelCount = 6;

public:
  explicit ADC();

  int initialize();
  int read(const size_t& ch);

  inline static size_t channelCount()
  {
    return kChannelCount;
  }

private:
  int channels_[kChannelCount];
  char buffer_[10];

  int openChannel(const size_t& ch);
};
}  // namespace navio
