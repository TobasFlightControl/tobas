#pragma once

#include <cstddef>

namespace navio
{
class RCInput
{
  static constexpr size_t kChannelCount = 14;
  static constexpr int kValidPeriodMin = 900;   // [us]
  static constexpr int kValidPeriodMax = 2100;  // [us]

public:
  explicit RCInput();

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
