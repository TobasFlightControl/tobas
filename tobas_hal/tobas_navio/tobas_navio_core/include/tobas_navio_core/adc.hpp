#pragma once

#include <cstddef>

namespace navio
{
class ADC
{
  static constexpr size_t kChannelCount = 6;
  static constexpr size_t kBufferSize = 10;
  static constexpr char kAdcSysfsPath[] = "/sys/kernel/rcio/adc";

public:
  explicit ADC();

  bool initialize();
  int read(const size_t& ch);

  inline static size_t channelCount()
  {
    return kChannelCount;
  }

private:
  int channels_[kChannelCount];
  char buffer_[kBufferSize];

  int openChannel(const size_t& ch);
};
}  // namespace navio
