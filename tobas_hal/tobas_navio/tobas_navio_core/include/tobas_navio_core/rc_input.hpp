#pragma once

#include <cstddef>
#include <cinttypes>

namespace navio
{
class RCInput
{
  static constexpr size_t kChannelCount = 14;
  static constexpr size_t kBufferSize = 10;
  static constexpr char kRcinSysfsPath[] = "/sys/kernel/rcio/rcin";

public:
  explicit RCInput();

  bool initialize();
  bool read(const size_t& ch);

  /* Get the latest PWM period. */
  inline int getPeriod() const
  {
    return period_;
  }

  /* Get the number of channels. */
  inline static constexpr size_t channelCount()
  {
    return kChannelCount;
  }

private:
  int period_;
  int channels_[kChannelCount];
  char buffer_[kBufferSize];
};
}  // namespace navio
