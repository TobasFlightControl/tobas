#pragma once

#include <cstddef>
#include <cinttypes>

namespace navio
{
class RCInput
{
  static constexpr size_t kChannelCount = 14;

public:
  enum error_t : int8_t
  {
    E_NO_ERROR = 0,
    E_FAILED_TO_OPEN = -1,
    E_FAILED_TO_READ = -2,
    E_NOT_RECEIVED = -3,
  };

  explicit RCInput();

  error_t initialize();
  error_t read(const size_t& ch);

  /* Get the latest error code. */
  inline error_t getError() const
  {
    return error_;
  }

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
  error_t error_;
  int period_;
  int channels_[kChannelCount];
  char buffer_[10];
};
}  // namespace navio
