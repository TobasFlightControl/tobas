#pragma once

#include <cstddef>

namespace navio
{
class ADC
{
public:
  static constexpr size_t kChannelCount = 6;

  explicit ADC();

  int initialize();
  int read(const size_t& ch);

private:
  int channels_[kChannelCount];

  int openChannel(const size_t& ch);
};
}  // namespace navio
