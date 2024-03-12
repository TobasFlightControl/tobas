#pragma once

#include <cstddef>

namespace navio
{
class RCInput
{
public:
  static constexpr size_t kChannelCount = 14;

  explicit RCInput();

  int initialize();
  int read(const size_t& ch);

private:
  int channels_[kChannelCount];

  int openChannel(const size_t& ch);
};
}
