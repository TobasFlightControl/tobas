#pragma once

#include <cstddef>

class RCInput_Navio2
{
  static constexpr size_t kChannelCount = 14;

public:
  explicit RCInput_Navio2();

  int initialize();
  int read(const size_t& ch);

  inline const size_t& channelCount() const;

private:
  int channels_[kChannelCount];

  int openChannel(const size_t& ch);
};

inline const size_t& RCInput_Navio2::channelCount() const
{
  return kChannelCount;
}
