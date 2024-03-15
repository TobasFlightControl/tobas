#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <err.h>

#include "../include/navio2/util.hpp"
#include "../include/navio2/rc_input.hpp"

using namespace std;

namespace navio
{
RCInput::RCInput()
{
}

RCInput::error_t RCInput::initialize()
{
  for (size_t ch = 0; ch < kChannelCount; ++ch)
  {
    char* channel_path;
    if (asprintf(&channel_path, "%s/ch%zu", "/sys/kernel/rcio/rcin", ch) == -1)
      return error_ = E_FAILED_TO_OPEN;

    channels_[ch] = ::open(channel_path, O_RDONLY);
    free(channel_path);
  }

  return error_ = E_NO_ERROR;
}

RCInput::error_t RCInput::read(const size_t& ch)
{
  assert(ch < kChannelCount);

  if (::pread(channels_[ch], buffer_, ARRAY_SIZE(buffer_), 0) < 0)
    return error_ = E_FAILED_TO_READ;

  period_ = atoi(buffer_);

  if (period_ < kValidPeriodMin || kValidPeriodMax < period_)
    return error_ = E_INVALID_PERIOD;

  return error_ = E_NO_ERROR;
}
}  // namespace navio
