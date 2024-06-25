#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include <tobas_std_tools/console.hpp>

#include "../include/tobas_navio_core/util.hpp"
#include "../include/tobas_navio_core/rc_input.hpp"

using namespace std;

namespace navio
{
RCInput::RCInput()
{
  PRINT_DEBUG("RCInput::RCInput");
}

RCInput::error_t RCInput::initialize()
{
  PRINT_DEBUG("RCInput::initialize");

  for (size_t ch = 0; ch < kChannelCount; ++ch)
  {
    PRINT_DEBUG("Initializing RC input channel " << ch << ".");

    char* channel_path;
    if (asprintf(&channel_path, "%s/ch%zu", kRcinSysfsPath, ch) == -1)
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

  if (period_ == 0)
    return error_ = E_NOT_RECEIVED;

  return error_ = E_NO_ERROR;
}
}  // namespace navio
