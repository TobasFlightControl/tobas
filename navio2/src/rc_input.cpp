#include <cstdio>
#include <cstdlib>
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

int RCInput::initialize()
{
  for (size_t ch = 0; ch < kChannelCount; ++ch)
  {
    channels_[ch] = openChannel(ch);
    if (channels_[ch] < 0)
    {
      cerr << "Failed to open RC input channel " << ch << "." << endl;
      return -1;
    }
  }

  return 0;
}

int RCInput::read(const size_t& ch)
{
  if (ch >= kChannelCount)
  {
    cerr << "Channel number too large: " << ch << endl;
    return -1;
  }

  if (::pread(channels_[ch], buffer_, ARRAY_SIZE(buffer_), 0) < 0)
  {
    cerr << "Failed to read RC input channel " << ch << "." << endl;
    return -1;
  }

  const auto period = atoi(buffer_);
  if (period < kValidPeriodMin || kValidPeriodMax < period)
  {
    cerr << "PWM period of channel " << ch << " is out of range: " << period << endl;
    return -1;
  }

  return period;
}

int RCInput::openChannel(const size_t& ch)
{
  char* channel_path;
  if (asprintf(&channel_path, "%s/ch%zu", "/sys/kernel/rcio/rcin", ch) == -1)
  {
    cerr << "Failed to open RC input channel " << ch << "." << endl;
    return -1;
  }

  const auto fd = ::open(channel_path, O_RDONLY);
  free(channel_path);

  return fd;
}
}  // namespace navio
