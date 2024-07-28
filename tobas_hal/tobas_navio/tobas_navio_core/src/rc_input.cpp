#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include "../include/tobas_navio_core/rc_input.hpp"

using namespace std;

namespace navio
{
RCInput::RCInput()
{
}

bool RCInput::initialize()
{
  for (size_t ch = 0; ch < kChannelCount; ++ch)
  {
    char* channel_path;
    if (asprintf(&channel_path, "%s/ch%zu", kRcinSysfsPath, ch) == -1)
    {
      cerr << "Failed to open RC channel " << ch << "." << endl;
      return false;
    }

    channels_[ch] = ::open(channel_path, O_RDONLY);
    free(channel_path);
  }

  return true;
}

bool RCInput::read(const size_t& ch)
{
  if (ch >= kChannelCount)
  {
    cerr << "Channel number too large." << endl;
    return false;
  }

  if (::pread(channels_[ch], buffer_, kBufferSize, 0) < 0)
  {
    cerr << "Failed to read RC channel " << ch << "." << endl;
    return false;
  }

  period_ = atoi(buffer_);

  if (period_ == 0)
  {
    cerr << "RC signal is not received." << endl;
    return false;
  }

  return true;
}
}  // namespace navio
