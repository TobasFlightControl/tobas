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
  for (size_t i = 0; i < kChannelCount; ++i)
  {
    channels_[i] = openChannel(i);
    if (channels_[i] < 0)
    {
      perror("open");
      return -1;
    }
  }

  return 0;
}

int RCInput::read(const size_t& ch)
{
  if (ch >= kChannelCount)
  {
    cerr << "Channel number too large." << endl;
    return -1;
  }

  char buffer[10];
  if (::pread(channels_[ch], buffer, ARRAY_SIZE(buffer), 0) < 0)
  {
    perror("pread");
    return -1;
  }

  return atoi(buffer);
}

int RCInput::openChannel(const size_t& ch)
{
  char* channel_path;
  if (asprintf(&channel_path, "%s/ch%zu", "/sys/kernel/rcio/rcin", ch) == -1)
  {
    err(1, "channel: %zu\n", ch);
    return -1;
  }

  const auto fd = ::open(channel_path, O_RDONLY);
  free(channel_path);

  return fd;
}
}
