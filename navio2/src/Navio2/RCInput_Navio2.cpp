#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <err.h>

#include "../../include/Common/Util.h"
#include "../../include/Navio2/RCInput_Navio2.h"

RCInput_Navio2::RCInput_Navio2()
{
}

void RCInput_Navio2::initialize()
{
  for (size_t i = 0; i < ARRAY_SIZE(channels_); ++i)
  {
    channels_[i] = openChannel(i);
    if (channels_[i] < 0)
      perror("open");
  }
}

int RCInput_Navio2::read(int ch)
{
  if (static_cast<size_t>(ch) > ARRAY_SIZE(channels_))
  {
    fprintf(stderr, "Channel number too large\n");
    return -1;
  }

  char buffer[10];

  if (::pread(channels_[ch], buffer, ARRAY_SIZE(buffer), 0) < 0)
    perror("pread");

  return atoi(buffer);
}

int RCInput_Navio2::openChannel(int channel)
{
  char* channel_path;
  if (asprintf(&channel_path, "%s/ch%d", "/sys/kernel/rcio/rcin", channel) == -1)
    err(1, "channel: %d\n", channel);

  const auto fd = ::open(channel_path, O_RDONLY);

  free(channel_path);

  return fd;
}
