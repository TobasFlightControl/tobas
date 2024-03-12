#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <err.h>

#include "../include/navio2/Util.h"
#include "../include/navio2/ADC.h"

#define ADC_SYSFS_PATH "/sys/kernel/rcio/adc"

using namespace std;

namespace navio
{
ADC::ADC()
{
}

int ADC::initialize()
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

int ADC::read(const size_t& ch)
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

int ADC::openChannel(const size_t& ch)
{
  char* channel_path;
  if (asprintf(&channel_path, "%s/ch%zu", ADC_SYSFS_PATH, ch) == -1)
  {
    err(1, "adc channel: %zu\n", ch);
    return -1;
  }

  const auto fd = ::open(channel_path, O_RDONLY);
  free(channel_path);

  return fd;
}
}
