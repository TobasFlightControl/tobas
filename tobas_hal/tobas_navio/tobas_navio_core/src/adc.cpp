#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include "../include/tobas_navio_core/adc.hpp"

using namespace std;

namespace navio
{
ADC::ADC()
{
}

bool ADC::initialize()
{
  for (size_t ch = 0; ch < kChannelCount; ++ch)
  {
    channels_[ch] = openChannel(ch);
    if (channels_[ch] < 0)
    {
      cerr << "Failed to open ADC channel " << ch << "." << endl;
      return false;
    }
  }

  return true;
}

int ADC::read(const size_t& ch)
{
  if (ch >= kChannelCount)
  {
    cerr << "Channel number too large." << endl;
    return -1;
  }

  if (::pread(channels_[ch], buffer_, kBufferSize, 0) < 0)
  {
    cerr << "Failed to read ADC channel " << ch << "." << endl;
    return -1;
  }

  return atoi(buffer_);
}

int ADC::openChannel(const size_t& ch)
{
  char* channel_path;
  if (asprintf(&channel_path, "%s/ch%zu", kAdcSysfsPath, ch) == -1)
    return -1;

  const auto fd = ::open(channel_path, O_RDONLY);
  free(channel_path);

  return fd;
}
}  // namespace navio
