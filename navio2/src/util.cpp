#include <cstdio>
#include <cmath>
#include <cinttypes>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "../include/navio2/util.hpp"

using namespace std;

namespace navio
{
int write_file(const char* path, const char* fmt, ...)
{
  errno = 0;

  const auto fd = ::open(path, O_WRONLY | O_CLOEXEC);
  if (fd == -1)
    return -errno;

  va_list args;
  va_start(args, fmt);

  const auto ret = ::vdprintf(fd, fmt, args);
  const auto errno_bkp = errno;
  ::close(fd);

  va_end(args);

  if (ret < 1)
    return -errno_bkp;

  return ret;
}

int read_file(const char* path, const char* fmt, ...)
{
  errno = 0;

  FILE* file = ::fopen(path, "re");
  if (!file)
    return -errno;

  va_list args;
  va_start(args, fmt);

  const auto ret = ::vfscanf(file, fmt, args);
  const auto errno_bkp = errno;
  ::fclose(file);

  va_end(args);

  if (ret < 1)
    return -errno_bkp;

  return ret;
}

bool check_apm()
{
  const auto ret = system("ps -AT | grep -c ap-timer > /dev/null");

  if (WEXITSTATUS(ret) <= 0)
  {
    fprintf(stderr, "APM is running. Can't launch the example\n");
    return true;
  }

  return false;
}

int get_navio_version()
{
  int version;
  read_file("/sys/firmware/devicetree/base/hat/product_id", "%x", &version);
  return version;
}

float decodeBinary32(uint32_t bin)
{
  const auto sign = bin >> 31 ? -1 : 1;
  const auto exponent = (bin >> 23) & 0xFF;
  const auto mantissa = (exponent == 0) ? (bin & 0x7FFFFF) << 1 : (bin & 0x7FFFFF) | 0x800000;
  return sign * mantissa * pow(2, exponent - 150);
}
}  // namespace navio
