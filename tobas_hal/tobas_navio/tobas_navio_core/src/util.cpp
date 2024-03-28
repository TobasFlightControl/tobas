#include <cstdio>
#include <cmath>
#include <cinttypes>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "../include/tobas_navio_core/util.hpp"

using namespace std;

namespace navio
{
int writeFile(const char* path, const char* fmt, ...)
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

int readFile(const char* path, const char* fmt, ...)
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

int getNavioVersion()
{
  int version;
  readFile("/sys/firmware/devicetree/base/hat/product_id", "%x", &version);
  return version;
}

bool checkAPM()
{
  const auto ret = system("ps -AT | grep -c ap-timer > /dev/null");

  if (WEXITSTATUS(ret) <= 0)
  {
    fprintf(stderr, "APM is running. Can't launch the example\n");
    return true;
  }

  return false;
}

float decodeBinary32(uint32_t bin)
{
  const int sign = bin >> 31 ? -1 : 1;
  const int exponent = (bin >> 23) & 0xFF;
  const int mantissa = (exponent == 0) ? (bin & 0x7FFFFF) << 1 : (bin & 0x7FFFFF) | 0x800000;
  return sign * mantissa * pow(2, exponent - 150);
}
}  // namespace navio
