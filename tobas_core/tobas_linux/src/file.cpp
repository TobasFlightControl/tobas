#include <cstdio>
#include <cstdarg>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "../include/tobas_linux/file.hpp"

using namespace std;

namespace linux
{
int writeFile(const char* path, const char* fmt, ...)
{
  errno = 0;

  const auto fd = ::open(path, O_WRONLY | O_CLOEXEC);
  if (fd == -1) {
    return -errno;
  }

  va_list args;
  va_start(args, fmt);

  const auto ret = ::vdprintf(fd, fmt, args);
  const auto errno_bkp = errno;
  ::close(fd);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }

  return ret;
}

int readFile(const char* path, const char* fmt, ...)
{
  errno = 0;

  FILE* file = ::fopen(path, "re");
  if (!file) {
    return -errno;
  }

  va_list args;
  va_start(args, fmt);

  const auto ret = ::vfscanf(file, fmt, args);
  const auto errno_bkp = errno;
  ::fclose(file);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }

  return ret;
}
}  // namespace linux
