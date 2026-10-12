// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/file.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>

namespace tobas
{
namespace linux
{
int writeFile(const char* _path, const char* _fmt, ...)
{
  errno = 0;

  const auto fd = ::open(_path, O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    return -errno;
  }

  va_list args;
  va_start(args, _fmt);

  const auto ret = ::vdprintf(fd, _fmt, args);
  const auto errno_bkp = errno;
  ::close(fd);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }

  return ret;
}

int readFile(const char* _path, const char* _fmt, ...)
{
  errno = 0;

  FILE* file = ::fopen(_path, "re");
  if (!file) {
    return -errno;
  }

  va_list args;
  va_start(args, _fmt);

  const auto ret = ::vfscanf(file, _fmt, args);
  const auto errno_bkp = errno;
  ::fclose(file);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }

  return ret;
}
}  // namespace linux
}  // namespace tobas
