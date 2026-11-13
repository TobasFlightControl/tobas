// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <unistd.h>

#include <cstdlib>

#include <tobas_linux/schedule.hpp>

int main()
{
  if (!tobas::linux::setRealtimePriorityFIFO(50)) {
    return EXIT_FAILURE;
  }

  pause();

  return EXIT_SUCCESS;
}
