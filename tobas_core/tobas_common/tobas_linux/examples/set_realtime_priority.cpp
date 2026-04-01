// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <unistd.h>

#include <iostream>

#include <tobas_linux/schedule.hpp>

using namespace std;

int main()
{
  if (!tobas::linux::setRealtimePriorityFIFO(50)) {
    return EXIT_FAILURE;
  }

  pause();

  return EXIT_SUCCESS;
}
