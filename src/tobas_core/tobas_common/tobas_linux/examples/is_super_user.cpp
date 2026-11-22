// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_linux/core.hpp>

int main()
{
  if (tobas::linux::isSuperUser()) {
    std::cout << "Running with super privileges.\n";
  }
  else {
    std::cout << "Running with regular privileges.\n";
  }
}
