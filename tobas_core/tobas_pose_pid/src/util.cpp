// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "./util.hpp"

#include <iostream>

namespace tobas
{
bool checkIndex(int idx)
{
  if (idx < 0 || 3 <= idx) {
    std::cerr << "Index " << idx << " is out of range." << std::endl;
    return false;
  }

  return true;
}
}  // namespace tobas
