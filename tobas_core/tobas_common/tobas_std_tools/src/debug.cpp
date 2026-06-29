// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_std_tools/debug.hpp"

#include <iostream>

#include "tobas_std_tools/ansi_text_styles.hpp"

namespace tobas
{
namespace st
{
void _printLocation(const char* file, int line)
{
  std::cout << GREEN_PREFIX << "Called from file " << file << ", line " << line << COLOR_RESET << std::endl;
}
}  // namespace st
}  // namespace tobas
