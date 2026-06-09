// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_std_tools/cmdline_parser.hpp"

#include <cstddef>
#include <cstring>
#include <iostream>

namespace tobas
{
namespace st
{
bool commandLineOptionExists(char** begin, char** end, const char* option)
{
  for (size_t i = 0; i < (size_t)(end - begin); ++i) {
    if (std::strcmp(begin[i], option) == 0) {
      return true;
    }
  }
  return false;
}

char* getCommandLineOption(char** begin, char** end, const char* option)
{
  size_t idx = 0;
  size_t end_idx = (size_t)(end - begin);
  for (; idx < end_idx; ++idx) {
    if (std::strncmp(begin[idx], option, std::strlen(option)) == 0) {
      break;
    }
  }

  if (idx < end_idx - 1 && !begin[idx++]) {
    std::cerr << "Command line option \"" << option << "\" does not exist." << std::endl;
    return nullptr;
  }

  return begin[idx];
}
}  // namespace st
}  // namespace tobas
