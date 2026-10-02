// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_string_tools/stream.hpp"

#include <fstream>
#include <sstream>

namespace tobas
{
namespace str
{
bool readText(const std::string& path, std::string& text)
{
  std::ifstream file(path);
  if (!file) {
    return false;
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  file.close();

  text = oss.str();

  return true;
}

bool writeText(const std::string& path, const std::string& text)
{
  std::ofstream file(path);
  if (!file) {
    return false;
  }

  file << text;
  file.close();

  return true;
}
}  // namespace str
}  // namespace tobas
