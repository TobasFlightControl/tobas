// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_string_tools/stream.hpp"

#include <fstream>
#include <sstream>

namespace tobas::str
{
bool readText(const std::string& _path, std::string& _text)
{
  std::ifstream file(_path);
  if (!file) {
    return false;
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  file.close();

  _text = oss.str();

  return true;
}

bool writeText(const std::string& _path, const std::string& _text)
{
  std::ofstream file(_path);
  if (!file) {
    return false;
  }

  file << _text;
  file.close();

  return true;
}
}  // namespace tobas::str
