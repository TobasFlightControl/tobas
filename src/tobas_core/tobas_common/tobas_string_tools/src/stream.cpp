// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_string_tools/stream.hpp"

#include <fstream>

namespace tobas::str
{
std::expected<std::string, std::string> readText(const std::string& _path)
{
  std::ifstream file(_path);
  if (!file) {
    return std::unexpected("Failed to open file for reading: " + _path);
  }

  std::string text;
  char buffer[1 << 12];
  while (file.read(buffer, sizeof(buffer)) || file.gcount() > 0) {
    text.append(buffer, file.gcount());
  }
  if (file.bad() || !file.eof()) {
    return std::unexpected("Failed to read file: " + _path);
  }

  return text;
}

std::expected<void, std::string> writeText(const std::string& _path, const std::string& _text)
{
  std::ofstream file(_path);
  if (!file) {
    return std::unexpected("Failed to open file for writing: " + _path);
  }

  file << _text;
  file.close();
  if (!file) {
    return std::unexpected("Failed to write file: " + _path);
  }

  return {};
}
}  // namespace tobas::str
