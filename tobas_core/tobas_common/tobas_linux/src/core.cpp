// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/core.hpp"

#include <unistd.h>

#include <cstdlib>

namespace fs = std::filesystem;

namespace tobas::linux
{
std::expected<std::string, std::string> userName() noexcept
{
  if (isSuperUser()) {
    return "root";
  }
  else {
    const auto user_name = std::getenv("USER");
    if (!user_name) {
      return std::unexpected("USER environment variable not set.");
    }
    return std::string(user_name);
  }
}

std::expected<fs::path, std::string> homeDir() noexcept
{
  if (isSuperUser()) {
    return "/root";
  }
  else {
    const auto home_dir = std::getenv("HOME");
    if (!home_dir) {
      return std::unexpected("HOME environment variable not set.");
    }
    return home_dir;
  }
}

std::expected<fs::path, std::string> expandUser(const std::string& _path) noexcept
{
  if (_path.substr(0, 2) == "~/") {
    const auto home = homeDir();
    if (!home) {
      return std::unexpected(home.error());
    }
    return *home / _path.substr(2);
  }
  else {
    return _path;
  }
}

bool isSuperUser() noexcept
{
  return getuid() == 0;
}
}  // namespace tobas::linux
