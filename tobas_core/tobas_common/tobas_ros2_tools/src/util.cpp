// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/util.hpp"

#include <cstring>

#include <rcutils/env.h>
#include <rcutils/filesystem.h>

namespace fs = std::filesystem;

namespace tobas
{
namespace ros2
{
std::expected<const char*, std::string> getEnv(const char* _name)
{
  const char* value = nullptr;
  const char* error = rcutils_get_env(_name, &value);

  if (error) {
    return std::unexpected(std::string(error));
  }

  if (std::strlen(value) == 0) {
    return std::unexpected("Environment variable '" + std::string(_name) + "' is not set.");
  }

  return value;
}

std::expected<const char*, std::string> getUserName()
{
  return getEnv("USER");
}

fs::path expandUser(const char* _path)
{
  return rcutils_expand_user(_path, rcutils_get_default_allocator());
}
}  // namespace ros2
}  // namespace tobas
