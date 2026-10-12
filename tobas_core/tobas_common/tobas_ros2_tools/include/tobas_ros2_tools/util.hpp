// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <string>

namespace tobas
{
namespace ros2
{
std::expected<const char*, std::string> getEnv(const char* name);

std::expected<const char*, std::string> getUserName();

std::filesystem::path expandUser(const char* path);
}  // namespace ros2
}  // namespace tobas
