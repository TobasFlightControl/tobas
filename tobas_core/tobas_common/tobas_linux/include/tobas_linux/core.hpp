// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <string>

namespace tobas
{
namespace linux
{
/** Get the user name. */
std::expected<std::string, std::string> userName() noexcept;

/** Get the home directory. */
std::expected<std::filesystem::path, std::string> homeDir() noexcept;

/** Expand the home directory to an absolute path. */
std::expected<std::filesystem::path, std::string> expandUser(const std::string& path) noexcept;

/** Return true when the program is running with root privileges. */
bool isSuperUser() noexcept;
}  // namespace linux
}  // namespace tobas
