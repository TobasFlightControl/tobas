// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

namespace tobas
{
namespace linux
{
/* Get the user name. */
std::string userName();

/* Get the home directory. */
std::filesystem::path homeDir();

/* Expand the home directory to an absolute path. */
std::filesystem::path expandUser(const std::string& path);

/* Return true when the program is running with root privileges. */
bool isSuperUser() noexcept;
}  // namespace linux
}  // namespace tobas
