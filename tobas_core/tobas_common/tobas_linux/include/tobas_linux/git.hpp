// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas
{
namespace linux
{
/** Return the Git user name. */
std::expected<std::string, std::string> getGitUserName();

/** Return the Git email address. */
std::expected<std::string, std::string> getGitUserEmail();
}  // namespace linux
}  // namespace tobas
