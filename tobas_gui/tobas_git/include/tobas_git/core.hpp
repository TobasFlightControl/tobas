// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas::git
{
std::expected<std::string, std::string> getGitConfigValue(const char* _key);
}  // namespace tobas::git
