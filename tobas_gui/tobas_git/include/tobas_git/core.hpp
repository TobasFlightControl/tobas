// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace git
{
std::string getGitConfigValue(const char* key);
}  // namespace git
}  // namespace tobas
