// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas
{
namespace linux
{
/* Execute a shell command and return its output or an error description. */
std::expected<std::string, std::string> executeCommand(std::string command);
}  // namespace linux
}  // namespace tobas
